/*****************************************************************************
* | File      	:   FluidFace.cpp
* | Function    :   Face implementation that renders the FLIP/PIC water
*                    simulation (Fluid.h/Fluid.cpp) in a single solid color.
******************************************************************************/
#include "FluidFace.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <span>

extern "C" {
#include "WS2812.h" // for LED_BRIGHTNESS, the hardware safety brightness cap
}

// Converts accelerometer tilt (in g) into simulation gravity units. Tuned by
// eye against the 10-unit-wide sim grid; raise for a snappier reaction.
static constexpr float kGravityScale = 1500.0f;

// A cell needs to reach at least this fraction of the settled "full water"
// density before its LED turns on -- e.g. 0.25 means a cell must be at
// least a quarter as dense as fully-settled water. Raise it to shrink the
// wet area down to only the densest parts of the water; lower it (down to
// 0.0) to light up on the faintest trace of water.
static constexpr float kWaterOnThreshold = 0.25f;

// A per-sample tilt change (in g) above this counts as real motion, which
// cancels calm mode immediately and restarts the calm timer. Deliberately
// well above the jitter of a pendant worn while walking/moving around --
// this should only trip on someone actually holding and tilting it, not on
// ordinary body motion while it hangs.
static constexpr float kMotionThreshold = 0.2f;

// How long tilt must stay below kMotionThreshold before calm mode engages.
static constexpr float kCalmDurationS = 2.0f;

// Calm mode engagement fades in over this long once triggered; exiting is
// immediate instead (see getFrame), so the water reacts right away the
// moment it's actually moved.
static constexpr float kCalmModeBlendInS = 2.0f;

// Orbital angular speed of the arc, radians/sec -- slow enough that the
// water can actually keep up with the arc instead of it sweeping past
// before particles catch up (which just leaves them hovering in a blob at
// the center rather than getting pulled out along the arc's curve).
static constexpr float kCalmModeAngularSpeed = 2.5f;

// Orbit radius, in Fluid's grid-cell units (see Fluid::gridWidth()/
// gridHeight()).
static constexpr float kCalmModeOrbitRadius = 2.75f;

// Not a private constant here to avoid relying on M_PI being available
// (see HeartFace.cpp for the identical reasoning).
static constexpr float kPi = 3.14;

// The attractor points are spread across this much of the orbit circle --
// a 270-degree arc, i.e. a "C" shape open on one side -- rather than
// clustered at one spot, so the pull reads as a curved stretched line
// sweeping around the center instead of a round blob.
static constexpr float kCalmModeArcSpanRadians = 1.0f * kPi;
static constexpr int kCalmModeNumAttractors = 15; // enough to cover the arc without gaps

// Attractor pull strength at full blend-in, same units as gravityX_/
// gravityY_ above. Strong enough that particles actually get pulled taut
// along the arc's curve rather than just clumping into a blob near
// whichever attractor happens to be nearest.
static constexpr float kCalmModeAttractorStrength = 2000.0f;

// An attractor is a central force, so any particle with velocity
// perpendicular to it orbits rather than falling in -- like a planet
// around a star -- and keeps orbiting indefinitely with nothing to bleed
// that momentum off, no matter how slowly (or not at all) the arc itself
// rotates. This damps particle velocity while calm mode is active so the
// attractors' pull can actually win and the water settles onto the arc's
// curve instead of endlessly swinging around it.
static constexpr float kCalmModeDamping = 2.0f;

// Fluid::kFlipRatio (0.98, pure-ish FLIP) is the right feel for normal
// tilt-driven splashing, but calm mode wants a thicker, more viscous look --
// lower blends in more PIC, which re-derives velocity from the smoothed
// grid every step instead of mostly carrying the particle's own velocity
// forward.
static constexpr float kCalmModeFlipRatio = 0.5f;

// How fast calm mode's center drifts towards a slow tilt *change* while
// active, and how far (in grid-cell units) it's allowed to wander from the
// grid's actual center.
static constexpr float kCalmModeDriftGain = 3.0f;
static constexpr float kCalmModeCenterMargin = 2.0f;

// Time constant for the tilt baseline that tracks the pendant's constant/
// resting tilt (see tiltBaselineX_/Y_ in FluidFace.h) -- long enough to
// stay well clear of kCalmDurationS so it settles before calm mode ever
// engages.
static constexpr float kTiltBaselineTauS = 2.0f;

FluidFace::FluidFace(Color color) : color_(color) {}

void FluidFace::feedImu(const ImuSample &sample) {
    // accel is in milli-g; the X/Y readings are the projection of gravity
    // onto the board's plane, i.e. exactly the "downhill" direction we want
    // the water to accelerate towards when the board is tilted. Signs match
    // the original single-dot demo's tilt convention.
    float ax = sample.accel[0] / 1000.0f;
    float ay = sample.accel[1] / 1000.0f;

    float dax = ax - lastAccelX_;
    float day = ay - lastAccelY_;
    motionIntensity_ = sqrtf(dax * dax + day * day);
    lastAccelX_ = ax;
    lastAccelY_ = ay;

    gravityX_ = -ax * kGravityScale;
    gravityY_ = ay * kGravityScale;

    // Same direction as gravityX_/gravityY_ above, kept separate (and
    // unscaled) since calm mode uses it only for a gentle center drift,
    // not as a force driving the water directly.
    tiltX_ = -ax;
    tiltY_ = ay;
}

FaceFrame FluidFace::getFrame(uint32_t dtUs) {
    // dtUs is expected to already be clamped to a sane range by the caller
    // (see main.cpp) -- the same clamped value the double-tap detector's
    // baseline filter sees, so a stalled frame doesn't destabilize the
    // solver or make FaceSwitcher's tap detection behave differently.
    float dt = dtUs / 1000000.0f;

    // Real motion cancels calm mode immediately; otherwise accumulate calm
    // time until it's held long enough to engage.
    if (motionIntensity_ > kMotionThreshold) {
        calmTimer_ = 0.0f;
        calmModeActive_ = false;
        calmModeBlend_ = 0.0f;
    } else {
        calmTimer_ += dt;
        if (calmTimer_ >= kCalmDurationS) {
            calmModeActive_ = true;
        }
    }

    // Slowly-adapting baseline for the pendant's constant/resting tilt,
    // updated regardless of mode so it's already settled by the time calm
    // mode can first engage (kTiltBaselineTauS is well under
    // kCalmDurationS). Subtracting it from the raw tilt isolates just the
    // slow-changing part.
    tiltBaselineX_ += (tiltX_ - tiltBaselineX_) * std::min(1.0f, dt / kTiltBaselineTauS);
    tiltBaselineY_ += (tiltY_ - tiltBaselineY_) * std::min(1.0f, dt / kTiltBaselineTauS);
    float tiltDeviationX = tiltX_ - tiltBaselineX_;
    float tiltDeviationY = tiltY_ - tiltBaselineY_;

    float appliedGravityX = gravityX_;
    float appliedGravityY = gravityY_;
    std::array<Attractor, kCalmModeNumAttractors> attractors{};
    std::span<const Attractor> attractorSpan;

    if (calmModeActive_) {
        calmModeBlend_ = std::min(1.0f, calmModeBlend_ + dt / kCalmModeBlendInS);

        calmModeAngle_ += kCalmModeAngularSpeed * dt;

        // Gentle drift towards a slow tilt *change* (not the constant
        // resting tilt, which would otherwise permanently drag the vortex
        // toward one side), clamped to a small region around the grid's
        // actual center so it can't wander off towards a wall.
        calmModeCenterX_ += tiltDeviationX * kCalmModeDriftGain * dt;
        calmModeCenterY_ += tiltDeviationY * kCalmModeDriftGain * dt;
        float defaultCenterX = Fluid::gridWidth() / 2.0f;
        float defaultCenterY = Fluid::gridHeight() / 2.0f;
        calmModeCenterX_ = std::clamp(calmModeCenterX_, defaultCenterX - kCalmModeCenterMargin, defaultCenterX + kCalmModeCenterMargin);
        calmModeCenterY_ = std::clamp(calmModeCenterY_, defaultCenterY - kCalmModeCenterMargin, defaultCenterY + kCalmModeCenterMargin);

        // Attractor points spread across a 270-degree arc around the
        // center, all rotating together as calmModeAngle_ advances --
        // strength tapers to 0 at both ends of the arc (via sin) so the
        // open side of the "C" fades smoothly rather than cutting off
        // abruptly.
        for (int i = 0; i < kCalmModeNumAttractors; i++) {
            float arcT = static_cast<float>(i) / static_cast<float>(kCalmModeNumAttractors - 1); // 0..1 across the arc
            float pointAngle = calmModeAngle_ + arcT * kCalmModeArcSpanRadians;
            float taper = sinf(arcT * kPi); // 0 at both ends, 1 at the middle
            attractors[i] = Attractor{
                calmModeCenterX_ + kCalmModeOrbitRadius * cosf(pointAngle),
                calmModeCenterY_ + kCalmModeOrbitRadius * sinf(pointAngle),
                kCalmModeAttractorStrength * taper * calmModeBlend_,
            };
        }
        attractorSpan = attractors;

        // The constant/resting component of tilt-gravity is fully cancelled
        // in calm mode (not just faded down) -- only its slow-changing
        // deviation still nudges the vortex above, via tiltDeviationX/Y.
        appliedGravityX = 0.0f;
        appliedGravityY = 0.0f;
    }

    fluid_.step(dt, appliedGravityX, appliedGravityY, attractorSpan,
                calmModeActive_ ? kCalmModeDamping : 0.0f,
                calmModeActive_ ? kCalmModeFlipRatio : Fluid::kFlipRatio);

    FaceFrame frame{};
    float restDensity = fluid_.restDensity();

    for (int x = 0; x < FACE_WIDTH; x++) {
        for (int y = 0; y < FACE_HEIGHT; y++) {
            float d = fluid_.cellDensity(x, y);
            float rel = restDensity > 0.0f ? d / restDensity : d;
            if (rel < kWaterOnThreshold) continue;

            // Every wet cell renders at full intensity, tinted by this
            // face's color -- no halftones for water depth/density,
            // strictly on or off. Overall display brightness is applied
            // later by the caller (see main.cpp), not here, so it's
            // consistent across every Face rather than duplicated per-Face.
            float intensity = LED_BRIGHTNESS;
            frame.pixels[x][y] = Color{
                intensity * color_.r,
                intensity * color_.g,
                intensity * color_.b,
            };
        }
    }
    return frame;
}
