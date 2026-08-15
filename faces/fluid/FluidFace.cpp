/*****************************************************************************
* | File      	:   FluidFace.cpp
* | Author      :   Leonid, https://www.youtube.com/@nocomake
* | Assisted by :   Claude AI code generation
* | License     :   MIT
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
static constexpr float kMotionThreshold = 0.15f;

// How long tilt must stay below kMotionThreshold before calm mode engages.
static constexpr float kCalmDurationS = 2.0f;

// How long calmModeBlend_ takes to ramp 0->1 when calm mode engages, and
// separately 1->0 when it disengages (see getFrame) -- adjustable
// independently since easing into the vortex and easing back to normal
// splashing don't need to feel the same.
static constexpr float kCalmModeBlendInS = 2.0f;
static constexpr float kCalmModeBlendOutS = 1.5f;

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

// Calm mode's arc always orbits the grid's actual center -- tilt has no
// influence on it at all once engaged.
static constexpr float kCalmModeCenterX = Fluid::gridWidth() / 2.0f;
static constexpr float kCalmModeCenterY = Fluid::gridHeight() / 2.0f;

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
}

FaceFrame FluidFace::getFrame(uint32_t dtUs) {
    // dtUs is expected to already be clamped to a sane range by the caller
    // (see main.cpp) -- the same clamped value the double-tap detector's
    // baseline filter sees, so a stalled frame doesn't destabilize the
    // solver or make FaceSwitcher's tap detection behave differently.
    float dt = dtUs / 1000000.0f;

    // Real motion cancels calm mode immediately; otherwise accumulate calm
    // time until it's held long enough to engage. This only decides the
    // *target* calmModeBlend_ ramps towards below -- it doesn't gate the
    // physics directly, so the transition itself can be gradual either way.
    if (motionIntensity_ > kMotionThreshold) {
        calmTimer_ = 0.0f;
        calmModeActive_ = false;
    } else {
        calmTimer_ += dt;
        if (calmTimer_ >= kCalmDurationS) {
            calmModeActive_ = true;
        }
    }

    // Ramp towards the target at whichever rate applies for that direction,
    // instead of snapping. Everything below scales continuously off this
    // one value, so easing in and easing out are each a single smooth
    // cross-fade between normal tilt-gravity and calm mode's vortex.
    if (calmModeActive_) {
        calmModeBlend_ = std::min(1.0f, calmModeBlend_ + dt / kCalmModeBlendInS);
    } else {
        calmModeBlend_ = std::max(0.0f, calmModeBlend_ - dt / kCalmModeBlendOutS);
    }

    float appliedGravityX = gravityX_ * (1.0f - calmModeBlend_);
    float appliedGravityY = gravityY_ * (1.0f - calmModeBlend_);
    std::array<Attractor, kCalmModeNumAttractors> attractors{};
    std::span<const Attractor> attractorSpan;

    if (calmModeBlend_ > 0.0f) {
        calmModeAngle_ += kCalmModeAngularSpeed * dt;

        // Attractor points spread across a 270-degree arc around the
        // grid's fixed center, all rotating together as calmModeAngle_
        // advances -- strength tapers to 0 at both ends of the arc (via
        // sin) so the open side of the "C" fades smoothly rather than
        // cutting off abruptly, and to 0 overall as calmModeBlend_ fades.
        for (int i = 0; i < kCalmModeNumAttractors; i++) {
            float arcT = static_cast<float>(i) / static_cast<float>(kCalmModeNumAttractors - 1); // 0..1 across the arc
            float pointAngle = calmModeAngle_ + arcT * kCalmModeArcSpanRadians;
            float taper = sinf(arcT * kPi); // 0 at both ends, 1 at the middle
            attractors[i] = Attractor{
                kCalmModeCenterX + kCalmModeOrbitRadius * cosf(pointAngle),
                kCalmModeCenterY + kCalmModeOrbitRadius * sinf(pointAngle),
                kCalmModeAttractorStrength * taper * calmModeBlend_,
            };
        }
        attractorSpan = attractors;
    }

    // Damping and viscosity cross-fade the same way as gravity/attractors
    // above, so the whole transition -- not just the force field -- eases
    // smoothly between normal and calm mode instead of switching partway
    // through.
    float damping = kCalmModeDamping * calmModeBlend_;
    float flipRatio = Fluid::kFlipRatio + (kCalmModeFlipRatio - Fluid::kFlipRatio) * calmModeBlend_;
    fluid_.step(dt, appliedGravityX, appliedGravityY, attractorSpan, damping, flipRatio);

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
