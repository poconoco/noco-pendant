/*****************************************************************************
* | File      	:   FluidFace.cpp
* | Function    :   Face implementation that renders the FLIP/PIC water
*                    simulation (Fluid.h/Fluid.cpp) in a single solid color.
******************************************************************************/
#include "FluidFace.h"

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

FluidFace::FluidFace(float r, float g, float b) : colorR_(r), colorG_(g), colorB_(b) {}

void FluidFace::feedImu(const ImuSample &sample) {
    // accel is in milli-g; the X/Y readings are the projection of gravity
    // onto the board's plane, i.e. exactly the "downhill" direction we want
    // the water to accelerate towards when the board is tilted. Signs match
    // the original single-dot demo's tilt convention.
    float ax = sample.accel[0] / 1000.0f;
    float ay = sample.accel[1] / 1000.0f;
    gravityX_ = -ax * kGravityScale;
    gravityY_ = ay * kGravityScale;
}

FaceFrame FluidFace::getFrame(uint32_t dtUs) {
    // dtUs is expected to already be clamped to a sane range by the caller
    // (see main.cpp) -- the same clamped value the double-tap detector's
    // baseline filter sees, so a stalled frame doesn't destabilize the
    // solver or make FaceSwitcher's tap detection behave differently.
    float dt = dtUs / 1000000.0f;
    fluid_.step(dt, gravityX_, gravityY_);

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
            frame.pixels[x][y] = FacePixel{
                intensity * colorR_,
                intensity * colorG_,
                intensity * colorB_,
            };
        }
    }
    return frame;
}
