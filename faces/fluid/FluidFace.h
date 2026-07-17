/*****************************************************************************
* | File      	:   FluidFace.h
* | Function    :   Face implementation that renders the FLIP/PIC water
*                    simulation (Fluid.h/Fluid.cpp) in a single solid color.
******************************************************************************/
#ifndef FLUIDFACE_H
#define FLUIDFACE_H

#include "Face.h"
#include "Fluid.h"

// Renders the water simulation tinted a single fixed color, reacting to
// tilt via gravity (X/Y accelerometer) exactly as before this was split out
// of main.c: same gravity scale and on/off density threshold. Only the
// color varies between instances. Frame pixels are handed back as raw,
// un-dithered channel intensities (see Color in Face.h) -- overall
// display brightness is a generic concern applied by the caller (main.cpp)
// on top of any Face's output, not something each Face implements itself.
//
// While left mostly still (no prominent tilt change for a few seconds --
// meant for when the board hangs as a pendant instead of being held and
// tilted), it drifts into "calm mode": normal tilt-gravity fades out and is
// replaced by a curved 270-degree arc of attractor points (see Fluid's
// Attractor) that orbits the grid's center, dragging the water around into
// a slow vortex. It still fades back to normal tilt-gravity immediately the
// moment a real tilt is detected again, and slow tilt *changes* during calm
// mode gently drift the vortex's center towards them -- the constant/
// resting component of tilt (whatever angle the pendant happens to hang at)
// is filtered out via a slowly-adapting baseline, the same idea as
// DoubleTapDetector's baseline filter, so it doesn't just permanently drag
// the vortex toward one side.
class FluidFace : public Face {
public:
    // color: channel weights in [0, 1] applied on top of the density-driven
    // on/off intensity, e.g. {1,0,0} for red, {1,0.1,0} for orange. Not
    // explicit so kFluidFaces in main.cpp can list-initialize each entry
    // directly from a braced Color literal.
    FluidFace(Color color);

    void feedImu(const ImuSample &sample) override;
    FaceFrame getFrame(uint32_t dtUs) override;

private:
    Color color_;
    Fluid fluid_;
    float gravityX_ = 0.0f;
    float gravityY_ = 0.0f;

    // Updated each feedImu call: how much tilt changed since the previous
    // sample (drives the calm/motion detection below), and the latest tilt
    // direction.
    float lastAccelX_ = 0.0f;
    float lastAccelY_ = 0.0f;
    float motionIntensity_ = 0.0f;
    float tiltX_ = 0.0f;
    float tiltY_ = 0.0f;

    // Slowly-adapting baseline for tiltX_/tiltY_ (updated every frame in
    // getFrame), tracking whatever constant/resting tilt the pendant
    // currently hangs at. tiltX_/tiltY_ minus this baseline isolates just
    // the slow-changing part of tilt, which is what drifts calm mode's
    // vortex center -- using the raw tilt directly would let a pendant's
    // resting angle alone drag the vortex permanently toward one side.
    float tiltBaselineX_ = 0.0f;
    float tiltBaselineY_ = 0.0f;

    // How long tilt has stayed below the motion threshold; once it crosses
    // kCalmDurationS, calm mode engages (see getFrame in the .cpp).
    float calmTimer_ = 0.0f;
    bool calmModeActive_ = false;

    // Ramps 0->1 over a couple of seconds as calm mode engages (so the
    // transition fades rather than snaps), and drops back to 0 immediately
    // the moment real motion resumes.
    float calmModeBlend_ = 0.0f;

    // The orbiting droplet cluster's current angle and center (in Fluid's
    // grid-cell coordinate units); the center drifts slowly towards
    // whatever subtle tilt is present while in calm mode.
    float calmModeAngle_ = 0.0f;
    float calmModeCenterX_ = Fluid::gridWidth() / 2.0f;
    float calmModeCenterY_ = Fluid::gridHeight() / 2.0f;
};

#endif // FLUIDFACE_H
