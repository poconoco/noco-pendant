/*****************************************************************************
* | File      	:   FluidFace.h
* | Author      :   poconoco, https://www.youtube.com/@nocomake
* | Assisted by :   Claude AI code generation
* | License     :   MIT
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
// tilted), it drifts into "calm mode": tilt-gravity is switched off entirely
// and replaced by a curved 270-degree arc of attractor points (see Fluid's
// Attractor) that orbits the grid's fixed center, dragging the water around
// into a slow vortex -- tilt has no influence at all while calm mode is
// active, only the orbiting arc does. It still fades back to normal
// tilt-gravity immediately the moment a real tilt is detected again.
class FluidFace : public Face {
public:
    // color: channel weights in [0, 1] applied on top of the density-driven
    // on/off intensity, e.g. {1,0,0} for red, {1,0.1,0} for orange. Each
    // entry in main.cpp's kFaceFactories names its own Color, so there's one
    // instance per color -- but only while that color is the Face on screen.
    explicit FluidFace(Color color);

    void feedImu(const ImuSample &sample) override;
    FaceFrame getFrame(uint32_t dtUs) override;

private:
    Color color_;
    Fluid fluid_;
    float gravityX_ = 0.0f;
    float gravityY_ = 0.0f;

    // Updated each feedImu call: how much tilt changed since the previous
    // sample, which is all that drives the calm/motion detection below --
    // calm mode ignores tilt entirely once engaged (see getFrame).
    float lastAccelX_ = 0.0f;
    float lastAccelY_ = 0.0f;
    float motionIntensity_ = 0.0f;

    // How long tilt has stayed below the motion threshold; once it crosses
    // kCalmDurationS, calm mode engages (see getFrame in the .cpp).
    float calmTimer_ = 0.0f;
    bool calmModeActive_ = false;

    // Ramps 0->1 over a couple of seconds as calm mode engages (so the
    // transition fades rather than snaps), and drops back to 0 immediately
    // the moment real motion resumes.
    float calmModeBlend_ = 0.0f;

    // The orbiting droplet cluster's current angle, in radians; it always
    // orbits the grid's fixed center (see kCalmModeCenterX/Y in the .cpp).
    float calmModeAngle_ = 0.0f;
};

#endif // FLUIDFACE_H
