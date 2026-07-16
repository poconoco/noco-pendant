/*****************************************************************************
* | File      	:   FluidFace.h
* | Function    :   Face implementation that renders the FLIP/PIC water
*                    simulation (Fluid.h/Fluid.cpp) in a single solid color.
******************************************************************************/
#ifndef FLUIDFACE_H
#define FLUIDFACE_H

#include "Face.h"
#include "Fluid.h"

#include <array>

// Renders the water simulation tinted a single fixed color, reacting to
// tilt via gravity (X/Y accelerometer) exactly as before this was split out
// of main.c: same gravity scale, brightness dithering, and on/off density
// threshold. Only the color varies between instances.
class FluidFace : public Face {
public:
    // r, g, b: channel weights in [0, 1] applied on top of the
    // density-driven on/off intensity, e.g. {1,0,0} for red, {1,0.1,0} for
    // orange.
    FluidFace(float r, float g, float b);

    void feedImu(const ImuSample &sample) override;
    FaceFrame getFrame(uint32_t dtUs) override;

private:
    float colorR_, colorG_, colorB_;
    Fluid fluid_;
    float gravityX_ = 0.0f;
    float gravityY_ = 0.0f;

    // Per-pixel-per-channel dither accumulators for the overall brightness
    // setting; persists across frames (see getFrame in fluidface.cpp).
    std::array<std::array<std::array<float, 3>, FACE_HEIGHT>, FACE_WIDTH> ditherAcc_{};
};

#endif // FLUIDFACE_H
