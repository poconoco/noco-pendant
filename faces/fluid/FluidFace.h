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
};

#endif // FLUIDFACE_H
