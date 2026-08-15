/*****************************************************************************
* | File      	:   CatFace.h
* | Function    :   Face implementation rendering a small pixel-art cat that
*                    falls under tilt-driven gravity and steps around to keep
*                    its feet pointing downhill.
******************************************************************************/
#ifndef CATFACE_H
#define CATFACE_H

#include "Face.h"

#include <cstdint>

// A 6x4 pixel-art cat (side view: ears and head at one end, tail up at the
// other) that lives in the 8x8 playfield under tilt-driven gravity -- same
// accelerometer axes and sign convention as HeartFace and FluidFace, so it
// falls the same way the water does.
//
// Unlike HeartFace, this body never rotates by a free angle: the sprite is
// pixel art, so it is only ever drawn at 0/90/180/270 degrees and always
// lands exactly on the pixel grid, with no antialiasing and no partial
// intensities -- a pixel is either the cat's color or off. Position is
// simulated as float and only quantized to int at draw time.
//
// The consequence of quantized rotation is that a tilted board leaves the cat
// visibly leaning: up to 45 degrees of tilt is simply absorbed, since no
// closer 90-degree step exists. Past 45 degrees the cat stops absorbing it and
// instead steps around -- splaying its feet, pushing off, and snapping one
// quadrant over -- so it ends up upright again (still leaning, but with its
// legs pointing at the nearest downhill wall). See kSnapMarginRad and
// kReorientS in the .cpp for the hysteresis and timing that keep that from
// chattering back and forth near the threshold.
//
// Left alone on a stable tilt it settles and plays small idle animations --
// tail flicks, ear twitches, a shuffle of the feet -- so the display is never
// completely static.
class CatFace : public Face {
public:
    CatFace();

    void feedImu(const ImuSample &sample) override;
    FaceFrame getFrame(uint32_t dtUs) override;

private:
    struct Vec2 {
        float x, y;
    };

    // Which sprite rows to draw this frame. The two middle rows are always
    // the solid body; only the ears/tail row and the feet row ever change,
    // so a pose is just that pair (see the kTop*/kFeet* strings in the .cpp).
    struct Pose {
        const char *top;
        const char *feet;
    };

    // The idle fidget currently playing, if any. Only reached once the cat
    // has been resting still for a moment (see kSettleS in the .cpp).
    enum class Idle : uint8_t {
        None,
        TailFlick,
        EarTwitch,
        Shuffle,
    };

    void updateOrientation(float dt);
    void updatePhysics(float dt);
    void updateAnimation(float dt);
    void beginReorient(int direction);
    void resolveWalls();
    Pose currentPose() const;

    // Footprint of the sprite in world units at the current rotation: 6x4
    // upright or on its head, 4x6 on either side.
    float footprintW() const;
    float footprintH() const;

    // Cheap xorshift32, so idle fidget timings don't fall into an obvious
    // repeating pattern. Not worth pulling <random> onto the board for.
    uint32_t nextRandom();
    float randomRange(float lo, float hi);

    // World state. pos_ is the center of the sprite's footprint, in pixels.
    Vec2 pos_{0.0f, 0.0f};
    Vec2 vel_{0.0f, 0.0f};
    Vec2 gravity_{0.0f, 0.0f};

    // Quarter turns clockwise from "feet at the bottom of the screen", so
    // rotation_ * 90 degrees is the only rotation ever rendered.
    int rotation_ = 0;

    // True while the sprite's footprint is touching whichever wall gravity
    // points into (or while gravity is too weak to have a direction at all,
    // e.g. the board lying flat on a table).
    bool onGround_ = false;

    // How long gravity has been more than 45 degrees off the cat's feet.
    // A step around is only committed once this passes kSnapDwellS, which is
    // what stops a tilt hovering right at the threshold from flip-flopping.
    float snapTimer_ = 0.0f;
    int snapDirection_ = 1;

    // Reorientation animation: reorientT_ runs 0 -> 1 over kReorientS, and
    // the actual rotation_ change lands at the halfway point.
    bool reorienting_ = false;
    bool reorientApplied_ = false;
    float reorientT_ = 0.0f;
    int reorientDirection_ = 1;

    // Idle fidgets: restTimer_ has to reach kSettleS before any of this
    // starts, idleGapS_ counts down to the next fidget, and idleT_ is the
    // time into the fidget currently playing.
    float restTimer_ = 0.0f;
    float idleGapS_ = 0.0f;
    float idleT_ = 0.0f;
    Idle idleEvent_ = Idle::None;

    // Last drawn integer origin (top-left of the footprint). Kept across
    // frames so the quantization can be hysteretic: a cat hovering exactly on
    // a half-pixel boundary would otherwise flicker between two positions.
    int originX_ = 0;
    int originY_ = 0;

    uint32_t randomState_ = 0x9E3779B9u;
};

#endif // CATFACE_H
