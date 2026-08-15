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
// Which way it faces is decided the same way an animal standing across a
// slope decides: head to the uphill side, tail downhill. Since the sprite is
// authored head-left, that is a mirror end for end, played through a hunched
// symmetric frame so the cat visibly turns around instead of snapping inside
// out (see kFlipS and kTurnTop in the .cpp).
//
// Gravity always applies, however weak. There is deliberately no "the board
// is lying flat, hold still" case: whatever it saved in idle jitter, it also
// meant declaring the cat to be standing wherever it happened to be, so any
// transient that cancelled in-plane gravity -- a swing, being lifted, the
// board tipping onto its back -- could leave it frozen in mid-air against an
// edge, tail flicking, refusing to fall. Being jostled around by
// accelerometer noise on a table is the better failure mode.
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

    // The four sprite rows to draw this frame (see the kTop*/kFeet* strings
    // in the .cpp). Ordinary poses only vary the ears/tail row and the feet
    // row, but the mid-turn frame replaces all four, so a pose carries the
    // whole sprite rather than just the parts that usually move.
    struct Pose {
        const char *top;   // ears and tail
        const char *upper; // head and back
        const char *body;  // body
        const char *feet;  // legs
    };

    // The idle fidget currently playing, if any. Only reached once the cat
    // has been resting still for a moment (see kSettleS in the .cpp).
    enum class Idle : uint8_t {
        None,
        TailFlick,
        EarTwitch,
        Shuffle,
        Walk,
    };

    void updateOrientation(float dt);
    void updateFacing(float dt);
    void updatePhysics(float dt);
    void updateAnimation(float dt);
    void updateBlink(float dt);
    void updateWalk(float dt);
    float nextBlinkGap();
    void beginFlip();

    // Unit vector the cat's feet point along, i.e. down as the cat sees it.
    Vec2 feetDir() const;

    // Unit vector the cat's head points along, i.e. the way it walks.
    Vec2 forwardDir() const;

    // Whole pixels of clear floor between the cat's leading edge and the wall
    // it is walking toward. Whole pixels because walks are, and rounded
    // rather than truncated -- see the .cpp.
    int stepsAhead() const;
    void beginReorient(int direction);
    void adoptDownhillFacing();
    void resolveWalls();
    Pose currentPose() const;

    // How much of gravity pulls along the body's own head-to-tail axis, as
    // the sprite is currently drawn. Positive means the tail is already the
    // downhill end, which is the way the cat wants to stand.
    float downhillAlignment() const;

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
    // points into -- and only then. However weak gravity gets, standing is
    // always a claim about touching a wall, never about having nowhere left
    // to fall, or a cat stalled in mid-air would count as standing.
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

    // Which way the cat faces. The sprite is authored head-left; when this is
    // set it is mirrored end for end at draw time, so the head always ends up
    // on the uphill side of whatever slope the cat is standing on. facingT_
    // runs 0 -> 1 over kFlipS while it turns around, passing through the
    // symmetric hunched frame, with the mirror itself applied at the midpoint.
    bool facingFlipped_ = false;
    bool flipping_ = false;
    bool flipApplied_ = false;
    float flipT_ = 0.0f;
    float facingTimer_ = 0.0f;

    // Whole pixels still to cover in the walk currently under way. Counted in
    // distance rather than time so an outing is always a round number of
    // pixels, however long the walk itself ends up taking. walkGapS_ counts
    // down to the next outing, on its own clock so that how often the cat
    // wanders is independent of how often it fidgets.
    float walkRemaining_ = 0.0f;
    float walkGapS_ = 0.0f;

    // Blinking, on its own clock independent of the idle fidgets below, so
    // the two can overlap. blinkGapS_ counts down to the next blink;
    // blinkT_ times the blink itself once the eyes are shut.
    bool blinking_ = false;
    float blinkT_ = 0.0f;
    float blinkGapS_ = 0.0f;

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
