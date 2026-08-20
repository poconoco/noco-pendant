/*****************************************************************************
* | File      	:   FlappyFace.h
* | Author      :   poconoco, https://www.youtube.com/@nocomake
* | Assisted by :   Claude AI code generation
* | License     :   MIT
* | Function    :   Face implementation rendering a simplified Flappy-Bird
*                    style game controlled by twisting/shaking the pendant.
******************************************************************************/
#ifndef FLAPPYFACE_H
#define FLAPPYFACE_H

#include "Face.h"

// A minimal Flappy-Bird-style game: an orange bird pixel in a fixed column
// accelerates downward under constant gravity and climbs while the player
// twists/shakes the pendant (detected from gyro angular rate, not tilt --
// tilt angle alone didn't work well as a control input), while a single
// green pipe scrolls in from the right with a randomized gap the bird must
// pass through. The rules are deliberately forgiving: the ceiling is solid
// but harmless (you just can't fly higher), and a pipe only kills on first
// contact with its leading edge -- once the bird has safely entered the
// gap, drifting up/down into the pipe from the side just blocks movement
// (keeping the bird in the gap) rather than killing it. A pipe hit or
// falling off the bottom freezes the game in place (clamped to the bottom
// row if it was a fall, so the bird stays visible) with the bird blinking
// red for a moment, then resets -- there's no game-over screen otherwise,
// it's meant to run forever as ambient decoration, same as the other Faces.
class FlappyFace : public Face {
public:
    FlappyFace();

    void feedImu(const ImuSample &sample) override;
    FaceFrame getFrame(uint32_t dtUs) override;

private:
    void resetGame();
    void randomizeGap();

    float gyroMagnitude_ = 0.0f;

    float birdY_ = 0.0f;
    float birdVelY_ = 0.0f;

    float pipeX_ = 0.0f;

    // Rows [gapLow_, gapHigh_] (inclusive) are the open part of the pipe;
    // both height and position are re-randomized each time a pipe spawns
    // (see randomizeGap).
    int gapLow_ = 0;
    int gapHigh_ = 0;

    // Set the first time the bird's column overlaps the current pipe's
    // span -- that frame is the only one checked for a killing hit;
    // afterwards the pipe just blocks vertical movement (see getFrame).
    bool frontEvaluated_ = false;

    // While positive, the game is frozen in place and getFrame flashes the
    // bird red instead of its normal color; reaching zero resets the game.
    float damageBlinkTimer_ = 0.0f;
};

#endif // FLAPPYFACE_H
