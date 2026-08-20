/*****************************************************************************
* | File      	:   LifeFace.h
* | Author      :   poconoco, https://www.youtube.com/@nocomake
* | Assisted by :   Claude AI code generation
* | License     :   MIT
* | Function    :   Face implementation running Conway's Game of Life on a
*                    wrapped 8x8 world, reseeded whenever it stagnates.
******************************************************************************/
#ifndef LIFEFACE_H
#define LIFEFACE_H

#include "Face.h"

#include <cstdint>

// Conway's Game of Life played out on the matrix itself, at B3/S23.
//
// The world is the 8x8 grid with both axes wrapped, making it a torus rather
// than a bounded board: the left column counts the right column as its
// left-hand neighbours, the top row counts the bottom row, and the corners
// follow from both wraps applying at once. On a board this small that matters
// more than it sounds -- with hard edges a glider walks into a wall and dies
// within a few generations, whereas on a torus it runs forever.
//
// The whole world fits in one uint64_t, a bit per cell. That is partly for
// compactness, but mostly because it makes comparing two generations a single
// integer ==, which is what the stagnation test below is built on.
//
// A world this small settles almost immediately -- usually inside a few dozen
// generations -- so the real problem is not running the simulation but
// noticing when it has stopped being worth watching. Two independent
// detectors handle that:
//
//   * An empty world is caught on sight. It cannot recover, and the display
//     is already black, so it skips both the confirmation hold and the fade
//     and simply waits out the dark second before new life appears.
//   * The previous two generations are kept, and the current one matching the
//     one from two steps back means the world is repeating with period 1 or
//     2 and nothing else is moving: still lifes and blinkers.
//     Sustained for kStableHoldS, that triggers a reseed.
//   * Anything cycling more slowly slips past that test -- a glider crossing
//     the torus, or a period-3-or-longer oscillator. Those are caught by a
//     plain wall-clock backstop instead (kWorldTimeoutS).
//
// Either way the world fades down while still being simulated, holds dark for
// a moment, and is reseeded from freshly harvested accelerometer noise, so no
// two worlds start from the same soup.
class LifeFace : public Face {
public:
    LifeFace() = default;

    void feedImu(const ImuSample &sample) override;
    FaceFrame getFrame(uint32_t dtUs) override;

private:
    enum class Phase : uint8_t {
        Running,   // full brightness, simulating
        FadingOut, // dimming towards black, still simulating
        Dark,      // held black, simulation paused, reseed imminent
    };

    void seed();
    void step();
    uint32_t nextRandom();

    // The world, one bit per cell (bit y * FACE_WIDTH + x), and the two
    // generations before it.
    uint64_t grid_ = 0;
    uint64_t prev1_ = 0;
    uint64_t prev2_ = 0;

    // Generations since the last seed. The cycle test compares against a
    // generation two steps back, so it means nothing until two have actually
    // been run and is suppressed until then.
    uint32_t generation_ = 0;

    float stepTimer_ = 0.0f;   // counts up to one generation interval
    float stableTimer_ = 0.0f; // how long the cycle test has held
    float worldAge_ = 0.0f;    // since the last seed, for the long backstop
    float phaseTimer_ = 0.0f;  // time spent in the current fade/dark phase

    Phase phase_ = Phase::Running;
    bool seeded_ = false;

    // Stirred by every IMU sample and folded into the generator at each
    // reseed. See feedImu for why the accelerometer is a usable entropy
    // source and which part of the reading actually carries the noise.
    uint32_t entropy_ = 0x2545F491u;
    uint32_t rng_ = 0x9E3779B9u;
};

#endif // LIFEFACE_H
