/*****************************************************************************
* | File      	:   LifeFace.cpp
* | Author      :   Leonid, https://www.youtube.com/@nocomake
* | Assisted by :   Claude AI code generation
* | License     :   MIT
* | Function    :   Face implementation running Conway's Game of Life on a
*                    wrapped 8x8 world, reseeded whenever it stagnates.
******************************************************************************/
#include "LifeFace.h"

#include <cstring>

extern "C" {
#include "WS2812.h" // for LED_BRIGHTNESS, the hardware safety brightness cap
}

namespace {

static_assert(FACE_WIDTH == 8 && FACE_HEIGHT == 8,
              "LifeFace packs the whole world into one uint64_t, a bit per cell, "
              "and wraps coordinates with a 3-bit mask; both assume an 8x8 grid.");

// How long one generation stays on screen, and the knob most worth playing
// with: much under 0.2s and the eye cannot follow what the rules are doing,
// much over 0.8s and a world this small looks frozen.
constexpr float kStepS = 0.100f;

// Fraction of cells seeded alive. Around a third is the classic soup density:
// denser, and the opening generations mostly annihilate each other into a
// still life; sparser, and too little survives first contact to interact.
constexpr float kLifeRatio = 0.35f;

// The ratio as a threshold on a 24-bit random value. Scaling to the full 32
// bits instead would overflow the conversion at a ratio of 1.0, and the top
// bits of an xorshift word are its best ones anyway.
constexpr uint32_t kLifeThreshold = static_cast<uint32_t>(kLifeRatio * 16777216.0f);

// Living cells are drawn this colour; dead ones are simply left off.
constexpr Color kLifeColor = {0.0f, 1.00f, 1.00f};

// Stagnation handling: how long the period-1-or-2 test has to hold before the
// world is written off, how long the fade to black then takes, how long the
// display is held dark before new life appears, and the backstop that
// reseeds regardless -- for gliders and longer oscillators, which cycle too
// slowly for the comparison test to ever catch.
constexpr float kStableHoldS = 3.0f;
constexpr float kFadeOutS = 1.5f;
constexpr float kDarkS = 1.0f;
constexpr float kWorldTimeoutS = 300.0f; // 5 minutes

// What makes the world a torus. The grid is 8 wide and 8 tall, so masking a
// coordinate to its low three bits maps -1 to 7 and 8 to 0 in one operation.
// The two axes are masked independently, which is what gives the corner cells
// their diagonal neighbours across both wraps without any special casing.
constexpr int kWrapMask = 7;

uint64_t cellBit(int x, int y) {
    return 1ull << (y * FACE_WIDTH + x);
}

bool alive(uint64_t world, int x, int y) {
    return (world & cellBit(x, y)) != 0ull;
}

int neighbours(uint64_t world, int x, int y) {
    int count = 0;
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            if (alive(world, (x + dx) & kWrapMask, (y + dy) & kWrapMask)) count++;
        }
    }
    return count;
}

} // namespace

void LifeFace::feedImu(const ImuSample &sample) {
    // Entropy harvesting. The top of an accelerometer reading is the board's
    // orientation -- steady, and near enough identical every time the pendant
    // hangs the same way -- but the bottom of the mantissa jitters with sensor
    // noise on every single sample. Stirring all three axes into a running
    // hash, and letting that accumulate between reseeds, means each new world
    // starts from a genuinely unpredictable soup rather than replaying the
    // same sequence from every boot.
    for (int axis = 0; axis < 3; axis++) {
        // memcpy rather than a cast or a union: it is the one way to reinterpret
        // the bits that is not undefined behaviour, and every compiler folds it
        // away to nothing. (std::bit_cast would read better, but this
        // toolchain's libstdc++ does not carry it.)
        uint32_t bits;
        std::memcpy(&bits, &sample.accel[axis], sizeof(bits));
        entropy_ = entropy_ * 1664525u + 1013904223u;
        entropy_ ^= bits;
    }
}

uint32_t LifeFace::nextRandom() {
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 17;
    rng_ ^= rng_ << 5;
    return rng_;
}

void LifeFace::seed() {
    // Fold in whatever the accelerometer has stirred up since the last time.
    rng_ ^= entropy_;
    if (rng_ == 0) rng_ = 0x9E3779B9u; // xorshift is stuck forever at zero

    grid_ = 0;
    for (int y = 0; y < FACE_HEIGHT; y++) {
        for (int x = 0; x < FACE_WIDTH; x++) {
            if ((nextRandom() >> 8) < kLifeThreshold) grid_ |= cellBit(x, y);
        }
    }

    // A fresh world has no history worth comparing against, so the cycle test
    // is disarmed by the generation count until two real steps have run.
    prev1_ = 0;
    prev2_ = 0;
    generation_ = 0;
    stepTimer_ = 0.0f;
    stableTimer_ = 0.0f;
    worldAge_ = 0.0f;
}

void LifeFace::step() {
    uint64_t next = 0;
    for (int y = 0; y < FACE_HEIGHT; y++) {
        for (int x = 0; x < FACE_WIDTH; x++) {
            // B3/S23: a dead cell with exactly three live neighbours is born,
            // a live one with two or three survives, everything else dies.
            int live = neighbours(grid_, x, y);
            if (live == 3 || (live == 2 && alive(grid_, x, y))) next |= cellBit(x, y);
        }
    }

    prev2_ = prev1_;
    prev1_ = grid_;
    grid_ = next;
    generation_++;
}

FaceFrame LifeFace::getFrame(uint32_t dtUs) {
    float dt = dtUs / 1000000.0f;

    if (!seeded_) {
        // Deferred to the first frame rather than done in the constructor:
        // FaceSwitcher hands a newly built Face its first IMU sample before it
        // ever calls getFrame, so by now entropy_ holds real sensor noise.
        // Seeding in the constructor would use the fixed initial value, and
        // every power-on would start from the identical world.
        seed();
        seeded_ = true;
    }

    // The simulation keeps running through the fade, so the world carries on
    // dying out of sight rather than freezing on screen. It only pauses once
    // the display is fully dark and a reseed is a moment away.
    if (phase_ != Phase::Dark) {
        stepTimer_ += dt;
        while (stepTimer_ >= kStepS) {
            stepTimer_ -= kStepS;
            step();
        }
    }

    worldAge_ += dt;

    if (phase_ == Phase::Running && grid_ == 0) {
        // Extinction, handled ahead of the general test and quite differently.
        // It is technically just another period-1 cycle, but it is the one
        // case where waiting proves nothing: an empty world can never do
        // anything again, and the screen is already black. Holding it for
        // kStableHoldS to confirm, then fading black to black over
        // kFadeOutS, would be four and a half seconds of blank display. So
        // skip both and go straight to the dark wait -- life dies out, and a
        // second later a new world appears.
        phase_ = Phase::Dark;
        phaseTimer_ = 0.0f;
    } else if (phase_ == Phase::Running) {
        // The current generation matching the one from two steps back means
        // the whole world is repeating with period 1 or 2 and nothing else is
        // moving. It has to hold for kStableHoldS rather than firing on the
        // first match, so a pattern that merely passes back through an earlier
        // state on its way somewhere else is not mistaken for a dead end.
        bool cycling = generation_ >= 2 && grid_ == prev2_;
        stableTimer_ = cycling ? stableTimer_ + dt : 0.0f;

        if (stableTimer_ >= kStableHoldS || worldAge_ >= kWorldTimeoutS) {
            phase_ = Phase::FadingOut;
            phaseTimer_ = 0.0f;
        }
    } else {
        phaseTimer_ += dt;
        if (phase_ == Phase::FadingOut && phaseTimer_ >= kFadeOutS) {
            phase_ = Phase::Dark;
            phaseTimer_ = 0.0f;
        } else if (phase_ == Phase::Dark && phaseTimer_ >= kDarkS) {
            seed();
            phase_ = Phase::Running;
            phaseTimer_ = 0.0f;
        }
    }

    float fade = 1.0f;
    if (phase_ == Phase::FadingOut) {
        fade = 1.0f - phaseTimer_ / kFadeOutS;
    } else if (phase_ == Phase::Dark) {
        fade = 0.0f;
    }

    // Rendered unflipped, unlike MinecraftFace and the games: a random soup on
    // a torus has no up, so there is no authored orientation to correct for.
    FaceFrame frame{};
    for (int y = 0; y < FACE_HEIGHT; y++) {
        for (int x = 0; x < FACE_WIDTH; x++) {
            if (!alive(grid_, x, y)) continue;
            frame.pixels[x][y] = Color{
                LED_BRIGHTNESS * kLifeColor.r * fade,
                LED_BRIGHTNESS * kLifeColor.g * fade,
                LED_BRIGHTNESS * kLifeColor.b * fade,
            };
        }
    }
    return frame;
}
