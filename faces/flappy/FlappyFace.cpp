/*****************************************************************************
* | File      	:   FlappyFace.cpp
* | Author      :   poconoco, https://www.youtube.com/@nocomake
* | Assisted by :   Claude AI code generation
* | License     :   MIT
* | Function    :   Face implementation rendering a simplified Flappy-Bird
*                    style game controlled by twisting/shaking the pendant.
******************************************************************************/
#include "FlappyFace.h"

#include "pico/time.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

extern "C" {
#include "WS2812.h" // for LED_BRIGHTNESS, the hardware safety brightness cap
}

namespace {

// Game logic below works in the same nominal "USB port up" orientation as
// the rest of the codebase (see Face.h's coordinate comment); like
// MinecraftFace, the board is worn rotated 180 degrees from that as a
// pendant, so getFrame applies the same 180-degree flip when writing pixels
// (see MinecraftFace.cpp for the identical transform) to keep every Face
// consistently oriented for the wearer.

// Fixed logical column the bird stays in. Pipes spawn at nominal x = -1 and
// count up towards FACE_WIDTH (see resetGame/getFrame below), which -- after
// the 180-degree flip applied at the bottom of getFrame -- makes them enter
// from the physical right edge and scroll towards the physical left, same as
// classic Flappy Bird. The bird sits near the nominal-right edge so it ends
// up near the physical-left edge, giving the pipe a full screen's travel
// distance before it reaches the bird.
constexpr int kBirdX = FACE_WIDTH - 2;

// Tuned by eye, same spirit as the other faces' tuning constants -- adjust
// freely to change the game's feel.
constexpr float kGravity = 10.0f;        // logical rows/s^2, always pulls the bird down -- never changed by tilt

// Tilt angle didn't work well as a control input, so flying is instead driven
// by gyro angular rate: twisting/shaking the pendant past this threshold
// starts flying, and settling back below it (holding still) stops it. A
// hand holding the pendant is never perfectly still, so this needs to sit
// meaningfully above normal hand jitter -- tune by feel on-device.
constexpr float kFlyGyroThreshold = 60.0f; // deg/s
constexpr float kFlyAccel = 15.0f;       // logical rows/s^2 of upward thrust while flying (must exceed kGravity to climb)
constexpr float kMaxVelY = 6.0f;         // clamp on vertical speed, rows/s
constexpr float kPipeSpeed = 2.0f;       // logical columns/s the pipe scrolls
constexpr int kMinGapHeight = 3;         // randomized vertical opening in the pipe, rows
constexpr int kMaxGapHeight = 6;
constexpr int kPipeWidth = 2;            // pipe columns wide; wider reads as smoother 1-pixel-at-a-time scrolling than a single column
constexpr float kPipeBrightness = 0.5f;  // pipes render at half intensity relative to the bird

constexpr float kDamageBlinkDuration = 0.75f; // seconds the bird flashes red after a hit
constexpr float kBlinkPeriod = 0.15f;        // seconds per red/normal flash cycle

constexpr Color kBirdColor = {1.0f, 0.35f, 0.0f};
constexpr Color kPipeColor = {0.0f, 1.0f, 0.0f};
constexpr Color kDamageColor = {1.0f, 0.0f, 0.0f};

// Perceived brightness isn't linear in coverage fraction -- even a sliver of
// overlap reads as nearly fully lit to the eye, which is why sub-pixel
// blending looked like two solid pixels instead of one pixel with a faint
// neighbor. Squaring the coverage fraction before using it as brightness
// pushes small overlaps down disproportionately more than large ones, so the
// dominant pixel stays crisp and the neighbor only glows once it's carrying
// a real share of the position.
float shapeCoverage(float coverage) {
    return coverage * coverage;
}

} // namespace

FlappyFace::FlappyFace() {
    srand(static_cast<unsigned>(time_us_64()));
    resetGame();
}

void FlappyFace::resetGame() {
    birdY_ = FACE_HEIGHT / 2.0f;
    birdVelY_ = 0.0f;
    pipeX_ = -1.0f;
    randomizeGap();
    frontEvaluated_ = false;
}

void FlappyFace::randomizeGap() {
    int gapHeight = kMinGapHeight + rand() % (kMaxGapHeight - kMinGapHeight + 1);
    gapLow_ = rand() % (FACE_HEIGHT - gapHeight + 1);
    gapHigh_ = gapLow_ + gapHeight - 1;
}

void FlappyFace::feedImu(const ImuSample &sample) {
    // gyro is in degrees/sec; magnitude is direction-agnostic so any twist or
    // shake counts, not just rotation about one particular axis.
    gyroMagnitude_ = sqrtf(sample.gyro[0] * sample.gyro[0] +
                            sample.gyro[1] * sample.gyro[1] +
                            sample.gyro[2] * sample.gyro[2]);
}

FaceFrame FlappyFace::getFrame(uint32_t dtUs) {
    float dt = dtUs / 1000000.0f;

    if (damageBlinkTimer_ > 0.0f) {
        // Frozen in place at the moment of the hit -- nothing moves, the
        // bird just blinks red until the timer runs out, then resets.
        damageBlinkTimer_ -= dt;
        if (damageBlinkTimer_ <= 0.0f) {
            damageBlinkTimer_ = 0.0f;
            resetGame();
        }
    } else {
        // Gravity is constant and never scaled by input. Twisting/shaking
        // past kFlyGyroThreshold switches on a fixed upward thrust (flying);
        // settling back below the threshold switches it off and gravity
        // alone takes over again.
        bool flying = gyroMagnitude_ > kFlyGyroThreshold;
        float accelY = kGravity - (flying ? kFlyAccel : 0.0f);
        birdVelY_ += accelY * dt;
        birdVelY_ = std::clamp(birdVelY_, -kMaxVelY, kMaxVelY);
        birdY_ += birdVelY_ * dt;

        // Ceiling is solid but harmless -- clamp position and kill any
        // residual upward velocity so gravity can retake the bird
        // immediately, but never treat it as a hit.
        if (birdY_ < 0.0f) {
            birdY_ = 0.0f;
            if (birdVelY_ < 0.0f) birdVelY_ = 0.0f;
        }

        pipeX_ += kPipeSpeed * dt;

        int pipeBack = static_cast<int>(std::lround(pipeX_));
        int pipeFront = pipeBack + kPipeWidth - 1;
        bool insideSpan = kBirdX >= pipeBack && kBirdX <= pipeFront;

        bool hitPipe = false;
        if (insideSpan) {
            int birdRow = static_cast<int>(std::lround(birdY_));
            bool inGap = birdRow >= gapLow_ && birdRow <= gapHigh_;
            if (!frontEvaluated_) {
                // First frame the bird's column overlaps this pipe -- this is
                // the only frame that can kill. Entering the gap here is safe.
                frontEvaluated_ = true;
                if (!inGap) {
                    hitPipe = true;
                }
            } else if (!inGap) {
                // Already inside the gap -- drifting up/down into the pipe
                // from the side just blocks movement, it doesn't kill.
                if (birdRow < gapLow_) {
                    birdY_ = static_cast<float>(gapLow_);
                    if (birdVelY_ < 0.0f) birdVelY_ = 0.0f;
                } else {
                    birdY_ = static_cast<float>(gapHigh_);
                    if (birdVelY_ > 0.0f) birdVelY_ = 0.0f;
                }
            }
        }

        bool fellToFloor = birdY_ >= FACE_HEIGHT - 1.0f;
        if (fellToFloor) {
            // Clamp to the bottom row so the bird is still visible there
            // while it blinks, instead of resting past the last row.
            birdY_ = FACE_HEIGHT - 1.0f;
        }

        if (hitPipe || fellToFloor) {
            // Freeze here and blink; resetGame() happens once the blink
            // timer above runs out, not on this frame.
            damageBlinkTimer_ = kDamageBlinkDuration;
        } else if (pipeX_ > FACE_WIDTH) {
            pipeX_ = -1.0f;
            randomizeGap();
            frontEvaluated_ = false;
        }
    }

    bool blinkRed = damageBlinkTimer_ > 0.0f && std::fmod(damageBlinkTimer_, kBlinkPeriod) < kBlinkPeriod * 0.5f;

    // Bird's continuous row is split between its two nearest rows, weighted
    // by how close it is to each -- e.g. birdY_ = 3.3 lights row 3 at 70%
    // and row 4 at 30% -- so vertical motion reads as smooth sub-pixel
    // movement rather than snapping between rows.
    float birdRow0f = std::floor(birdY_);
    int birdRow0 = static_cast<int>(birdRow0f);
    float birdFrac = birdY_ - birdRow0f;

    FaceFrame frame{};
    for (int ny = 0; ny < FACE_HEIGHT; ny++) {
        for (int nx = 0; nx < FACE_WIDTH; nx++) {
            float r = 0.0f, g = 0.0f, b = 0.0f;

            // Pipe first, so it still fills its column at rows other than
            // the bird's own -- previously this was an else-if keyed on the
            // bird's column, which blanked the whole column of pipe whenever
            // the bird flew through it instead of just the row(s) it occupies.
            if (ny < gapLow_ || ny > gapHigh_) {
                // Fraction of this column's width actually covered by the
                // pipe's continuous span, so its leading/trailing edges
                // fade in/out across a column instead of popping in whole.
                float overlapStart = std::max(static_cast<float>(nx), pipeX_);
                float overlapEnd = std::min(static_cast<float>(nx + 1), pipeX_ + kPipeWidth);
                float coverage = std::clamp(overlapEnd - overlapStart, 0.0f, 1.0f);
                if (coverage > 0.0f) {
                    float shaped = shapeCoverage(coverage);
                    r = kPipeColor.r * kPipeBrightness * shaped;
                    g = kPipeColor.g * kPipeBrightness * shaped;
                    b = kPipeColor.b * kPipeBrightness * shaped;
                }
            }

            // Bird drawn on top, overriding only the row(s) it actually
            // covers in its column.
            if (nx == kBirdX) {
                float coverage = 0.0f;
                if (ny == birdRow0) {
                    coverage = 1.0f - birdFrac;
                } else if (ny == birdRow0 + 1) {
                    coverage = birdFrac;
                }
                if (coverage > 0.0f) {
                    float shaped = shapeCoverage(coverage);
                    const Color &birdColor = blinkRed ? kDamageColor : kBirdColor;
                    r = birdColor.r * shaped;
                    g = birdColor.g * shaped;
                    b = birdColor.b * shaped;
                }
            }

            int px = FACE_WIDTH - 1 - nx;
            int py = FACE_HEIGHT - 1 - ny;
            frame.pixels[px][py] = Color{
                LED_BRIGHTNESS * r,
                LED_BRIGHTNESS * g,
                LED_BRIGHTNESS * b,
            };
        }
    }
    return frame;
}
