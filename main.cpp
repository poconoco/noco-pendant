/*****************************************************************************
* | File      	:   main.cpp
* | Author      :   Waveshare Team
* | Function    :   Drives an 8x8 LED matrix "face" from the onboard IMU.
* |                 The active Face implementation owns the visualization;
* |                 this file just wires up hardware, feeds IMU samples, and
* |                 blits the returned frame to the LED strip.
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2025-07-29
* | Info        :
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of theex Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
******************************************************************************/
extern "C" {
#include "DEV_Config.h"
#include "WS2812.h"
#include "QMI8658.h"
}
#include "pico/time.h"

#include "Face.h"
#include "FaceSwitcher.h"
#include "FluidFace.h"
#include "MinecraftFace.h"
#include "FlappyFace.h"
#include "SnakeFace.h"

#include <array>
#include <cmath>

/*
 * RGB LED matrix coordinates description
 * =======================================
 * The program is set so that when the USB port is facing upward,
 * the upper left corner of the matrix is (0,0) and the lower right corner is (7,7)
 *
 * Faces
 * =====
 * Each visualization is a Face (see faces/base/Face.h); FaceSwitcher holds a
 * fixed set of them and shows one at a time, advancing to the next on a
 * double-tap of the board's face. Today that's one FluidFace (see
 * faces/fluid/FluidFace.h) per color -- a FLIP/PIC water simulation tinted a
 * single fixed color -- one MinecraftFace (see
 * faces/minecraft/MinecraftFace.h), which cycles through hardcoded block/mob
 * icons on its own timer -- one FlappyFace (see faces/flappy/FlappyFace.h),
 * a simplified Flappy-Bird-style game steered by tilt -- and one SnakeFace
 * (see faces/snake/SnakeFace.h), a classic Snake game also steered by tilt.
 */

// Frame pacing: keep the physics/I2C loop well below the sensor+solver's
// natural speed so tilts feel smooth without hammering the I2C bus. Each
// Face here is cheap (small grid, ~70 particles), so this is the main lever
// for how fluid the animation looks; lower it further if there's headroom.
#define LOOP_DELAY_MS 0.025

// The time delta handed to Faces each frame is clamped to this range so a
// slow frame (e.g. while debugging) can't destabilize a solver or make
// FaceSwitcher's double-tap detection see an unreasonable jump, and so the
// first frame's delta isn't zero.
#define MIN_DT_S 0.0005f
#define MAX_DT_S 0.05f

// Overall display brightness, independent of whichever Face is active:
// 0.0 = off, 1.0 = the Face's own full-intensity output (see Color's
// scale in Face.h). Applied via per-pixel-per-channel dithering rather than
// directly scaling values down or blanking whole frames -- scaling shrinks
// the already-small 0..LED_BRIGHTNESS integer range further, leaving too
// few distinct levels for a smooth gradient or correct color-weight ratios;
// blanking whole frames dims correctly but flickers, since the entire
// display swings between lit and dark in lockstep. Dithering each channel
// independently spreads that same on/off averaging across many independent
// accumulators instead of one global one, so it settles on the right
// average brightness and hue without the whole matrix visibly strobing.
static constexpr float kBrightness = 0.3f;

// Per-pixel-per-channel dither accumulators for kBrightness; persists
// across frames regardless of which Face is currently active.
static std::array<std::array<std::array<float, 3>, FACE_HEIGHT>, FACE_WIDTH> ditherAcc;

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// One FluidFace per color, cycled by double-tap; red is first so it's the
// startup default. The extra pair of braces per entry is because FluidFace's
// constructor now takes a single Color -- the inner braces build that Color,
// the outer ones are the FluidFace itself.
static FluidFace kFluidFaces[] = {
    {{1.0f, 0.1f, 0.0f}},  // orange
    {{0.6f, 0.2f, 0.0f}},  // yellow
    {{0.0f, 0.4f, 0.4f}},  // cyan
    {{1.0f, 0.0f, 0.0f}},  // red
    {{0.4f, 0.0f, 0.4f}},  // magenta
    {{0.3f, 0.3f, 0.2f}},  // white
};
static constexpr int kNumFluidFaces = sizeof(kFluidFaces) / sizeof(kFluidFaces[0]);

// Cycles through hardcoded Minecraft block/mob icons; sits alongside the
// fluid colors in the same double-tap cycle.
static MinecraftFace kMinecraftFace;

// A simplified tilt-controlled Flappy Bird; also part of the double-tap cycle.
static FlappyFace kFlappyFace;

// A classic tilt-controlled Snake; also part of the double-tap cycle.
static SnakeFace kSnakeFace;

static constexpr int kNumFaces = kNumFluidFaces + 3;
static std::array<Face *, kNumFaces> kFacePtrs;

int main()
{
    if (DEV_Module_Init() != 0) {
        return -1;
    }

    QMI8658_init();
    WS2812_init();

    for (int i = 0; i < kNumFluidFaces; i++) {
        kFacePtrs[i] = &kFluidFaces[i];
    }
    kFacePtrs[kNumFluidFaces] = &kMinecraftFace;
    kFacePtrs[kNumFluidFaces + 1] = &kFlappyFace;
    kFacePtrs[kNumFluidFaces + 2] = &kSnakeFace;
    static FaceSwitcher switcher(kFacePtrs);

    uint64_t lastUs = time_us_64();

    while (1)
    {
        DEV_Delay_ms(LOOP_DELAY_MS);

        uint64_t nowUs = time_us_64();
        float dtSeconds = clampf((float)(nowUs - lastUs) / 1000000.0f, MIN_DT_S, MAX_DT_S);
        uint32_t dtUs = (uint32_t)(dtSeconds * 1000000.0f);
        lastUs = nowUs;

        float acc[3], gyro[3];
        unsigned int timCount;
        QMI8658_read_xyz(acc, gyro, &timCount);

        ImuSample sample;
        sample.accel[0] = acc[0];
        sample.accel[1] = acc[1];
        sample.accel[2] = acc[2];
        sample.gyro[0] = gyro[0];
        sample.gyro[1] = gyro[1];
        sample.gyro[2] = gyro[2];

        float accelMag = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        if (accelMag > 0.0f) {
            sample.orientation[0] = acc[0] / accelMag;
            sample.orientation[1] = acc[1] / accelMag;
            sample.orientation[2] = acc[2] / accelMag;
        } else {
            sample.orientation[0] = sample.orientation[1] = sample.orientation[2] = 0.0f;
        }

        switcher.feedImu(sample);
        FaceFrame frame = switcher.getFrame(dtUs);

        WS2812_clear();
        for (int x = 0; x < FACE_WIDTH; x++)
        {
            for (int y = 0; y < FACE_HEIGHT; y++)
            {
                const Color &p = frame.pixels[x][y];
                float targets[3] = {p.r * kBrightness, p.g * kBrightness, p.b * kBrightness};

                // Each channel's true (fractional, brightness-scaled) target
                // is accumulated over time and only the integer part is
                // ever sent to the LED; the fractional remainder carries
                // into the next frame. So e.g. a target of 0.4 shows as 1
                // on ~40% of frames and 0 the rest, averaging out to the
                // right brightness and hue without ever needing to round a
                // small value down to a flat, wrong zero.
                uint8_t out[3];
                for (int c = 0; c < 3; c++) {
                    ditherAcc[x][y][c] += targets[c];
                    float level = floorf(ditherAcc[x][y][c]);
                    ditherAcc[x][y][c] -= level;
                    out[c] = (uint8_t)level;
                }
                WS2812_set_pixel(x, y, out[0], out[1], out[2]);
            }
        }
        WS2812_show();
    }

    DEV_Module_Exit();
}
