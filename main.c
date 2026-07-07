/*****************************************************************************
* | File      	:   main.c
* | Author      :   Waveshare Team
* | Function    :   FLIP/PIC water simulation on the 8x8 LED matrix, with
* |                 gravity driven by the onboard accelerometer's tilt, and
* |                 a double-tap gesture to cycle the water's color.
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
#include "DEV_Config.h"
#include "WS2812.h"
#include "QMI8658.h"
#include "fluid.h"
#include "pico/time.h"
#include <math.h>
#include <stdbool.h>

/*
 * RGB LED matrix coordinates description
 * =======================================
 * The program is set so that when the USB port is facing upward,
 * the upper left corner of the matrix is (0,0) and the lower right corner is (7,7)
 *
 * Water simulation
 * =================
 * A small FLIP/PIC fluid solver (see lib/fluid) runs on a 10x10 grid, whose
 * inner 8x8 cells map 1:1 onto the LED matrix. Gravity for the solver is
 * derived every frame from the accelerometer's in-plane tilt reading, so
 * tilting the board pours the water toward the lowered edge.
 *
 * Double-tapping the board's face cycles the water's color (see
 * DetectDoubleTap below).
 */

// Converts accelerometer tilt (in g) into simulation gravity units. Tuned by
// eye against the 10-unit-wide sim grid; raise for a snappier reaction.
#define GRAVITY_SCALE 500.0f

// Frame pacing: keep the physics/I2C loop well below the sensor+solver's
// natural speed so tilts feel smooth without hammering the I2C bus. The
// solver itself is cheap (small grid, ~70 particles), so this is the main
// lever for how fluid the animation looks; lower it further if there's headroom.
#define LOOP_DELAY_MS 0.025

// Simulated time step is clamped to this range so a slow frame (e.g. while
// debugging) can't destabilize the solver, and so the first frame's dt isn't zero.
#define MIN_DT 0.0005f
#define MAX_DT 0.05f

// Overall display brightness, independent of density: 0.0 = off, 1.0 =
// current maximum. Applied via per-pixel-per-channel dithering (see
// ditherAcc below) rather than directly scaling values down or blanking
// whole frames -- scaling shrinks the already-small 0..LED_BRIGHTNESS
// integer range further, leaving too few distinct levels for a smooth
// gradient or correct color-weight ratios; blanking whole frames dims
// correctly but flickers, since the entire display swings between lit and
// dark in lockstep. Dithering each channel independently spreads that same
// on/off averaging across 192 independent accumulators instead of one
// global one, so it settles on the right average brightness and hue without
// the whole matrix visibly strobing.
#define BRIGHTNESS .3f

// A cell needs to reach at least this fraction of the settled "full water"
// density before its LED turns on -- e.g. 0.5 means a cell must be at least
// half as dense as fully-settled water. Raise it to shrink the wet area down
// to only the densest parts of the water; lower it (down to 0.0) to light up
// on the faintest trace of water.
#define WATER_ON_THRESHOLD 0.25f

// Double-tap detection: a tap is a fast, brief spike on the Z axis (the
// board's face normal -- into/out of the LED matrix) *on top of whatever the
// board is currently doing* -- not just an absolute level, since slow
// handling/tilting also moves the Z reading around as the board tips. A
// slowly-adapting baseline tracks the "current normal" for that axis
// (gravity's current Z component plus any slow motion); only a sudden jump
// away from that baseline, lasting a moment and then dropping back, counts
// as a tap. Only using Z, rather than the full 3-axis magnitude, is what
// keeps this reacting to knocks on the face and not sideways nudges.
#define TAP_BASELINE_TAU_S 0.25f           // how fast the baseline follows slow motion (bigger = slower to follow, more resistant to being fooled by a tap)
#define TAP_DEVIATION_THRESHOLD_MG 2500.0f // spike size above baseline that counts as a tap candidate
#define TAP_RELEASE_DEVIATION_MG 900.0f    // must settle back this close to baseline before the next tap can fire
#define DOUBLE_TAP_WINDOW_MS 400

// A single knock isn't a clean spike -- the board/sensor rings for a bit and
// can cross the thresholds more than once. Anything this soon after an
// accepted tap is the same knock settling down, not a second tap, and is
// ignored.
#define TAP_DEBOUNCE_MS 80

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// Looks for the classic phone-style "double tap" gesture in the raw Z-axis
// accelerometer reading (az, in mg) and reports when one completes. Call
// this once per sample, with dt = seconds since the previous call; it tracks
// its own state between calls.
static bool DetectDoubleTap(float az, float dt, uint64_t nowUs) {
    static bool haveBaseline = false;
    static float baseline = 0.0f;
    static bool armed = true;
    static bool haveFirstTap = false;
    static uint64_t firstTapUs = 0;
    static bool haveLastTap = false;
    static uint64_t lastTapUs = 0;

    if (!haveBaseline) {
        baseline = az;
        haveBaseline = true;
    }

    float deviation = az - baseline;

    // Only let the baseline follow the signal while it's not in the middle
    // of a spike, so a tap doesn't drag its own reference point up with it.
    if (fabsf(deviation) < TAP_DEVIATION_THRESHOLD_MG) {
        float alpha = clampf(dt / TAP_BASELINE_TAU_S, 0.0f, 1.0f);
        baseline += alpha * (az - baseline);
    }

    // A knock can spike Z either positive or negative depending on which way
    // it jolts the sensor, so trigger on the deviation's magnitude.
    bool tapEvent = false;
    if (armed && fabsf(deviation) > TAP_DEVIATION_THRESHOLD_MG) {
        tapEvent = true;
        armed = false; // require settling back near baseline before the next tap can fire
    } else if (!armed && fabsf(deviation) < TAP_RELEASE_DEVIATION_MG) {
        armed = true;
    }

    if (!tapEvent) {
        if (haveFirstTap && (nowUs - firstTapUs) > (uint64_t)DOUBLE_TAP_WINDOW_MS * 1000) {
            haveFirstTap = false; // first tap went stale, forget it
        }
        return false;
    }

    if (haveLastTap && (nowUs - lastTapUs) < (uint64_t)TAP_DEBOUNCE_MS * 1000) {
        return false; // ringing from the same knock, not a new tap
    }
    lastTapUs = nowUs;
    haveLastTap = true;

    if (haveFirstTap && (nowUs - firstTapUs) <= (uint64_t)DOUBLE_TAP_WINDOW_MS * 1000) {
        haveFirstTap = false;
        return true;
    }

    firstTapUs = nowUs;
    haveFirstTap = true;
    return false;
}

// Water color palette, cycled by double-tap. Channel weights (0..1), applied
// on top of the density-driven intensity so the color itself never encodes
// the brightness.
typedef struct { float r, g, b; } ColorWeights;

static const ColorWeights COLOR_PALETTE[] = {
    {1.0f, 0.0f, 0.0f}, // red (startup default)
    {1.0f, 0.1f, 0.0f}, // orange
    {0.6f, 0.2f, 0.0f}, // yellow
    {0.0f, 0.75f, 0.0f}, // green
    {0.0f, 0.4f, 0.4f}, // cyan
    {0.0f, 0.0f, 0.8f}, // blue
    {0.4f, 0.0f, 0.4f}, // magenta
};
#define NUM_COLORS (sizeof(COLOR_PALETTE) / sizeof(COLOR_PALETTE[0]))

static Fluid fluid;

// Per-pixel-per-channel dither accumulators for BRIGHTNESS (see comment on
// BRIGHTNESS above); persists across frames.
static float ditherAcc[WIDTH][HEIGHT][3];

int main()
{
    if(DEV_Module_Init()!=0){
        return -1;
    }

    float acc[3], gyro[3];
    unsigned int tim_count;
    QMI8658_init();
    WS2812_init();

    Fluid_init(&fluid);

    int colorIndex = 0;
    uint64_t lastUs = time_us_64();

    while(1)
    {
        DEV_Delay_ms(LOOP_DELAY_MS);

        uint64_t nowUs = time_us_64();
        float dt = clampf((float)(nowUs - lastUs) / 1000000.0f, MIN_DT, MAX_DT);
        lastUs = nowUs;

        QMI8658_read_xyz(acc, gyro, &tim_count);

        if (DetectDoubleTap(acc[2], dt, nowUs)) {
            colorIndex = (colorIndex + 1) % NUM_COLORS;
        }

        // acc is in milli-g; the X/Y readings are the projection of gravity
        // onto the board's plane, i.e. exactly the "downhill" direction we
        // want the water to accelerate towards when the board is tilted.
        // Signs match the original single-dot demo's tilt convention.
        float ax = acc[0] / 1000.0f;
        float ay = acc[1] / 1000.0f;
        float gravityX = -ax * GRAVITY_SCALE;
        float gravityY = ay * GRAVITY_SCALE;

        Fluid_step(&fluid, dt, gravityX, gravityY);

        // Every cell at or above WATER_ON_THRESHOLD renders at full
        // intensity, tinted by the current color -- no halftones for water
        // depth/density, strictly on or off.
        ColorWeights color = COLOR_PALETTE[colorIndex];
        float restDensity = Fluid_restDensity(&fluid);
        WS2812_clear();
        for (int x = 0; x < WIDTH; x++)
        {
            for (int y = 0; y < HEIGHT; y++)
            {
                float d = Fluid_cellDensity(&fluid, x, y);
                float rel = restDensity > 0.0f ? d / restDensity : d;
                if (rel < WATER_ON_THRESHOLD) continue;

                float intensity = LED_BRIGHTNESS;

                // Each channel's true (fractional, BRIGHTNESS-scaled) target
                // is accumulated over time and only the integer part is ever
                // sent to the LED; the fractional remainder carries into the
                // next frame. So e.g. a target of 0.4 shows as 1 on ~40% of
                // frames and 0 the rest, averaging out to the right
                // brightness and hue without ever needing to round a small
                // value down to a flat, wrong zero.
                float targets[3] = {
                    intensity * color.r * BRIGHTNESS,
                    intensity * color.g * BRIGHTNESS,
                    intensity * color.b * BRIGHTNESS,
                };
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
