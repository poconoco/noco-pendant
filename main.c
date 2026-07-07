/*****************************************************************************
* | File      	:   main.c
* | Author      :   Waveshare Team
* | Function    :   FLIP/PIC water simulation on the 8x8 LED matrix, with
* |                 gravity driven by the onboard accelerometer's tilt.
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

// Overall display brightness, independent of density: 0.0 = off,
// 1.0 = current maximum (equivalent to LED_BRIGHTNESS). Turn this down
// instead of touching WS2812's LED_BRIGHTNESS.
#define BRIGHTNESS .3f

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static Fluid fluid;

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

    uint64_t lastUs = time_us_64();

    while(1)
    {
        DEV_Delay_ms(LOOP_DELAY_MS);

        uint64_t nowUs = time_us_64();
        float dt = clampf((float)(nowUs - lastUs) / 1000000.0f, MIN_DT, MAX_DT);
        lastUs = nowUs;

        QMI8658_read_xyz(acc, gyro, &tim_count);

        // acc is in milli-g; the X/Y readings are the projection of gravity
        // onto the board's plane, i.e. exactly the "downhill" direction we
        // want the water to accelerate towards when the board is tilted.
        // Signs match the original single-dot demo's tilt convention.
        float ax = acc[0] / 1000.0f;
        float ay = acc[1] / 1000.0f;
        float gravityX = -ax * GRAVITY_SCALE;
        float gravityY = ay * GRAVITY_SCALE;

        Fluid_step(&fluid, dt, gravityX, gravityY);

        // Render local water density as red brightness.
        float restDensity = Fluid_restDensity(&fluid);
        WS2812_clear();
        for (int x = 0; x < WIDTH; x++)
        {
            for (int y = 0; y < HEIGHT; y++)
            {
                float d = Fluid_cellDensity(&fluid, x, y);
                if (d <= 0.0f) continue;

                float rel = restDensity > 0.0f ? d / restDensity : d;
                float redF = clampf(rel * LED_BRIGHTNESS, 1.0f, 255.0f);

                uint8_t red = (uint8_t)(redF * BRIGHTNESS + 0.5f);
                WS2812_set_pixel(x, y, red, 0, 0);
            }
        }
        WS2812_show();
    }

    DEV_Module_Exit();
}
