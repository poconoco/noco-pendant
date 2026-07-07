/*****************************************************************************
* | File      	:   main.c
* | Author      :   Waveshare Team
* | Function    :   Control the RGB LED to move in the direction the board is tilted
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

/* 
 * RGB LED moving program coordinates description
 * ==============================================
 * The program is set so that when the USB port is facing upward,
 * the upper left corner of the matrix is (0,0) and the lower right corner is (7,7)
 * Initial position (5,5)
 */

int main() 
{
    if(DEV_Module_Init()!=0){
        return -1;
    }

    float acc[3], gyro[3];
    unsigned int tim_count;
    bool move = false;
    QMI8658_init();
    WS2812_init();

    int x = 5, y = 5;
    float threshold = 100;  // Acceleration threshold
    int move_delay = 100;   // Move interval (ms)

    while(1)
    {
        DEV_Delay_ms(move_delay);
        QMI8658_read_xyz(acc, gyro, &tim_count);

        /*
         * X-axis acceleration controls the X direction
         * Y-axis acceleration controls the Y direction
         */ 

        // X-axis control (development board tilts up and down)
        if(acc[0] < -threshold)
        {
            x++;
            if(x > 7)x = 7;
            move = true;
        }
        else if(acc[0] > threshold)
        {
            x--;
            if(x < 0)x = 0;
            move = true;
        }

        // Y-axis control (development board tilts left and right)
        if(acc[1] < -threshold)
        {
            y--;
            if(y < 0)y = 0;
            move = true;
        } 
        else if(acc[1] > threshold)
        {
            y++;
            if(y > 7)y = 7;
            move = true;
        }

        // Update the display
        if(move)
        {
            WS2812_clear();
            WS2812_set_pixel(x, y, 0x00, 0x00, 0xFF);
            WS2812_show();
            move = false;
        }
    }
    
    DEV_Module_Exit();
}
