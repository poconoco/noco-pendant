#include "WS2812.h"
#include "WS2812.pio.h"

LEDController led_ctrl;
PIO pio = pio0;
int sm = 0;

// WS2812 init
void WS2812_init()
{
    // Init led_ctrl
    led_ctrl.num_leds = NUM_LEDS;
    led_ctrl.width = WIDTH;
    led_ctrl.height = HEIGHT;
    led_ctrl.brightness = LED_BRIGHTNESS;

    // Configure the PIO
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_LED_IN, 800000, false);
}

// Send data
static void put_pixel(uint32_t pixel_grb)
{
    pio_sm_put_blocking(pio, 0, pixel_grb);
}

// Convert (x,y) position to LED index
static uint8_t pos_to_index(uint8_t x, uint8_t y) {
    return led_ctrl.width * x + y;
}

// Set a pixel at (x,y) with RGB color
void WS2812_set_pixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b) {
    if (x < led_ctrl.width && y < led_ctrl.height) {
        uint8_t index = pos_to_index(x, y);
        led_ctrl.leds[index].r = r > led_ctrl.brightness ? led_ctrl.brightness : r;
        led_ctrl.leds[index].g = g > led_ctrl.brightness ? led_ctrl.brightness : g;
        led_ctrl.leds[index].b = b > led_ctrl.brightness ? led_ctrl.brightness : b;
    }
}

// Clear all led_ctrl.leds (set to black)
void WS2812_clear() {
    for (int i = 0; i < NUM_LEDS; i++) {
        led_ctrl.leds[i].r = 0;
        led_ctrl.leds[i].g = 0;
        led_ctrl.leds[i].b = 0;
    }
}

// Update the LED strip with current colors
void WS2812_show() {
    for (int i = 0; i < NUM_LEDS; i++) {
        uint32_t rgb = (led_ctrl.leds[i].r) << 24 | (led_ctrl.leds[i].g) << 16 | (led_ctrl.leds[i].b) << 8;
        put_pixel(rgb);
    }
}

// Update the LED strip with current colors
void WS2812_show2(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < NUM_LEDS; i++) {
        uint32_t rgb = (r << 24) | (g << 16) | (b << 8);
        put_pixel(rgb);
    }
}
