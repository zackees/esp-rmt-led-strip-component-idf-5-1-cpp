
#pragma once

#include <stdint.h>

#include "led_strip_types.h"
#include "led_strip/rmt_strip.h"


enum LedStripMode {
    WS2812,
    kSK6812,
    WS2812_RGBW,
    kSK6812_RGBW,
};

class ColorCycle {
public:
    ColorCycle(uint32_t num_leds, bool rgb_active): mNumLeds(num_leds), mRgbwActive(rgb_active) {}
    void draw_loop(led_strip_handle_t led_strip);
    void draw_loop(IRmtLedStrip* led_strip);
private:
    uint32_t mNumLeds;
    bool mRgbwActive;
};

// void demo(int pin1, int pin2, uint32_t num_leds, LedStripMode mode);


void to_esp_modes(LedStripMode mode, led_model_t* out_chipset, led_pixel_format_t* out_rgbw);

inline bool is_rgbw_mode_active(led_pixel_format_t rgbw_mode) {
    return rgbw_mode == LED_PIXEL_FORMAT_GRBW;
}