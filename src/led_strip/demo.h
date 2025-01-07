
#pragma once

#include <stdint.h>

#include "led_strip_types.h"


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
private:
    uint32_t mNumLeds;
    bool mRgbwActive;
};

void demo(int led_strip_gpio, uint32_t num_leds, LedStripMode mode);
