#include <Arduino.h>

#include "led_strip.h"
#include "demo.h"
#include "esp_log.h"
#include "configure_led.h"

#include "construct.h"



#define TAG "rmt_demo.cpp"

// #define DRAW_BLINK_DEMO




void ColorCycle::draw_loop(led_strip_handle_t led_strip) {
    const int MAX_BRIGHTNESS = 64;
    uint32_t now = millis();
    double now_f = now / 1000.0;

    for (int i = 0; i < mNumLeds; i++) {
        float hue = fmodf(now_f + (float)i / mNumLeds, 1.0f);
        float r = MAX_BRIGHTNESS * (0.5f + 0.5f * std::sin(2 * PI * (hue + 0.0f / 3.0f)));
        float g = MAX_BRIGHTNESS * (0.5f + 0.5f * std::sin(2 * PI * (hue + 1.0f / 3.0f)));
        float b = MAX_BRIGHTNESS * (0.5f + 0.5f * std::sin(2 * PI * (hue + 2.0f / 3.0f)));
        set_pixel(led_strip, i, mRgbwActive, r, g, b);
    }
    draw_strip(led_strip);
}



void to_esp_modes(LedStripMode mode, led_model_t* out_chipset, led_pixel_format_t* out_rgbw) {
    switch (mode) {
        case WS2812:
            *out_rgbw = LED_PIXEL_FORMAT_GRB;
            *out_chipset = LED_MODEL_WS2812;
            break;
        case kSK6812:
            *out_rgbw = LED_PIXEL_FORMAT_GRB;
            *out_chipset = LED_MODEL_SK6812;
            break;
        case WS2812_RGBW:
            *out_rgbw = LED_PIXEL_FORMAT_GRBW;
            *out_chipset = LED_MODEL_WS2812;
            break;
        case kSK6812_RGBW:
            *out_rgbw = LED_PIXEL_FORMAT_GRBW;
            *out_chipset = LED_MODEL_SK6812;
            break;
        default:
            ESP_LOGE(TAG, "Invalid LedStripMode");
            break;
    }
}

bool is_rgbw_mode_active(led_pixel_format_t rgbw_mode) {
    return rgbw_mode == LED_PIXEL_FORMAT_GRBW;
}

void convert_to_rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t* out_r, uint8_t* out_g, uint8_t* out_b, uint8_t* out_w) {
    // This is a simple conversion that just takes the average of the RGB values and assigns it to the W value.
    // This is not a good conversion, but it is a simple one.
    uint8_t w = max(r, max(g, b));
    r = r - w;
    g = g - w;
    b = b - w;
    *out_r = r;
    *out_g = g;
    *out_b = b;
    *out_w = w;
}

void set_pixel(led_strip_handle_t led_strip, uint32_t index, bool is_rgbw_active, uint8_t r, uint8_t g, uint8_t b) {
    if (is_rgbw_active) {
        uint8_t w = 0;
        convert_to_rgbw(r, g, b, &r, &g, &b, &w);
        ESP_ERROR_CHECK(led_strip_set_pixel_rgbw(led_strip, index, g, r, b, w));
    } else {
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, index, g, r, b));
    }
}

void draw_strip(led_strip_handle_t led_strip) {
    ESP_ERROR_CHECK(led_strip_refresh_async(led_strip));
    ESP_ERROR_CHECK(led_strip_wait_refresh_done(led_strip, portMAX_DELAY, true));
}


void draw_loop_color_cycle(led_strip_handle_t led_strip, uint32_t num_leds, bool rgbw_active) {
    ColorCycle color_cycle(num_leds, rgbw_active);
    while (1) {
        color_cycle.draw_loop(led_strip);
    }
}

void draw_loop_blink_on_off_white(led_strip_handle_t led_strip, uint32_t num_leds, bool rgbw_active) {
    const int MAX_BRIGHTNESS = 5;
    bool led_on_off = false;
    while (1) {
        ESP_LOGI(TAG, "Looping");
        if (led_on_off) {
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each
             * color */
            uint8_t r = MAX_BRIGHTNESS;
            uint8_t g = MAX_BRIGHTNESS;
            uint8_t b = MAX_BRIGHTNESS;
            for (int i = 0; i < num_leds; i++) {
                set_pixel(led_strip, i, rgbw_active, r, g, b);
            }
            /* Refresh the strip to send data */
            draw_strip(led_strip);
            ESP_LOGI(TAG, "LED ON!");
            vTaskDelay(pdMS_TO_TICKS(8));
        } else {
            /* Set all LED off to clear all pixels */
            ESP_ERROR_CHECK(led_strip_clear(led_strip));
            ESP_LOGI(TAG, "LED OFF!");
        }

        led_on_off = !led_on_off;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void draw_loop(led_strip_handle_t led_strip, uint32_t num_leds, bool rgbw_active) {
    ESP_LOGE(TAG, "LOOP!");
    #ifdef DRAW_BLINK_DEMO
    draw_loop_blink_on_off_white(led_strip, num_leds, rgbw_active);
    #else
    draw_loop_color_cycle(led_strip, num_leds, rgbw_active);
    #endif
}

// T0H (Time for logic '0' high):

// Typical: 0.35 µs
// Min: 0.2 µs
// Max: 0.5 µs
// T0L (Time for logic '0' low):

// Typical: 0.8 µs
// Min: 0.65 µs
// Max: 0.95 µs
// T1H (Time for logic '1' high):

// Typical: 0.7 µs
// Min: 0.55 µs
// Max: 0.9 µs
// T1L (Time for logic '1' low):

// Typical: 0.6 µs
// Min: 0.45 µs
// Max: 0.8 µs

void demo(int led_strip_gpio, uint32_t num_leds, LedStripMode mode) {
    led_pixel_format_t rgbw_mode = {};
    led_model_t chipset = {};
    to_esp_modes(mode, &chipset, &rgbw_mode);
    const bool is_rgbw_active = is_rgbw_mode_active(rgbw_mode);


    // const uint16_t T0H = 35;
    // const uint16_t T0L = 80;
    // const uint16_t T1H = 70;
    // const uint16_t T1L = 60;
    // const uint32_t TRESET = 30000;  # nano seconds
    const uint16_t T0H = 350;
    const uint16_t T0L = 800;
    const uint16_t T1H = 700;
    const uint16_t T1L = 600;
    const uint32_t TRESET = 30000;  // nano seconds
    rmt_symbol_word_t reset;

    rmt_bytes_encoder_config_t bytes_encoder_config = make_encoder_config(
        T0H, T0L, T1H, T1L, TRESET, &reset);

    config_led_t led_strip_config = {
        .pin = led_strip_gpio,
        .max_leds = num_leds,
        .rgbw = is_rgbw_active,
        .rmt_bytes_encoder_config = bytes_encoder_config,
        .reset_code = reset,
    };


    //make_led_config(led_strip_gpio, num_leds, chipset, rgbw_mode, &led_strip_config);
    // make_led_config(T0H, T0L, T1H, T1L, TRESET, led_strip_gpio, num_leds, is_rgbw_active, nullptr);
    // construct_new_led_strip(led_strip_gpio, num_leds, chipset, rgbw_mode);
    led_strip_handle_t led_strip = 0;
    construct_new_led_strip(led_strip_config, &led_strip);

    ColorCycle color_cycle(num_leds, is_rgbw_active);
    while (1) {
        color_cycle.draw_loop(led_strip);
    }

}

