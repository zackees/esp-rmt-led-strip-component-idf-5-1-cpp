#include <Arduino.h>

#include "led_strip/led_strip.h"
#include "demo.h"
#include "esp_log.h"
#include "configure_led.h"

#include "namespace.h"
LED_STRIP_NAMESPACE_BEGIN

#define TAG "rmt_demo.cpp"

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

void draw_loop(led_strip_handle_t led_strip, uint32_t num_leds, bool rgbw_active) {
    const int MAX_BRIGHTNESS = 5;
    bool led_on_off = false;
    while (1) {
        ESP_LOGI(TAG, "Looping");
        if (led_on_off) {
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each
             * color */
            for (int i = 0; i < num_leds; i++) {
                // Yes, we are sending RGB instead of RGBW data, but something
                // should still appear.
                if (rgbw_active) {
                    ESP_ERROR_CHECK(
                        led_strip_set_pixel_rgbw(led_strip, i, MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS));
                } else {
                    ESP_ERROR_CHECK(
                        led_strip_set_pixel(led_strip, i, MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS));
                }
            }
            /* Refresh the strip to send data */
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
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
void demo(int led_strip_gpio, uint32_t num_leds, LedStripMode mode) {
    led_pixel_format_t rgbw_mode = {};
    led_model_t chipset = {};
    to_esp_modes(mode, &chipset, &rgbw_mode);
    const bool is_rgbw_active = is_rgbw_mode_active(rgbw_mode);
    led_strip_handle_t led_strip = configure_led(led_strip_gpio, num_leds, chipset, rgbw_mode);
    draw_loop(led_strip, num_leds, is_rgbw_active);
}


LED_STRIP_NAMESPACE_END
