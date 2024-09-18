#include <Arduino.h>

#include "led_strip/led_strip.h"
#include "demo.h"
#include "esp_log.h"

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


led_strip_handle_t configure_led(int pin, uint32_t max_leds, led_model_t chipset, led_pixel_format_t rgbw) {
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = pin;
    strip_config.max_leds = max_leds;
    strip_config.led_pixel_format = rgbw;
    strip_config.led_model = chipset;
    strip_config.flags.invert_out = 0;

    // print out the values of the configuration
    ESP_LOGI(TAG, "strip_config.strip_gpio_num: %d",
             strip_config.strip_gpio_num);
    ESP_LOGI(TAG, "strip_config.max_leds: %d", strip_config.max_leds);
    ESP_LOGI(TAG, "strip_config.led_pixel_format: %d",
             strip_config.led_pixel_format);
    ESP_LOGI(TAG, "strip_config.led_model: %d", strip_config.led_model);
    ESP_LOGI(TAG, "strip_config.flags.invert_out: %d",
             strip_config.flags.invert_out);

    led_strip_rmt_config_t rmt_config = {};
    memset(&rmt_config, 0, sizeof(led_strip_rmt_config_t));
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = 10000000;
    rmt_config.mem_block_symbols = 64;
    rmt_config.flags.with_dma = false;

    ESP_LOGI(TAG, "rmt_config.clk_src: %d", rmt_config.clk_src);
    ESP_LOGI(TAG, "rmt_config.resolution_hz: %d", rmt_config.resolution_hz);
    ESP_LOGI(TAG, "rmt_config.mem_block_symbols: %d",
             rmt_config.mem_block_symbols);
    ESP_LOGI(TAG, "rmt_config.flags.with_dma: %d", rmt_config.flags.with_dma);

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(
        led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}




void demo(int led_strip_gpio, uint32_t num_leds, LedStripMode mode) {
    const int MAX_BRIGHTNESS = 5;
    led_pixel_format_t rgbw_mode = {};
    led_model_t chipset = {};
    to_esp_modes(mode, &chipset, &rgbw_mode);
    const bool is_rgbw_active = is_rgbw_mode_active(rgbw_mode);
    // rmt_demo(DATA_PIN, NUM_LEDS);
    led_strip_handle_t led_strip = configure_led(led_strip_gpio, num_leds, chipset, rgbw_mode);
    bool led_on_off = false;
    while (1) {
        ESP_LOGI(TAG, "Looping");
        if (led_on_off) {
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each
             * color */
            for (int i = 0; i < num_leds; i++) {
                // Yes, we are sending RGB instead of RGBW data, but something
                // should still appear.
                ESP_ERROR_CHECK(
                    led_strip_set_pixel_rgbw(led_strip, i, MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS));
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


LED_STRIP_NAMESPACE_END
