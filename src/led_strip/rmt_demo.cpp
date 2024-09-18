#include <Arduino.h>

#include "led_strip/led_strip.h"
#include "led_strip/rmt_demo.h"
#include "esp_log.h"

#define TAG "rmt_demo.cpp"

#include "namespace.h"
LED_STRIP_NAMESPACE_BEGIN

led_strip_handle_t configure_led(int pin, uint32_t max_leds) {
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = pin;
    strip_config.max_leds = max_leds;
    strip_config.led_pixel_format = LED_PIXEL_FORMAT_GRBW;
    strip_config.led_model = LED_MODEL_WS2812;
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


void led_component_loop(int pin, uint32_t max_leds) {
    const int MAX_BRIGHTNESS = 5;
    ESP_LOGI(TAG, "Starting loop");
    // rmt_demo(DATA_PIN, NUM_LEDS);
    led_strip_handle_t led_strip = configure_led(pin, max_leds);
    bool led_on_off = false;
    while (1) {
        ESP_LOGI(TAG, "Looping");
        if (led_on_off) {
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each
             * color */
            for (int i = 0; i < max_leds; i++) {
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


void rmt_demo(int led_strip_gpio, uint32_t num_leds) {
    // TODO: handle rmt_res_hz
    led_component_loop(led_strip_gpio, num_leds);
}


LED_STRIP_NAMESPACE_END