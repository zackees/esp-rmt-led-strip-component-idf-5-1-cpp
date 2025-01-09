/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#ifndef LED_STRIP_GPIO_PIN
// GPIO assignment
#define LED_STRIP_GPIO_PIN  2
#endif

#ifndef LED_STRIP_LED_COUNT
// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 24
#endif


// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

static const char *TAG = "example";

led_strip_handle_t configure_led(int pin, uint32_t led_count, led_model_t led_model, bool is_rgbw)
{
    const bool use_dma = false;  // there's a bug in the current implementation: using dma
    // is always going to fail, so it's disabled for now.
    uint32_t memory_block_symbols = use_dma ? 1024 : 0;
    led_color_component_format_t color_component_format =
        is_rgbw ? LED_STRIP_COLOR_COMPONENT_FMT_RGBW : LED_STRIP_COLOR_COMPONENT_FMT_RGB;
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = pin, // The GPIO that connected to the LED strip's data line
        .max_leds = led_count,      // The number of LEDs in the strip,
        .led_model = led_model,        // LED strip model
        .color_component_format = color_component_format, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = memory_block_symbols,               // the memory size of each RMT channel, in words (4 bytes)
        .flags = {
            .with_dma = false, // DMA feature is available on chips like ESP32-S3/P4
        }
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}


class RmtStrip {
 public:
  RmtStrip(int pin, uint32_t led_count, led_model_t led_model, bool is_rgbw): mIsRgbw(is_rgbw) {
    led_strip_handle_t led_strip = configure_led(pin, led_count, led_model, is_rgbw);
    mStrip = led_strip;
  }

  ~RmtStrip() {
    wait_done();
    led_strip_del(mStrip);
    mStrip = nullptr;
  }

  esp_err_t setPixel(uint32_t index, uint32_t red, uint32_t green, uint32_t blue) {
    ESP_RETURN_ON_FALSE(!mIsRgbw, ESP_ERR_INVALID_ARG, TAG, "cannot set RGB on RGBW strip");
    ESP_ERROR_CHECK(led_strip_set_pixel(mStrip, index, red, green, blue));
    return ESP_OK;
  }

  esp_err_t setPixelRGBW(uint32_t index, uint32_t red, uint32_t green, uint32_t blue, uint32_t white) {
    ESP_RETURN_ON_FALSE(mIsRgbw, ESP_ERR_INVALID_ARG, TAG, "cannot set RGBW on RGB strip");
    ESP_ERROR_CHECK(led_strip_set_pixel_rgbw(mStrip, index, red, green, blue, white));
    return ESP_OK;
  }

  void draw_sync() {
    draw_async();
    wait_done();
  }

  void draw_async() {
    if (mDrawIssued) {
        wait_done();
    }
    ESP_ERROR_CHECK(led_strip_refresh_async(mStrip));
    mDrawIssued = true;
  }

  void wait_done() {
    if (!mDrawIssued) {
        //ESP_LOGE(TAG, "No draw issued, skipping wait");
        return;
    }
    ESP_ERROR_CHECK(led_strip_refresh_wait_done(mStrip));
    mDrawIssued = false;
  }

  bool is_drawing() {
    return mDrawIssued;
  }

  void clear() {
    ESP_ERROR_CHECK(led_strip_clear(mStrip));
  }

  void fill_color(uint32_t red, uint32_t green, uint32_t blue) {
    for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
      setPixel(i, red, green, blue);
    }
  }

  private:
    led_strip_handle_t mStrip;
    bool mDrawIssued = false;
    bool mIsRgbw;
};

void app_main(void)
{
    // led_strip_handle_t led_strip = configure_led(6, LED_STRIP_LED_COUNT, LED_MODEL_WS2812);

    RmtStrip led_strip2(7, LED_STRIP_LED_COUNT, LED_MODEL_WS2812, false);
    RmtStrip led_strip3(8, LED_STRIP_LED_COUNT, LED_MODEL_WS2812, false);
    RmtStrip led_strip4(9, LED_STRIP_LED_COUNT, LED_MODEL_WS2812, false);
    RmtStrip led_strip1(6, LED_STRIP_LED_COUNT, LED_MODEL_WS2812, false);
    
    // RmtStrip* rmtstrips[] = {&led_strip1, &led_strip2, &led_strip3, &led_strip4};
    RmtStrip* rmtstrips[] = {&led_strip1, &led_strip2, &led_strip3, &led_strip4};
    bool led_on_off = false;

    ESP_LOGI(TAG, "Start blinking LED strip");
    while (1) {
        if (led_on_off) {
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
            for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
                //ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 5, 5, 5));
                // led_strip.setPixel(i, 5, 5, 5);
                for (auto strip : rmtstrips) {
                    strip->setPixel(i, 5, 5, 5);
                }
            }
            /* Refresh the strip to send data */
            // ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            // led_strip.refresh();
            // led_strip.draw_async();
            for (auto strip : rmtstrips) {
                strip->draw_async();
            }
            // ESP_LOGI(TAG, "LED ON!");
        } else {
            /* Set all LED off to clear all pixels */
            // ESP_ERROR_CHECK(led_strip_clear(led_strip));
            // led_strip.clear();
            // led_strip.fill_color(0, 0, 0);
            for (auto strip : rmtstrips) {
                strip->fill_color(0, 0, 0);
            }
            //led_strip.draw_async();
            for (auto strip : rmtstrips) {
                strip->draw_async();
            }
            //ESP_LOGI(TAG, "LED OFF!");
        }

        led_on_off = !led_on_off;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
