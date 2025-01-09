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
#define LED_STRIP_GPIO_PIN 2
#endif

#ifndef LED_STRIP_LED_COUNT
// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 24
#endif

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)

static const char *TAG = "example";

enum dma_mode_t {
    DMA_AUTO,
    DMA_ENABLED,
    DMA_DISABLED,
};

led_strip_handle_t configure_led_with_timings(int pin, uint32_t led_count, bool is_rgbw, uint32_t t0h, uint32_t t0l, uint32_t t1h, uint32_t t1l, uint32_t reset, dma_mode_t dma_config)
{
    bool use_dma = false;
    if (dma_config == DMA_ENABLED)
    {
        use_dma = true;
    }

    led_strip_encoder_timings_t timings = {
        .t0h = t0h,
        .t1h = t1h,
        .t0l = t0l,
        .t1l = t1l,
        .reset = reset};

    // is always going to fail, so it's disabled for now.
    uint32_t memory_block_symbols = use_dma ? 1024 : 0;
    led_color_component_format_t color_component_format =
        is_rgbw ? LED_STRIP_COLOR_COMPONENT_FMT_RGBW : LED_STRIP_COLOR_COMPONENT_FMT_RGB;

    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = pin,                            // The GPIO that connected to the LED strip's data line
        .max_leds = led_count,                            // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,                    // LED strip model
        .color_component_format = color_component_format, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        },
        .timings = timings};

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,            // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ,     // RMT counter clock frequency
        .mem_block_symbols = memory_block_symbols, // the memory size of each RMT channel, in words (4 bytes)
        .flags = {
            .with_dma = false, // DMA feature is available on chips like ESP32-S3/P4
        }};

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;

    return nullptr;
}

led_strip_handle_t configure_led(int pin, uint32_t led_count, led_model_t led_model, bool is_rgbw, dma_mode_t dma_config)
{
    bool use_dma = false;
    if (dma_config == DMA_ENABLED)
    {
        use_dma = true;
    }
    // is always going to fail, so it's disabled for now.
    uint32_t memory_block_symbols = use_dma ? 1024 : 0;
    led_color_component_format_t color_component_format =
        is_rgbw ? LED_STRIP_COLOR_COMPONENT_FMT_RGBW : LED_STRIP_COLOR_COMPONENT_FMT_RGB;

    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = pin,                            // The GPIO that connected to the LED strip's data line
        .max_leds = led_count,                            // The number of LEDs in the strip,
        .led_model = led_model,                           // LED strip model
        .color_component_format = color_component_format, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }};

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,            // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ,     // RMT counter clock frequency
        .mem_block_symbols = memory_block_symbols, // the memory size of each RMT channel, in words (4 bytes)
        .flags = {
            .with_dma = false, // DMA feature is available on chips like ESP32-S3/P4
        }};

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

class IRmtStrip
{
public:
    virtual ~IRmtStrip() {}
    virtual esp_err_t setPixel(uint32_t index, uint32_t red, uint32_t green, uint32_t blue) = 0;
    virtual esp_err_t setPixelRGBW(uint32_t index, uint32_t red, uint32_t green, uint32_t blue, uint32_t white) = 0;
    virtual void drawSync()
    {
        drawAsync();
        waitDone();
    }
    virtual void drawAsync() = 0;
    virtual void waitDone() = 0;
    virtual bool isDrawing() = 0;
};

class RmtStrip : public IRmtStrip
{
public:
    RmtStrip(int pin, uint32_t led_count, bool is_rgbw, uint32_t th0, uint32_t tl0, uint32_t th1, uint32_t tl1, uint32_t reset, dma_mode_t dma_config = DMA_AUTO)
        : mIsRgbw(is_rgbw)
    {
        led_strip_handle_t led_strip = configure_led_with_timings(pin, led_count, is_rgbw, th0, tl0, th1, tl1, reset, dma_config);
        mStrip = led_strip;
    }

    ~RmtStrip() override
    {
        waitDone();
        led_strip_del(mStrip);
        mStrip = nullptr;
    }

    esp_err_t setPixel(uint32_t index, uint32_t red, uint32_t green, uint32_t blue) override
    {
        ESP_RETURN_ON_FALSE(!mIsRgbw, ESP_ERR_INVALID_ARG, TAG, "cannot set RGB on RGBW strip");
        ESP_ERROR_CHECK(led_strip_set_pixel(mStrip, index, red, green, blue));
        return ESP_OK;
    }

    esp_err_t setPixelRGBW(uint32_t index, uint32_t red, uint32_t green, uint32_t blue, uint32_t white) override
    {
        ESP_RETURN_ON_FALSE(mIsRgbw, ESP_ERR_INVALID_ARG, TAG, "cannot set RGBW on RGB strip");
        ESP_ERROR_CHECK(led_strip_set_pixel_rgbw(mStrip, index, red, green, blue, white));
        return ESP_OK;
    }

    void drawAsync() override
    {
        if (mDrawIssued)
        {
            waitDone();
        }
        ESP_ERROR_CHECK(led_strip_refresh_async(mStrip));
        mDrawIssued = true;
    }

    void waitDone() override
    {
        if (!mDrawIssued)
        {
            // ESP_LOGE(TAG, "No draw issued, skipping wait");
            return;
        }
        ESP_ERROR_CHECK(led_strip_refresh_wait_done(mStrip));
        mDrawIssued = false;
    }

    bool isDrawing() override
    {
        return mDrawIssued;
    }

    void clear()
    {
        ESP_ERROR_CHECK(led_strip_clear(mStrip));
    }

    void fill_color(uint32_t red, uint32_t green, uint32_t blue)
    {
        for (int i = 0; i < LED_STRIP_LED_COUNT; i++)
        {
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
    // ws2812 timings
    uint32_t th0 = 300;   // ns
    uint32_t tl0 = 900;   // ns
    uint32_t th1 = 900;   // ns
    uint32_t tl1 = 300;   // ns
    uint32_t reset = 280; // us

    RmtStrip led_strip2(7, LED_STRIP_LED_COUNT, false, th0, tl0, th1, tl1, reset);
    RmtStrip led_strip3(8, LED_STRIP_LED_COUNT, false, th0, tl0, th1, tl1, reset);
    RmtStrip led_strip4(9, LED_STRIP_LED_COUNT, false, th0, tl0, th1, tl1, reset);
    RmtStrip led_strip1(6, LED_STRIP_LED_COUNT, false, th0, tl0, th1, tl1, reset);

    // RmtStrip* rmtstrips[] = {&led_strip1, &led_strip2, &led_strip3, &led_strip4};
    RmtStrip *rmtstrips[] = {&led_strip1, &led_strip2, &led_strip3, &led_strip4};
    bool led_on_off = false;

    ESP_LOGI(TAG, "Start blinking LED strip");
    while (1)
    {
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        if (led_on_off)
        {
            r = 5;
            g = 5;
            b = 5;
        }
        for (auto strip : rmtstrips)
        {
            strip->fill_color(r, g, b);
        }
        for (auto strip : rmtstrips)
        {
            strip->drawAsync();
        }

        led_on_off = !led_on_off;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
