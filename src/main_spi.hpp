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

#ifndef LED_STRIP_GPIO_PIN
// GPIO assignment
#define LED_STRIP_GPIO_PIN  2
#endif

#ifndef LED_STRIP_LED_COUNT
// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 24
#endif

static const char *TAG = "example";

led_strip_handle_t configure_led(int pin, uint32_t led_count, led_model_t led_model, spi_host_device_t spi_bus, dma_mode_t dma_mode)
{
    bool with_dma = dma_mode == DMA_ENABLED || dma_mode == DMA_AUTO;
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = pin, // The GPIO that connected to the LED strip's data line
        .max_leds = led_count,      // The number of LEDs in the strip,
        .led_model = led_model,        // LED strip model
        // set the color order of the strip: GRB
        .color_component_format = {
            .format = {
                .r_pos = 0, // red is the second byte in the color data
                .g_pos = 1, // green is the first byte in the color data
                .b_pos = 2, // blue is the third byte in the color data
                .num_components = 3, // total 3 color components
            },
        },
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    // LED strip backend configuration: SPI
    led_strip_spi_config_t spi_config = {
        .clk_src = SPI_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
        .spi_bus = spi_bus,           // SPI bus ID
        .flags = {
            .with_dma = with_dma, // Using DMA can improve performance and help drive more LEDs
        }
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with SPI backend");
    return led_strip;
}


class ISpiStrip
{
public:
    virtual ~ISpiStrip() {}
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

class SpiStrip : public ISpiStrip {
public:
    SpiStrip(int pin, uint32_t led_count, led_model_t led_model, spi_host_device_t spi_bus = SPI2_HOST, dma_mode_t dma_mode = DMA_AUTO)
        : mIsRgbw(false) // SPI implementation currently only supports RGB
    {
        led_strip_handle_t led_strip = configure_led(pin, led_count, led_model, spi_bus, dma_mode);
        mStrip = led_strip;
    }

    ~SpiStrip() override
    {
        waitDone();
        led_strip_del(mStrip);
        mStrip = nullptr;
    }

    esp_err_t setPixel(uint32_t index, uint32_t red, uint32_t green, uint32_t blue) override
    {
        ESP_ERROR_CHECK(led_strip_set_pixel(mStrip, index, red, green, blue));
        return ESP_OK;
    }

    esp_err_t setPixelRGBW(uint32_t index, uint32_t red, uint32_t green, uint32_t blue, uint32_t white) override
    {
        return ESP_ERR_NOT_SUPPORTED; // SPI implementation doesn't support RGBW
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
    // led_strip_handle_t led_strip = configure_led(LED_STRIP_GPIO_PIN, LED_STRIP_LED_COUNT, LED_MODEL_WS2812, SPI2_HOST, DMA_AUTO);
    // SpiStrip *led_strip = new SpiStrip(LED_STRIP_GPIO_PIN, LED_STRIP_LED_COUNT, LED_MODEL_WS2812, SPI2_HOST);
    SpiStrip led_strip(LED_STRIP_GPIO_PIN, LED_STRIP_LED_COUNT, LED_MODEL_WS2812, SPI2_HOST);
    bool led_on_off = false;

    ESP_LOGI(TAG, "Start blinking LED strip");
    while (1) {
        if (led_on_off) {
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
            led_strip.fill_color(5,5,5);
            /* Refresh the strip to send data */
            // ESP_ERROR_CHECK(led_strip_refresh(led_strip));

            ESP_LOGI(TAG, "LED ON!");
        } else {
            /* Set all LED off to clear all pixels */
            // ESP_ERROR_CHECK(led_strip_clear(led_strip));
            led_strip.fill_color(0,0,0);
            ESP_LOGI(TAG, "LED OFF!");
        }
        led_strip.drawSync();

        led_on_off = !led_on_off;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
