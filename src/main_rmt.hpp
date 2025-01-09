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

#include "strip_rmt.h"


static const char *TAG = "example";


void app_main(void)
{
    // ws2812 timings
    uint32_t th0 = 300;   // ns
    uint32_t tl0 = 900;   // ns
    uint32_t th1 = 900;   // ns
    uint32_t tl1 = 300;   // ns
    uint32_t reset = 280; // us

    IRmtStrip* led_strip2 = IRmtStrip::Create(7, LED_STRIP_LED_COUNT, false, th0, tl0, th1, tl1, reset);
    IRmtStrip* led_strip3 = IRmtStrip::Create(8, LED_STRIP_LED_COUNT, false, th0, tl0, th1, tl1, reset);
    IRmtStrip* led_strip4 = IRmtStrip::Create(9, LED_STRIP_LED_COUNT, false, th0, tl0, th1, tl1, reset);
    IRmtStrip* led_strip1 = IRmtStrip::Create(6, LED_STRIP_LED_COUNT, false, th0, tl0, th1, tl1, reset);

    // RmtStrip* rmtstrips[] = {&led_strip1, &led_strip2, &led_strip3, &led_strip4};
    IRmtStrip *rmtstrips[] = {led_strip1, led_strip2, led_strip3, led_strip4};
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
            strip->fill(r, g, b);
        }
        for (auto strip : rmtstrips)
        {
            strip->drawAsync();
        }

        led_on_off = !led_on_off;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
