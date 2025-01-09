
#include <Arduino.h>

#include "strip_rmt.h"
#include "strip_spi.h"

enum TestMode {
    SPI,
    RMT
};

TestMode test_rmt = SPI;

void app_main_rmt(void)
{
    static const char *TAG = "app_main_rmt";
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


void app_main_spi(void)
{
    static const char *TAG = "app_main_rmt";
    // led_strip_handle_t led_strip = configure_led(LED_STRIP_GPIO_PIN, LED_STRIP_LED_COUNT, LED_MODEL_WS2812, SPI2_HOST, DMA_AUTO);
    // SpiStrip *led_strip = new SpiStrip(LED_STRIP_GPIO_PIN, LED_STRIP_LED_COUNT, LED_MODEL_WS2812, SPI2_HOST);
    ISpiStripWs2812* led_strip = ISpiStripWs2812::Create(LED_STRIP_GPIO_PIN, LED_STRIP_LED_COUNT);
    bool led_on_off = false;

    ESP_LOGI(TAG, "Start blinking LED strip");
    while (1) {
        if (led_on_off) {
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
            led_strip->fill(5,5,5);
            /* Refresh the strip to send data */
            ESP_LOGI(TAG, "LED ON!");
        } else {
            /* Set all LED off to clear all pixels */
            led_strip->fill(0,0,0);
            ESP_LOGI(TAG, "LED OFF!");
        }
        led_strip->drawSync();

        led_on_off = !led_on_off;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}



void setup() {
    delay(3000);
}

void loop() {
    if (test_rmt) {
        app_main_rmt();
    } else {
        app_main_spi();
    }
}

// spi_device_queue_trans
// spi_device_get_trans_result