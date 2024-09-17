
#include "platforms/esp/32/led_strip/led_strip.h"
#include "platforms/esp/32/led_strip/rmt_demo.h"
#include <FastLED.h>
#include <iostream>

#include <Arduino.h>

#include "rgbw.h"

extern "C" {
#include "esp_log.h"
}

// Only the ARDUINO_RMT backend seems to work?!?!
// #define USE_ARDUINO_RMT

#define TAG "dev.ino"

// How many leds in your strip?
#define NUM_LEDS 9

// For led chips like WS2812, which have a data line, ground, and power, you
// just need to define DATA_PIN.  For led chipsets that are SPI based (four
// wires - data, clock, ground, and power), like the LPD8806 define both
// DATA_PIN and CLOCK_PIN Clock pin only needed for SPI based chipsets when not
// using hardware SPI
#define DATA_PIN 9

// Define the array of leds
CRGB leds[NUM_LEDS];

void setup() {
    Serial.begin(9600);
    Serial.setDebugOutput(true);
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    delay(3000);
    ESP_LOGI(TAG, "Start blinking LED strip");
}

led_strip_handle_t configure_led(void) {
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {};
    //    .strip_gpio_num = DATA_PIN,   // The GPIO that connected to the LED
    //    strip's data line .max_leds = NUM_LEDS,        // The number of LEDs
    //    in the strip, .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel
    //    format of your LED strip .led_model = LED_MODEL_WS2812,            //
    //    LED strip model .flags.invert_out = false,                // whether
    //    to invert the output signal
    //};
    strip_config.strip_gpio_num = DATA_PIN;
    strip_config.max_leds = NUM_LEDS;
    strip_config.led_pixel_format = LED_PIXEL_FORMAT_GRB;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.flags.invert_out = 0;

    // LED strip backend configuration: RMT
    //     led_strip_rmt_config_t rmt_config = {
    // #if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    //         .rmt_channel = 0,
    // #else
    //         .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source
    //         can lead to different power consumption .resolution_hz =
    //         LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
    //         .flags.with_dma = false,               // DMA feature is
    //         available on ESP target like ESP32-S3
    // #endif
    //     };

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

/// Copied from adafruit. Working as expected.
void espShow(uint8_t pin, uint8_t *pixels, uint32_t numBytes,
             boolean is800KHz) {
    rmt_data_t led_data[numBytes * 8];

    if (!rmtInit(pin, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000)) {
        log_e("Failed to init RMT TX mode on pin %d", pin);
        return;
    }

    int i = 0;
    for (int b = 0; b < numBytes; b++) {
        for (int bit = 0; bit < 8; bit++) {
            if (pixels[b] & (1 << (7 - bit))) {
                led_data[i].level0 = 1;
                led_data[i].duration0 = 8;
                led_data[i].level1 = 0;
                led_data[i].duration1 = 4;
            } else {
                led_data[i].level0 = 1;
                led_data[i].duration0 = 4;
                led_data[i].level1 = 0;
                led_data[i].duration1 = 8;
            }
            i++;
        }
    }

    // pinMode(pin, OUTPUT);  // don't do this, will cause the rmt to disable!
    rmtWrite(pin, led_data, numBytes * 8, RMT_WAIT_FOR_EVER);
}

// THIS WORKS GREAT!
void arduino_rmt_loop() {
    // Time scaling factors for each component
    const int TIME_FACTOR_HUE = 60;
    const int TIME_FACTOR_SAT = 100;
    const int TIME_FACTOR_VAL = 100;

    uint32_t now = millis() >> 3;

    for (int i = 0; i < NUM_LEDS; i++) {
        // Use different noise functions for each LED and each color component
        uint8_t hue = inoise16(now * TIME_FACTOR_HUE, i * 1000, 0) >> 8;
        uint8_t sat = inoise16(now * TIME_FACTOR_SAT, i * 2000, 1000) >> 8;
        uint8_t val = inoise16(now * TIME_FACTOR_VAL, i * 3000, 2000) >> 8;

        // Map the noise to full range for saturation and value
        sat = map(sat, 0, 255, 30, 255);
        val = map(val, 0, 255, 100, 255);

        leds[i] = CHSV(hue, sat, val);
    }

    uint8_t rgbw_pixels[NUM_LEDS * 4];
    uint8_t *rgbw_pixels_ptr = rgbw_pixels;
    for (int i = 0; i < NUM_LEDS; i++) {
        rgb_2_rgbw(kRGBWExactColors, kRGBWDefaultColorTemp, leds[i].r,
                   leds[i].g, leds[i].b, 255, 255, 255, rgbw_pixels_ptr,
                   rgbw_pixels_ptr + 1, rgbw_pixels_ptr + 2,
                   rgbw_pixels_ptr + 3);
        rgbw_pixels_ptr += 4;
    }

    espShow(DATA_PIN, rgbw_pixels, sizeof(rgbw_pixels), true);
}

// WHY YOU NO WORK?!
void led_component_loop() {
    ESP_LOGI(TAG, "Starting loop");
    // rmt_demo(DATA_PIN, NUM_LEDS);
    led_strip_handle_t led_strip = configure_led();
    bool led_on_off = false;
    while (1) {
        ESP_LOGI(TAG, "Looping");
        if (led_on_off) {
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each
             * color */
            for (int i = 0; i < NUM_LEDS; i++) {
                // Yes, we are sending RGB instead of RGBW data, but something
                // should still appear.
                ESP_ERROR_CHECK(
                    led_strip_set_pixel(led_strip, i, 255, 255, 255));
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

void loop() {
#ifdef USE_ARDUINO_RMT
    // rmt_demo(DATA_PIN, NUM_LEDS);
    arduino_rmt_loop();
#else
    led_component_loop();
#endif
    delay(500);
}
