


#include <Arduino.h>
#include "led_strip/demo.h"
#include "led_strip/construct.h"
#include "led_strip/rmt_strip.h"

// How many leds in your strip?
#define NUM_LEDS 256

// For led chips like WS2812, which have a data line, ground, and power, you
// just need to define DATA_PIN.  For led chipsets that are SPI based (four
// wires - data, clock, ground, and power), like the LPD8806 define both
// DATA_PIN and CLOCK_PIN Clock pin only needed for SPI based chipsets when not
// using hardware SPI

#define PIN1 1
#define PIN2 6
#define PIN3 7
#define PIN4 8


#define TAG "main.cpp"


void setup() {
    Serial.begin(9600);
    Serial.setDebugOutput(true);
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    delay(2000);
    ESP_LOGI(TAG, "Start blinking LED strip");

}


void demo_low_level_api(int pin1, int pin2, uint32_t num_leds, LedStripMode mode) {
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
        .pin = pin1,
        .max_leds = num_leds,
        .rgbw = is_rgbw_active,
        .rmt_bytes_encoder_config = bytes_encoder_config,
        .reset_code = reset,
    };

    config_led_t led_strip_config2 = {
        .pin = pin2,
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

    led_strip_handle_t led_strip2 = 0;
    construct_new_led_strip(led_strip_config, &led_strip2);

    ColorCycle color_cycle(num_leds, is_rgbw_active);
    while (1) {
        uint32_t start = millis();
        color_cycle.draw_loop(led_strip);
        color_cycle.draw_loop(led_strip2);
        uint32_t diff = millis() - start;
        ESP_LOGE(TAG, "Time to draw: %d", diff);
    }
}

void demo_high_level_api(int pin1, int pin2, int pin3, int pin4, uint32_t num_leds, LedStripMode mode) {

    IRmtLedStrip* strip = create_rmt_led_strip(350, 800, 700, 600, 30000, pin1, NUM_LEDS, false);
    IRmtLedStrip* strip3 = create_rmt_led_strip(350, 800, 700, 600, 30000, pin3, NUM_LEDS, false);
    IRmtLedStrip* strip4 = create_rmt_led_strip(350, 800, 700, 600, 30000, pin4, NUM_LEDS, false);
    IRmtLedStrip* strip2 = create_rmt_led_strip(350, 800, 700, 600, 30000, pin2, NUM_LEDS, false);

    led_pixel_format_t rgbw_mode = {};
    led_model_t chipset = {};
    to_esp_modes(mode, &chipset, &rgbw_mode);
    const bool is_rgbw_active = is_rgbw_mode_active(rgbw_mode);
    ColorCycle color_cycle(num_leds, is_rgbw_active);
    while (1) {
        uint32_t start = millis();
        color_cycle.draw_loop(strip);
        color_cycle.draw_loop(strip2);
        color_cycle.draw_loop(strip3);
        color_cycle.draw_loop(strip4);
        uint32_t diff = millis() - start;
        ESP_LOGE(TAG, "Time to draw: %d", diff);
    }

    // never reached.
    delete strip;
    delete strip2;
    delete strip3;
    delete strip4;
}

void loop() {
    demo_high_level_api(PIN1, PIN2, PIN3, PIN4, NUM_LEDS, WS2812);
}
