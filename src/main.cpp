
#ifdef LED_STRIP_HAS_MAIN_H

#include <Arduino.h>
#include "led_strip/demo.h"
#include "led_strip/namespace.h"

// How many leds in your strip?
#define NUM_LEDS 9

// For led chips like WS2812, which have a data line, ground, and power, you
// just need to define DATA_PIN.  For led chipsets that are SPI based (four
// wires - data, clock, ground, and power), like the LPD8806 define both
// DATA_PIN and CLOCK_PIN Clock pin only needed for SPI based chipsets when not
// using hardware SPI
#define DATA_PIN 9


#define TAG "main.cpp"


void setup() {
    Serial.begin(9600);
    Serial.setDebugOutput(true);
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    ESP_LOGI(TAG, "Start blinking LED strip");
}


void loop() {
    led_strip::demo(DATA_PIN, NUM_LEDS, led_strip::WS2812_RGBW);
    delay(500);
}

#endif // _LED_STRIP_HAS_MAIN_H