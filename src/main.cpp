
#include <Arduino.h>
#include "led_strip/rmt_demo.h"

#define TAG "dev.ino"

void setup() {
    Serial.begin(9600);
    Serial.setDebugOutput(true);
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    ESP_LOGI(TAG, "Start blinking LED strip");
}

// How many leds in your strip?
#define NUM_LEDS 9

// For led chips like WS2812, which have a data line, ground, and power, you
// just need to define DATA_PIN.  For led chipsets that are SPI based (four
// wires - data, clock, ground, and power), like the LPD8806 define both
// DATA_PIN and CLOCK_PIN Clock pin only needed for SPI based chipsets when not
// using hardware SPI
#define DATA_PIN 9

void loop() {

    rmt_demo(DATA_PIN, NUM_LEDS);
    delay(500);
}
