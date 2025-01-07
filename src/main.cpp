


#include <Arduino.h>
#include "led_strip/demo.h"

// How many leds in your strip?
#define NUM_LEDS 16

// For led chips like WS2812, which have a data line, ground, and power, you
// just need to define DATA_PIN.  For led chipsets that are SPI based (four
// wires - data, clock, ground, and power), like the LPD8806 define both
// DATA_PIN and CLOCK_PIN Clock pin only needed for SPI based chipsets when not
// using hardware SPI



#define TAG "main.cpp"


void setup() {
    Serial.begin(9600);
    Serial.setDebugOutput(true);
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    delay(1000);
    ESP_LOGI(TAG, "Start blinking LED strip");
}


void loop() {
    demo(NUM_LEDS, WS2812);
    ESP_LOGE(TAG, "LOOP!");
    delay(500);
}
