
#include <FastLED.h>
#include "platforms/esp/32/led_strip/rmt_demo.h"
#include <iostream>



// How many leds in your strip?
#define NUM_LEDS 10

// For led chips like WS2812, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
// Clock pin only needed for SPI based chipsets when not using hardware SPI
#define DATA_PIN 9

// Define the array of leds
CRGB leds[NUM_LEDS];


// Time scaling factors for each component
#define TIME_FACTOR_HUE 60
#define TIME_FACTOR_SAT 100
#define TIME_FACTOR_VAL 100


void setup() {
    Serial.begin(115200);
    delay(3000);
    std::cout << "Start blinking LED strip" << std::endl;

}

void loop() {
    static int count = 0;
    count++;
    // rmt_demo(DATA_PIN, NUM_LEDS);
    std::cout << "HELLO!: " << count << std::endl;

    if (count > 10) {
        rmt_demo(DATA_PIN, NUM_LEDS);
    }
    delay(1000);
}
