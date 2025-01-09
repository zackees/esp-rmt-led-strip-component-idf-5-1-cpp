
#include <Arduino.h>

#include "strip_rmt.h"
#include "strip_spi.h"


// refactor the spi test into a class
class SpiTest {
public:
    SpiTest() {
        mLedStrip = ISpiStripWs2812::Create(1, LED_STRIP_LED_COUNT);
    }
    ~SpiTest() {
        delete mLedStrip;
    }

    void loop() {
        if (mLedOn) {
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
            mLedStrip->fill(5,5,5);
            /* Refresh the strip to send data */
            ESP_LOGI("SpiTest", "LED ON!");
        } else {
            /* Set all LED off to clear all pixels */
            mLedStrip->fill(0,0,0);
            ESP_LOGI("SpiTest", "LED OFF!");
        }
        mLedStrip->drawSync();

        mLedOn = !mLedOn;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

private:
    ISpiStripWs2812* mLedStrip;
    bool mLedOn = false;
};

// refactor the rmt test into a class
class RmtTest {
public:
    RmtTest() {
        // ws2812 timings
        uint32_t th0 = 300;   // ns
        uint32_t tl0 = 900;   // ns
        uint32_t th1 = 900;   // ns
        uint32_t tl1 = 300;   // ns
        uint32_t reset = 280; // us

        mLedStrip1 = IRmtStrip::Create(6, LED_STRIP_LED_COUNT, false, th0, tl0, th1, tl1, reset);
        mLedStrip2 = IRmtStrip::Create(7, LED_STRIP_LED_COUNT, false, th0, tl0, th1, tl1, reset);
        mLedStrip3 = IRmtStrip::Create(8, LED_STRIP_LED_COUNT, false, th0, tl0, th1, tl1, reset);
        mLedStrip4 = IRmtStrip::Create(9, LED_STRIP_LED_COUNT, false, th0, tl0, th1, tl1, reset);
    }
    ~RmtTest() {
        delete mLedStrip1;
        delete mLedStrip2;
        delete mLedStrip3;
        delete mLedStrip4;
    }

    void loop() {
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        if (mLedOn) {
            r = 5;
            g = 5;
            b = 5;
        }
        mLedStrip1->fill(r, g, b);
        mLedStrip2->fill(r, g, b);
        mLedStrip3->fill(r, g, b);
        mLedStrip4->fill(r, g, b);
        mLedStrip1->drawAsync();
        mLedStrip2->drawAsync();
        mLedStrip3->drawAsync();
        mLedStrip4->drawAsync();

        mLedOn = !mLedOn;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    private:
    IRmtStrip* mLedStrip1;
    IRmtStrip* mLedStrip2;
    IRmtStrip* mLedStrip3;
    IRmtStrip* mLedStrip4;
    bool mLedOn = false;
};


void setup() {
    delay(3000);
}

void loop() {
    static SpiTest spiTest;
    static RmtTest rmtTest;
    spiTest.loop();
    rmtTest.loop();
}
