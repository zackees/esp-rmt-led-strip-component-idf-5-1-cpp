#ifdef ESP32

#include "enabled.h"

#if FASTLED_RMT5

#include "rmt_strip.h"
#include "esp_log.h"
#include "configure_led.h"
#include "construct.h"
#include "esp_check.h"

#include "rmt_strip_group.h"
#include "fl/warn.h"


#define TAG "rtm_strip.cpp"

#define RMT_ASSERT(x)                  \
    {                                  \
        if (!(x)) {                    \
            ESP_ERROR_CHECK(ESP_FAIL); \
        }                              \
    }

#define RMT_ASSERT_LT(x, y)            \
    {                                  \
        if (!((x) < (y))) {            \
            ESP_ERROR_CHECK(ESP_FAIL); \
        }                              \
    }

#define RMT_ASSERT_MSG(x, msg)         \
    {                                  \
        if (!(x)) {                    \
            ESP_LOGE(TAG, msg);        \
            ESP_ERROR_CHECK(ESP_FAIL); \
        }                              \
    }


IRmtLedStrip* create_rmt_led_strip_no_recycle(
        uint16_t T0H, uint16_t T0L, uint16_t T1H, uint16_t T1L, uint32_t TRESET, // Timing is in nanoseconds
        int pin, uint32_t max_leds, bool is_rgbw){
    return nullptr;
}

IRmtLedStrip* create_rmt_led_strip_deprecated(uint16_t T0H, uint16_t T0L, uint16_t T1H, uint16_t T1L, uint32_t TRESET, int pin, uint32_t max_leds, bool is_rgbw) {
    return nullptr;
}


#endif  // FASTLED_RMT5

#endif  // ESP32
