/// @file    Blink.ino
/// @brief   Blink the first LED of an LED strip
/// @example Blink.ino

#ifdef ESP32
#include "enabled.h"
#include <iostream>

#if FASTLED_ESP32_COMPONENT_LED_STRIP_BUILT_IN

#include "rmt_demo.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "led_strip.h"
#include "enabled.h"
#include "esp_log.h"
#include "esp_err.h"
#include "led_strip_rmt.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "esp_private/esp_clk.h"
#endif

static const char *TAG = "example";



led_strip_config_t make_config(
      int strip_gpio_num,
      uint32_t max_leds,
      led_pixel_format_t led_pixel_format,
      led_model_t led_model,
      uint32_t invert_out) {
    // Note this is the idf 5.1 code with the idf 4.4 code stripped out.
    led_strip_config_t config = {};
    config.strip_gpio_num = strip_gpio_num;
    config.max_leds = max_leds;
    config.led_pixel_format = led_pixel_format;
    config.led_model = led_model;
    config.flags.invert_out = invert_out;
    return config;
}

led_strip_rmt_config_t make_rmt_config(rmt_clock_source_t clk_src, uint32_t resolution_hz, size_t mem_block_symbols, bool with_dma) {
    // assume #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    led_strip_rmt_config_t config = {};
    #if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    config.rmt_channel = rmt_channel;
    #else
    config.clk_src = clk_src;
    config.resolution_hz = resolution_hz;
    #endif
    config.mem_block_symbols = mem_block_symbols;
    config.flags.with_dma = with_dma;
    return config;
}

led_strip_handle_t configure_led(int pin, uint32_t led_numbers, uint32_t rmt_res_hz)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    led_strip_config_t strip_config = make_config(pin, led_numbers, LED_PIXEL_FORMAT_GRB, LED_MODEL_WS2812, 0);
    led_strip_rmt_config_t rmt_config = make_rmt_config(RMT_CLK_SRC_DEFAULT, rmt_res_hz, 0, true);

    // print out the config
    std::cout << "strip_config.strip_gpio_num: " << strip_config.strip_gpio_num << std::endl;
    std::cout << "strip_config.max_leds: " << strip_config.max_leds << std::endl;
    std::cout << "strip_config.led_pixel_format: " << strip_config.led_pixel_format << std::endl;
    std::cout << "strip_config.led_model: " << strip_config.led_model << std::endl;
    std::cout << "strip_config.flags.invert_out: " << strip_config.flags.invert_out << std::endl;

    // print out the config
    std::cout << "rmt_config.clk_src: " << rmt_config.clk_src << std::endl;
    std::cout << "rmt_config.resolution_hz: " << rmt_config.resolution_hz << std::endl;
    std::cout << "rmt_config.mem_block_symbols: " << rmt_config.mem_block_symbols << std::endl;
    std::cout << "rmt_config.flags.with_dma: " << rmt_config.flags.with_dma << std::endl;
    

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
#else
    // For older ESP-IDF versions, use the appropriate LED strip initialization
    led_strip_config_t strip_config = {
        .strip_gpio_num = pin,
        .max_leds = led_numbers,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = rmt_res_hz,
    };
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
#endif
}


void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}


// src\platforms\esp\32\led_strip\led_strip_rmt_encoder.h

#include "led_strip_rmt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "string.h"


#define EXAMPLE_LED_NUMBERS         24
#define EXAMPLE_CHASE_SPEED_MS      100

rmt_tx_channel_config_t make_tx_channel_config(
    rmt_clock_source_t clock_src,
    size_t mem_block_symbols,
    uint32_t resolution_hz,
    int strip_gpio_num,
    size_t trans_queue_depth,
    int intr_priority,
    bool with_dma,
    bool invert_out
)
{
    rmt_tx_channel_config_t out = {};
    memset(&out, 0, sizeof(rmt_tx_channel_config_t));

    //     .clk_src = clock_src,
    //     .gpio_num = strip_gpio_num,
    //     .mem_block_symbols = mem_block_symbols,
    //     .resolution_hz = resolution_hz
    // };
    out.gpio_num = static_cast<gpio_num_t>(strip_gpio_num);
    out.clk_src = clock_src;
    out.resolution_hz = resolution_hz;
    out.mem_block_symbols = mem_block_symbols;
    out.trans_queue_depth = trans_queue_depth;
    out.intr_priority = intr_priority;
    out.flags.with_dma = with_dma;
    out.flags.invert_out = invert_out;

    // print out all values
    std::cout << "out.clk_src: " << out.clk_src << std::endl;
    std::cout << "out.gpio_num: " << out.gpio_num << std::endl;
    std::cout << "out.mem_block_symbols: " << out.mem_block_symbols << std::endl;
    std::cout << "out.resolution_hz: " << out.resolution_hz << std::endl;
    std::cout << "out.flags.with_dma: " << out.flags.with_dma << std::endl;
    std::cout << "out.flags.invert_out: " << out.flags.invert_out << std::endl;
    return out;
}

#undef ESP_ERROR_CHECK

#define ESP_ERROR_CHECK(x) do { \
    esp_err_t __err_rc = (x); \
    if (__err_rc != ESP_OK) { \
        std::cout << "Error " << __err_rc << ": " << esp_err_to_name(__err_rc) << std::endl; \
        return; \
    } \
} while (0)

void rmt_demo(int led_strip_gpio, uint32_t num_leds, uint32_t rmt_res_hz) {
    std::cout << "rmt_demo\n";
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t start_rgb = 0;

    static uint8_t* led_strip_pixels = (uint8_t*)malloc(num_leds * 3);
    memset(led_strip_pixels, 0, num_leds * 3);

    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t led_chan = NULL;
    // rmt_tx_channel_config_t tx_chan_config = {
    //     .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
    //     .gpio_num = led_strip_gpio,
    //     .mem_block_symbols = 64, // increase the block size can make the LED less flickering
    //     .resolution_hz = rmt_res_hz,
    //     .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    // };
    rmt_tx_channel_config_t tx_chan_config = make_tx_channel_config(
        RMT_CLK_SRC_DEFAULT,
        64,
        rmt_res_hz,
        led_strip_gpio,
        4,
        0,
        false,
        false
    );
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));


    // print out the values
    std::cout << "tx_chan_config.clk_src: " << tx_chan_config.clk_src << std::endl;
    std::cout << "tx_chan_config.gpio_num: " << tx_chan_config.gpio_num << std::endl;
    std::cout << "tx_chan_config.mem_block_symbols: " << tx_chan_config.mem_block_symbols << std::endl;
    std::cout << "tx_chan_config.resolution_hz: " << tx_chan_config.resolution_hz << std::endl;
    std::cout << "tx_chan_config.flags.with_dma: " << tx_chan_config.flags.with_dma << std::endl;
    std::cout << "tx_chan_config.flags.invert_out: " << tx_chan_config.flags.invert_out << std::endl;
    std::cout << "tx_chan_config.flags.io_loop_back: " << tx_chan_config.flags.io_loop_back << std::endl;
    std::cout << "tx_chan_config.flags.io_od_mode: " << tx_chan_config.flags.io_od_mode << std::endl;
    std::cout << "tx_chan_config.trans_queue_depth: " << tx_chan_config.trans_queue_depth << std::endl;

    // print out values of led_chan
    //std::cout << "led_chan: " << led_chan << std::endl;

    

    ESP_LOGI(TAG, "Install led strip encoder");
    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = rmt_res_hz,
        .led_model = LED_MODEL_WS2812,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    ESP_LOGI(TAG, "Start LED rainbow chase");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };
    tx_config.flags.queue_nonblocking = 0;
    tx_config.flags.eot_level = 0;
    while (1) {
        //std::cout << "loop\n";
        memset(led_strip_pixels, 0xff, num_leds * 3);
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
        memset(led_strip_pixels, 0, num_leds * 3);
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
        vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));


        // for (int i = 0; i < 3; i++) {
        //     for (int j = i; j < num_leds; j += 3) {
        //         // Build RGB pixels
        //         hue = j * 360 / num_leds + start_rgb;
        //         led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
        //         led_strip_pixels[j * 3 + 0] = green;
        //         led_strip_pixels[j * 3 + 1] = blue;
        //         led_strip_pixels[j * 3 + 2] = red;
        //     }
// 
// 
        //     // Flush RGB values to LEDs
        //     ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        //     ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
// 
        //     // print the led values
        //     for (int i = 0; i < num_leds * 3; i++) {
        //         std::cout << "led_strip_pixels[" << i << "]: " << (int)led_strip_pixels[i] << std::endl;
        //     }
        //     //std::cout << "flush\n";
        //     vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
        //     // memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
        //     //memset(led_strip_pixels, 0, num_leds * 3);
        //     //ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        //     //ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
        //     //vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
        // }
        // start_rgb += 60;
    }
}

#endif  // FASTLED_ESP32_COMPONENT_LED_STRIP_BUILT_IN
#endif  // ESP32
