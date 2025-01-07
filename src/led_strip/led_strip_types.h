/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include "driver/rmt_encoder.h"



/**
 * @brief LED strip pixel format
 */
typedef enum {
    // Note - Zach Vorhies - Only this enum means that there is no re-ordering of the colors.
    LED_PIXEL_FORMAT_GRB,    /*!< Pixel format: GRB */
    LED_PIXEL_FORMAT_GRBW,   /*!< Pixel format: GRBW */
    LED_PIXEL_FORMAT_INVALID /*!< Invalid pixel format */
} led_pixel_format_t;

/**
 * @brief LED strip model
 * @note Different led model may have different timing parameters, so we need to distinguish them.
 */
typedef enum {
    LED_MODEL_WS2812, /*!< LED strip model: WS2812 */
    LED_MODEL_SK6812, /*!< LED strip model: SK6812 */
    LED_MODEL_WS2811, /*!< LED strip model: WS2811 */
    LED_MODEL_INVALID /*!< Invalid LED strip model */
} led_model_t;

/**
 * @brief LED strip handle
 */
typedef struct led_strip_t *led_strip_handle_t;


typedef union {
    struct format_layout {
        uint32_t r_pos: 2;          /*!< Position of the red channel in the color order: 0~3 */
        uint32_t g_pos: 2;          /*!< Position of the green channel in the color order: 0~3 */
        uint32_t b_pos: 2;          /*!< Position of the blue channel in the color order: 0~3 */
        uint32_t w_pos: 2;          /*!< Position of the white channel in the color order: 0~3 */
        uint32_t reserved: 21;      /*!< Reserved */
        uint32_t num_components: 3; /*!< Number of color components per pixel: 3 or 4. If set to 0, it will fallback to 3 */
    } format;                       /*!< Format layout */
    uint32_t format_id;             /*!< Format ID */
} led_color_component_format_t;
#define LED_STRIP_COLOR_COMPONENT_FMT_GRB (led_color_component_format_t){.format = {.r_pos = 1, .g_pos = 0, .b_pos = 2, .w_pos = 3, .reserved = 0, .num_components = 3}}
#define LED_STRIP_COLOR_COMPONENT_FMT_GRBW (led_color_component_format_t){.format = {.r_pos = 1, .g_pos = 0, .b_pos = 2, .w_pos = 3, .reserved = 0, .num_components = 4}}
#define LED_STRIP_COLOR_COMPONENT_FMT_RGB (led_color_component_format_t){.format = {.r_pos = 0, .g_pos = 1, .b_pos = 2, .w_pos = 3, .reserved = 0, .num_components = 3}}
#define LED_STRIP_COLOR_COMPONENT_FMT_RGBW (led_color_component_format_t){.format = {.r_pos = 0, .g_pos = 1, .b_pos = 2, .w_pos = 3, .reserved = 0, .num_components = 4}}
/**
 * @brief LED Strip Configuration
 */
typedef struct {
    gpio_num_t strip_gpio_num;      /*!< GPIO number that used by LED strip */
    uint32_t max_leds;       /*!< Maximum LEDs in a single strip */
    //led_pixel_format_t led_pixel_format; /*!< LED pixel format */
    // led_model_t led_model;   /*!< LED model */
    rmt_bytes_encoder_config_t rmt_bytes_encoder_config; /*!< RMT bytes encoder configuration */
                                                              // Use helper macros like `LED_STRIP_COLOR_COMPONENT_FMT_GRB` to set the format */
    rmt_symbol_word_t reset_code; /*!< Reset code for LED strip */
    struct led_strip_extra_flags {
        uint32_t invert_out: 1; /*!< Invert output signal */
        uint32_t rgbw: 1;       /*!< RGBW mode */
    } flags;                    /*!< Extra driver flags */


} led_strip_config_t;