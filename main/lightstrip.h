/**
 * File: lightstrip.h
 *
 * Author: Andrew Teta
 * Date Created: 02/27/2021
 *
 **/

#include <stdbool.h>
#include <stdint.h>

#include "driver/rmt.h"

#define STRIP_NUM_PIX 64
#define STRIP_DATA_LEN (STRIP_NUM_PIX * 3)  // for ws2812b rgb strip

typedef struct hsb {
    uint16_t h;  // 0-360
    uint16_t s;  // 0-100
    uint16_t b;  // 0-100
} hsb_t;

typedef struct rgb {
    uint32_t r;  // 0-100 %
    uint32_t g;  // 0-100 %
    uint32_t b;  // 0-100 %
} rgb_t;

/**
 * @brief initialize dev-kit led
 *
 * @param none
 *
 * @return none
 */
int led_init(void);

/**
 * @brief turn on/off the dev-kit led
 *
 * @param value The "On" value
 *
 * @return none
 */
int led_set_on(bool value);

/**
 * @brief initialize ws2812b light-strip
 *
 * @param none
 *
 * @return none
 */
int strip_init(void);

/**
 * @brief update light-strip with data in pixel array
 *
 * @param none
 *
 * @return none
 */
int strip_update(uint8_t* pixels, uint32_t numBytes, hsb_t* color);

void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b);
