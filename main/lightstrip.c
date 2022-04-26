/**
 * File: lightstrip.c
 * 
 * Author: Andrew Teta
 * Date Created: 02/27/2021
 * 
 **/

#include <stdio.h>
#include <string.h>
#include <esp_log.h>

#include "driver/gpio.h"
#include "driver/rmt.h"

#include "lightstrip.h"
#include "led_strip.h"

#define DEV_KIT_LED         GPIO_NUM_13
#define DEV_KIT_LED_SEL     GPIO_SEL_13

#define STRIP_PIN           GPIO_NUM_8
#define STRIP_PIN_SEL       GPIO_SEL_8

#define WS2812_T0H_NS (400)
#define WS2812_T0L_NS (850)
#define WS2812_T1H_NS (800)
#define WS2812_T1L_NS (450)

#define RMT_TX_CHANNEL RMT_CHANNEL_0

static const char *TAG = "ws2812";
#define STRIP_CHECK(a, str, goto_tag, ret_value, ...)                             \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

typedef struct {
    led_strip_t parent;
    rmt_channel_t rmt_channel;
    uint32_t strip_len;
    uint8_t buffer[0];
} ws2812_t;

extern const char *TAG;

static uint32_t ws2812_t0h_ticks = 0;
static uint32_t ws2812_t1h_ticks = 0;
static uint32_t ws2812_t0l_ticks = 0;
static uint32_t ws2812_t1l_ticks = 0;

// static rmt_config_t rmt_cfg;
// static led_strip_t *strip;

/**
 * @brief initialize the dev-kit led
 */
int led_init(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = DEV_KIT_LED_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;

    //configure GPIO with the given settings
    gpio_config(&io_conf);

    return 0;
}

/**
 * @brief turn on/off the dev-kit led
 */
int led_set_on(bool value)
{
    gpio_set_level(DEV_KIT_LED, value);

    return 0;
}

/**
 * @brief transform "HSV" to "RGB"
 */
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

/**
 * @brief Convert RGB data to RMT format.
 *
 * @note For WS2812, R,G,B each contains 256 different choices (i.e. uint8_t)
 *
 * @param[in] src: source data, to converted to RMT format
 * @param[in] dest: place where to store the convert result
 * @param[in] src_size: size of source data
 * @param[in] wanted_num: number of RMT items that want to get
 * @param[out] translated_size: number of source data that got converted
 * @param[out] item_num: number of RMT items which are converted from source data
 */
static void IRAM_ATTR ws2812_rmt_adapter(const void *src, rmt_item32_t *dest, size_t src_size,
        size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    if (src == NULL || dest == NULL) {
        *translated_size = 0;
        *item_num = 0;
        return;
    }
    const rmt_item32_t bit0 = {{{ ws2812_t0h_ticks, 1, ws2812_t0l_ticks, 0 }}}; //Logical 0
    const rmt_item32_t bit1 = {{{ ws2812_t1h_ticks, 1, ws2812_t1l_ticks, 0 }}}; //Logical 1
    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    rmt_item32_t *pdest = dest;
    while (size < src_size && num < wanted_num) {
        for (int i = 0; i < 8; i++) {
            // MSB first
            if (*psrc & (1 << (7 - i))) {
                pdest->val =  bit1.val;
            } else {
                pdest->val =  bit0.val;
            }
            num++;
            pdest++;
        }
        size++;
        psrc++;
    }
    *translated_size = size;
    *item_num = num;
}

static esp_err_t ws2812_set_pixel(led_strip_t *strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue)
{
    esp_err_t ret = ESP_OK;
    ws2812_t *ws2812 = __containerof(strip, ws2812_t, parent);
    STRIP_CHECK(index < ws2812->strip_len, "index out of the maximum number of leds", err, ESP_ERR_INVALID_ARG);
    uint32_t start = index * 3;
    // In thr order of GRB
    ws2812->buffer[start + 0] = green & 0xFF;
    ws2812->buffer[start + 1] = red & 0xFF;
    ws2812->buffer[start + 2] = blue & 0xFF;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t ws2812_refresh(led_strip_t *strip, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    ws2812_t *ws2812 = __containerof(strip, ws2812_t, parent);
    STRIP_CHECK(rmt_write_sample(ws2812->rmt_channel, ws2812->buffer, ws2812->strip_len * 3, true) == ESP_OK,
                "transmit RMT samples failed", err, ESP_FAIL);
    return rmt_wait_tx_done(ws2812->rmt_channel, pdMS_TO_TICKS(timeout_ms));
err:
    return ret;
}

static esp_err_t ws2812_clear(led_strip_t *strip, uint32_t timeout_ms)
{
    ws2812_t *ws2812 = __containerof(strip, ws2812_t, parent);
    // Write zero to turn off all leds
    memset(ws2812->buffer, 0, ws2812->strip_len * 3);
    return ws2812_refresh(strip, timeout_ms);
}

static esp_err_t ws2812_del(led_strip_t *strip)
{
    ws2812_t *ws2812 = __containerof(strip, ws2812_t, parent);
    free(ws2812);
    return ESP_OK;
}

/**
 * @brief initialize ws2812b light-strip
 */
int strip_init(void)
{
    // uint32_t counter_clk_hz = 0;

    ESP_LOGI(TAG, "Configuring RMT settings...");
    rmt_config_t rmt_cfg = {
        .channel = RMT_CHANNEL_0,
        .gpio_num = STRIP_PIN,
        .rmt_mode = RMT_MODE_TX,
        .clk_div = 2,
        .mem_block_num = 1,
        .tx_config = {
            .carrier_freq_hz = 38000,
            .carrier_level = RMT_CARRIER_LEVEL_HIGH,
            .idle_level = RMT_IDLE_LEVEL_LOW,
            .carrier_duty_percent = 33,
            .carrier_en = false,
            .loop_en = false,
            .idle_output_en = true,
        }
    };

    // rmt_config(&rmt_cfg);
    // rmt_driver_install(rmt_cfg.channel, 0, 0);

    // // rmt_get_counter_clock(rmt_cfg.channel, &counter_clk_hz);

    // // NS to tick converter
    // float ratio = 40e6 / 1e9;

    // t0h_ticks = (uint32_t)(ratio * WS2812_T0H_NS);
    // t0l_ticks = (uint32_t)(ratio * WS2812_T0L_NS);
    // t1h_ticks = (uint32_t)(ratio * WS2812_T1H_NS);
    // t1l_ticks = (uint32_t)(ratio * WS2812_T1L_NS);

    // // Initialize automatic timing translator
    // rmt_translator_init(rmt_cfg.channel, ws2812_rmt_adapter);  //TODO: might be able to do this only once in init function

    // gpio_set_direction(STRIP_PIN, GPIO_MODE_OUTPUT);

    // rmt_config_t config = RMT_DEFAULT_CONFIG_TX(STRIP_PIN, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    // config.clk_div = 2;

    ESP_LOGI(TAG, "&rmt_cfg=0x%08X", (uint32_t)(&rmt_cfg));
    ESP_LOGI(TAG, "Calling rmt_config(&rmt_cfg)...");
    ESP_ERROR_CHECK(rmt_config(&rmt_cfg));
    ESP_LOGI(TAG, "Calling rmt_driver_install(rmt_cfg.channel, 0, 0)...");
    ESP_ERROR_CHECK(rmt_driver_install(rmt_cfg.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(STRIP_NUM_PIX, (led_strip_dev_t)rmt_cfg.channel);
    ESP_LOGI(TAG, "Registering RMT callabacks and strip object");
    led_strip_t *strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));

    return 0;
}

static void set_pixel_color(uint8_t* pix, uint32_t index, rgb_t* color)
{
    if(index < STRIP_NUM_PIX) {
        uint8_t* p = &pix[index * 3];
        p[1] = color->r;
        p[0] = color->g;
        p[2] = color->b;
    }
}

/**
 * @brief write an array of colors to the light strip
 */
// int strip_update(uint8_t* pixels, uint32_t numBytes, hsb_t* color)
// {
//     if(!pixels || !color)
//         return -1;

//     // Convert HSV color to RGB
//     rgb_t color_r;
//     hsb2rgb(color->h, color->s, color->b, &color_r);

//     // Set color of entire strip
//     for (int i = 0; i < STRIP_NUM_PIX; i++) {
//         set_pixel_color(pixels, i, &color_r);
//     }

//     // Write and wait to finish
//     rmt_write_sample(rmt_cfg.channel, pixels, (size_t)numBytes, true);
//     rmt_wait_tx_done(rmt_cfg.channel, pdMS_TO_TICKS(100));

//     return 0;
// }

led_strip_t *led_strip_new_rmt_ws2812(const led_strip_config_t *config)
{
    led_strip_t *ret = NULL;
    STRIP_CHECK(config, "configuration can't be null", err, NULL);

    // 24 bits per led
    uint32_t ws2812_size = sizeof(ws2812_t) + config->max_leds * 3;
    ws2812_t *ws2812 = calloc(1, ws2812_size);
    STRIP_CHECK(ws2812, "request memory for ws2812 failed", err, NULL);

    uint32_t counter_clk_hz = 40e6;
    // STRIP_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev, &counter_clk_hz) == ESP_OK,
    //             "get rmt counter clock failed", err, NULL);
    // ns -> ticks
    float ratio = (float)counter_clk_hz / 1e9;
    ws2812_t0h_ticks = (uint32_t)(ratio * WS2812_T0H_NS);
    ws2812_t0l_ticks = (uint32_t)(ratio * WS2812_T0L_NS);
    ws2812_t1h_ticks = (uint32_t)(ratio * WS2812_T1H_NS);
    ws2812_t1l_ticks = (uint32_t)(ratio * WS2812_T1L_NS);

    // set ws2812 to rmt adapter
    rmt_translator_init((rmt_channel_t)config->dev, ws2812_rmt_adapter);

    ws2812->rmt_channel = (rmt_channel_t)config->dev;
    ws2812->strip_len = config->max_leds;

    ws2812->parent.set_pixel = ws2812_set_pixel;
    ws2812->parent.refresh = ws2812_refresh;
    ws2812->parent.clear = ws2812_clear;
    ws2812->parent.del = ws2812_del;

    return &ws2812->parent;
err:
    return ret;
}