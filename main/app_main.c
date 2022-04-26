/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS products only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/* HomeKit Neopixel Controller Project
 */

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_event.h>
#include <esp_log.h>

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>

#include <hap_fw_upgrade.h>
#include <iot_button.h>

#include <app_wifi.h>
#include <app_hap_setup_payload.h>

#include "driver/gpio.h"
#include "driver/rmt.h"

#include "lightstrip.h"
#include "led_strip.h"

#define LED_STRIP_DEFAULT_CONFIG(number, dev_hdl) \
    {                                             \
        .max_leds = number,                       \
        .dev = dev_hdl,                           \
    }

/* Comment out the below line to disable Firmware Upgrades */
// #define CONFIG_FIRMWARE_SERVICE

const char *TAG = "HAP Lightstrip";

#define LS_TASK_PRIORITY 1
#define LS_TASK_STACKSIZE 4 * 1024
#define LS_TASK_NAME "hap_lightstrip"

/* Reset network credentials if button is pressed for more than 3 seconds and then released */
#define RESET_NETWORK_BUTTON_TIMEOUT 3

/* Reset to factory if button is pressed and held for more than 10 seconds */
#define RESET_TO_FACTORY_BUTTON_TIMEOUT 10

/* The button "Boot" will be used as the Reset button for the example */
#define RESET_GPIO GPIO_NUM_0

hsb_t strip_color;
rmt_config_t config = RMT_DEFAULT_CONFIG_TX(33, RMT_CHANNEL_0);
led_strip_t *strip;

/**
 * @brief The network reset button callback handler.
 * Useful for testing the Wi-Fi re-configuration feature of WAC2
 */
static void reset_network_handler(void *arg)
{
    hap_reset_network();
}
/**
 * @brief The factory reset button callback handler.
 */
static void reset_to_factory_handler(void *arg)
{
    hap_reset_to_factory();
}

/**
 * The Reset button  GPIO initialisation function.
 * Same button will be used for resetting Wi-Fi network as well as for reset to factory based on
 * the time for which the button is pressed.
 */
static void reset_key_init(uint32_t key_gpio_pin)
{
    button_handle_t handle = iot_button_create(key_gpio_pin, BUTTON_ACTIVE_LOW);
    iot_button_add_on_release_cb(handle, RESET_NETWORK_BUTTON_TIMEOUT, reset_network_handler, NULL);
    iot_button_add_on_press_cb(handle, RESET_TO_FACTORY_BUTTON_TIMEOUT, reset_to_factory_handler, NULL);
}

/* Mandatory identify routine for the accessory.
 * In a real accessory, something like LED blink should be implemented
 * got visual identification
 */
static int lightstrip_identify(hap_acc_t *ha)
{
    ESP_LOGI(TAG, "Accessory identified");
    return HAP_SUCCESS;
}

/*
 * An optional HomeKit Event handler which can be used to track HomeKit
 * specific events.
 */
static void lightstrip_hap_event_handler(void *arg, esp_event_base_t event_base, int event, void *data)
{
    switch (event)
    {
    case HAP_EVENT_PAIRING_STARTED:
        ESP_LOGI(TAG, "Pairing Started");
        break;
    case HAP_EVENT_PAIRING_ABORTED:
        ESP_LOGI(TAG, "Pairing Aborted");
        break;
    case HAP_EVENT_CTRL_PAIRED:
        ESP_LOGI(TAG, "Controller %s Paired. Controller count: %d",
                 (char *)data, hap_get_paired_controller_count());
        break;
    case HAP_EVENT_CTRL_UNPAIRED:
        ESP_LOGI(TAG, "Controller %s Removed. Controller count: %d",
                 (char *)data, hap_get_paired_controller_count());
        break;
    case HAP_EVENT_CTRL_CONNECTED:
        ESP_LOGI(TAG, "Controller %s Connected", (char *)data);
        break;
    case HAP_EVENT_CTRL_DISCONNECTED:
        ESP_LOGI(TAG, "Controller %s Disconnected", (char *)data);
        break;
    case HAP_EVENT_GET_ACC_COMPLETED:
        ESP_LOGI(TAG, "Accessory Found");
        break;
    case HAP_EVENT_ACC_REBOOTING:
    {
        char *reason = (char *)data;
        ESP_LOGI(TAG, "Accessory Rebooting (Reason: %s)", reason ? reason : "null");
        break;
    }
    default:
        /* Silently ignore unknown events */
        break;
    }
}

/* A dummy callback for handling a read on the "Direction" characteristic of Fan.
 * In an actual accessory, this should read from hardware.
 * Read routines are generally not required as the value is available with th HAP core
 * when it is updated from write routines. For external triggers (like fan switched on/off
 * using physical button), accessories should explicitly call hap_char_update_val()
 * instead of waiting for a read request.
 */
static int lightstrip_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv)
{
    if (hap_req_get_ctrl_id(read_priv))
    {
        ESP_LOGI(TAG, "Received read from %s", hap_req_get_ctrl_id(read_priv));
    }
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_BRIGHTNESS))
    {
        /* Read the current value, toggle it and set the new value.
         * A separate variable should be used for the new value, as the hap_char_get_val()
         * API returns a const pointer
         */
        const hap_val_t *cur_val = hap_char_get_val(hc);

        hap_val_t new_val;
        if (cur_val->i == 1)
        {
            new_val.i = 0;
        }
        else
        {
            new_val.i = 1;
        }
        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
    }
    return HAP_SUCCESS;
}

/* A dummy callback for handling a write on the "On" characteristic of Fan.
 * In an actual accessory, this should control the hardware
 */
static int lightstrip_write(hap_write_data_t write_data[], int count,
                            void *serv_priv, void *write_priv)
{
    int i, ret = HAP_SUCCESS;
    hap_write_data_t *write;

    for (i = 0; i < count; i++)
    {
        write = &write_data[i];
        /* Setting a default error value */
        *(write->status) = HAP_STATUS_VAL_INVALID;
        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_ON))
        {
            rgb_t color_r;

            ESP_LOGI(TAG, "Received Write for Light %s", write->val.b ? "On" : "Off");
            if (write->val.b == 0)
            {
                // ESP_LOGI(TAG, "hue=%d, saturation=%d, brightness=%d",
                //  strip_color.h, strip_color.s, strip_color.b);
                // Set light color to black (off)
                led_strip_hsv2rgb(strip_color.h, strip_color.s, 0,
                                  &(color_r.r), &(color_r.g), &(color_r.b));
                // ESP_LOGI(TAG, "red=%d, green=%d, blue=%d",
                //  color_r.r, color_r.g, color_r.b);
                // Write RGB values to strip driver
                for (int i = 0; i < STRIP_NUM_PIX; i++)
                    ESP_ERROR_CHECK(strip->set_pixel(strip, i, color_r.r, color_r.g, color_r.b));
            }
            else
            {
                // ESP_LOGI(TAG, "hue=%d, saturation=%d, brightness=%d",
                //  strip_color.h, strip_color.s, strip_color.b);
                // Set light back to previously set brightness
                led_strip_hsv2rgb(strip_color.h, strip_color.s, strip_color.b,
                                  &(color_r.r), &(color_r.g), &(color_r.b));
                // ESP_LOGI(TAG, "red=%d, green=%d, blue=%d",
                //  color_r.r, color_r.g, color_r.b);
                // Write RGB values to strip driver
                for (int i = 0; i < STRIP_NUM_PIX; i++)
                    ESP_ERROR_CHECK(strip->set_pixel(strip, i, color_r.r, color_r.g, color_r.b));
            }

            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(strip->refresh(strip, 100));

            int status = 0;
            status = led_set_on(write->val.b);
            if (status == 0)
            {
                *(write->status) = HAP_STATUS_SUCCESS;
            }
        }
        else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_BRIGHTNESS))
        {
            ESP_LOGI(TAG, "Received Write for Light Brightness %d", write->val.i);
            strip_color.b = write->val.i;

            rgb_t color_r;
            // ESP_LOGI(TAG, "hue=%d, saturation=%d, brightness=%d",
            //  strip_color.h, strip_color.s, strip_color.b);
            led_strip_hsv2rgb(strip_color.h, strip_color.s, strip_color.b,
                              &(color_r.r), &(color_r.g), &(color_r.b));
            // ESP_LOGI(TAG, "red=%d, green=%d, blue=%d",
            //  color_r.r, color_r.g, color_r.b);
            // Write RGB values to strip driver
            for (int i = 0; i < STRIP_NUM_PIX; i++)
                ESP_ERROR_CHECK(strip->set_pixel(strip, i, color_r.r, color_r.g, color_r.b));
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(strip->refresh(strip, 100));

            *(write->status) = HAP_STATUS_SUCCESS;
        }
        else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_HUE))
        {
            ESP_LOGI(TAG, "Received Write for Light Hue %f", write->val.f);
            strip_color.h = write->val.f;

            rgb_t color_r;
            // ESP_LOGI(TAG, "hue=%d, saturation=%d, brightness=%d",
            //  strip_color.h, strip_color.s, strip_color.b);
            led_strip_hsv2rgb(strip_color.h, strip_color.s, strip_color.b,
                              &(color_r.r), &(color_r.g), &(color_r.b));
            // ESP_LOGI(TAG, "red=%d, green=%d, blue=%d",
            //  color_r.r, color_r.g, color_r.b);
            // Write RGB values to strip driver
            for (int i = 0; i < STRIP_NUM_PIX; i++)
                ESP_ERROR_CHECK(strip->set_pixel(strip, i, color_r.r, color_r.g, color_r.b));
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(strip->refresh(strip, 100));

            *(write->status) = HAP_STATUS_SUCCESS;
        }
        else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_SATURATION))
        {
            ESP_LOGI(TAG, "Received Write for Light Saturation %f", write->val.f);
            strip_color.s = write->val.f;

            rgb_t color_r;
            // ESP_LOGI(TAG, "hue=%d, saturation=%d, brightness=%d",
            //  strip_color.h, strip_color.s, strip_color.b);
            led_strip_hsv2rgb(strip_color.h, strip_color.s, strip_color.b,
                              &(color_r.r), &(color_r.g), &(color_r.b));
            // ESP_LOGI(TAG, "red=%d, green=%d, blue=%d",
            //  color_r.r, color_r.g, color_r.b);
            // Write RGB values to strip driver
            for (int i = 0; i < STRIP_NUM_PIX; i++)
                ESP_ERROR_CHECK(strip->set_pixel(strip, i, color_r.r, color_r.g, color_r.b));
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(strip->refresh(strip, 100));

            *(write->status) = HAP_STATUS_SUCCESS;
        }
        else
        {
            *(write->status) = HAP_STATUS_RES_ABSENT;
        }
        /* If the characteristic write was successful, update it in hap core
         */
        if (*(write->status) == HAP_STATUS_SUCCESS)
        {
            hap_char_update_val(write->hc, &(write->val));
        }
        else
        {
            /* Else, set the return value appropriately to report error */
            ret = HAP_FAIL;
        }
    }
    return ret;
}

/*The main thread for handling the Lightstrip Accessory */
static void lightstrip_thread_entry(void *p)
{
    hap_acc_t *accessory;
    hap_serv_t *service;

    /* Configure HomeKit core to make the Accessory name (and thus the WAC SSID) unique,
     * instead of the default configuration wherein only the WAC SSID is made unique.
     */
    hap_cfg_t hap_cfg;
    hap_get_config(&hap_cfg);
    hap_cfg.unique_param = UNIQUE_NAME;
    hap_set_config(&hap_cfg);

    /* Initialize the HAP core */
    hap_init(HAP_TRANSPORT_WIFI);

    /* Initialise the mandatory parameters for Accessory which will be added as
     * the mandatory services internally
     */
    hap_acc_cfg_t cfg = {
        .name = "ESP-Lightstrip",
        .manufacturer = "Cosmo1",
        .model = "np_strip",
        .serial_num = "112233445566",
        .fw_rev = "0.9.0",
        .hw_rev = NULL,
        .pv = "1.1",
        .identify_routine = lightstrip_identify,
        .cid = HAP_CID_LIGHTING,
    };
    /* Create accessory object */
    accessory = hap_acc_create(&cfg);
    if (!accessory)
    {
        ESP_LOGE(TAG, "Failed to create accessory");
        goto light_err;
    }

    /* Add a dummy Product Data */
    uint8_t product_data[] = {'E', 'S', 'P', '3', '2', 'H', 'A', 'P'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));

    /* Add Wi-Fi Transport service required for HAP Spec R16 */
    // hap_acc_add_wifi_transport_service(accessory, 0);

    /* Create the Light services. Include the "name" since this is a user visible service  */
    service = hap_serv_lightbulb_create(false);
    if (!service)
    {
        ESP_LOGE(TAG, "Failed to create LightBulb Service");
        goto light_err;
    }

    int ret = hap_serv_add_char(service, hap_char_name_create("My Lightstrip"));
    ret |= hap_serv_add_char(service, hap_char_hue_create(0));
    ret |= hap_serv_add_char(service, hap_char_saturation_create(0));
    ret |= hap_serv_add_char(service, hap_char_brightness_create(0));
    ret |= hap_serv_add_char(service, hap_char_identify_create());

    if (ret != HAP_SUCCESS)
    {
        ESP_LOGE(TAG, "Failed to add optional characteristics to LightBulb");
        goto light_err;
    }

    /* Set the write callback for the service */
    hap_serv_set_write_cb(service, lightstrip_write);

    /* Set the read callback for the service (optional) */
    // hap_serv_set_read_cb(service, lightstrip_read);

    /* Add the lightstrip Service to the Accessory Object */
    hap_acc_add_serv(accessory, service);

#ifdef CONFIG_FIRMWARE_SERVICE
    /*  Required for server verification during OTA, PEM format as string  */
    static char server_cert[] = {};
    hap_fw_upgrade_config_t ota_config = {
        .server_cert_pem = server_cert,
    };
    /* Create and add the Firmware Upgrade Service, if enabled.
     * Please refer the FW Upgrade documentation under components/homekit/extras/include/hap_fw_upgrade.h
     * and the top level README for more information.
     */
    service = hap_serv_fw_upgrade_create(&ota_config);
    if (!service)
    {
        ESP_LOGE(TAG, "Failed to create Firmware Upgrade Service");
        goto light_err;
    }
    hap_acc_add_serv(accessory, service);
#endif

    /* Add the Accessory to the HomeKit Database */
    hap_add_accessory(accessory);

    /* Register a common button for reset Wi-Fi network and reset to factory.
     */
    reset_key_init(RESET_GPIO);

    /* Query the controller count (just for information) */
    ESP_LOGI(TAG, "Accessory is paired with %d controllers",
             hap_get_paired_controller_count());

    /* Hardware initialization */
    led_init();

    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(STRIP_NUM_PIX, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip)
    {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));

    /* For production accessories, the setup code shouldn't be programmed on to
     * the device. Instead, the setup info, derived from the setup code must
     * be used. Use the factory_nvs_gen utility to generate this data and then
     * flash it into the factory NVS partition.
     *
     * By default, the setup ID and setup info will be read from the factory_nvs
     * Flash partition and so, is not required to set here explicitly.
     *
     * However, for testing purpose, this can be overridden by using hap_set_setup_code()
     * and hap_set_setup_id() APIs, as has been done here.
     */
#ifdef CONFIG_EXAMPLE_USE_HARDCODED_SETUP_CODE
    /* Unique Setup code of the format xxx-xx-xxx. Default: 111-22-333 */
    hap_set_setup_code(CONFIG_EXAMPLE_SETUP_CODE);
    /* Unique four character Setup Id. Default: ES32 */
    hap_set_setup_id(CONFIG_EXAMPLE_SETUP_ID);
#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, true, cfg.cid);
#else
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, false, cfg.cid);
#endif
#endif

    /* Enable Hardware MFi authentication (applicable only for MFi variant of SDK) */
    // hap_enable_mfi_auth(HAP_MFI_AUTH_HW);

    /* Initialize Wi-Fi */
    app_wifi_init();

    /* Register an event handler for HomeKit specific events.
     * All event handlers should be registered only after app_wifi_init()
     */
    esp_event_handler_register(HAP_EVENT, ESP_EVENT_ANY_ID, &lightstrip_hap_event_handler, NULL);

    /* After all the initializations are done, start the HAP core */
    // hap_http_debug_enable();
    hap_start();
    /* Start Wi-Fi */
    app_wifi_start(portMAX_DELAY);
    /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
    vTaskDelete(NULL);

light_err:
    // hap_acc_delete(accessory);
    vTaskDelete(NULL);
}

void app_main()
{

    xTaskCreate(lightstrip_thread_entry, LS_TASK_NAME, LS_TASK_STACKSIZE, NULL, LS_TASK_PRIORITY, NULL);
}
