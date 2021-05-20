/* board.c - Board-specific hooks */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "esp_log.h"
#include "iot_button.h"
#include "esp_ble_mesh_sensor_model_api.h"

#include "driver/gpio.h"

#include <time.h>

#include "board.h"

#define TAG "BOARD"

#define BUTTON_IO_NUM           0
#define BUTTON_ACTIVE_LEVEL     0

extern void example_ble_mesh_send_sensor_message(uint32_t opcode, int specialProp);

time_t pushTime;

static void button_push_cb(void* arg)
{
    time(&pushTime);
}

static void button_release_cb(void* arg)
{
    time_t releaseTime;
    time(&releaseTime);
    if (releaseTime - pushTime >= 4) {
        // Set MAM or BTM-R relay
        example_ble_mesh_send_sensor_message(ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET, 5);
    } else {
        // Send DISCOVERY packet
        example_ble_mesh_send_sensor_message(ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET, 6);
    }
}

static void board_button_init(void)
{
    button_handle_t btn_handle = iot_button_create(BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_PUSH, button_push_cb, "PUSH");
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, button_release_cb, "RELEASE");
    }
}

struct _led_state led_state[3] = {
    { LED_OFF, LED_OFF, LED_R, "red"   },
    { LED_OFF, LED_OFF, LED_G, "green" },
    { LED_OFF, LED_OFF, LED_B, "blue"  },
};

void board_led_operation(uint8_t pin, uint8_t onoff)
{
    for (int i = 0; i < 3; i++) {
        if (led_state[i].pin != pin) {
            continue;
        }
        if (onoff == led_state[i].previous) {
            ESP_LOGW(TAG, "led %s is already %s",
                     led_state[i].name, (onoff ? "on" : "off"));
            return;
        }
        gpio_set_level(pin, onoff);
        led_state[i].previous = onoff;
        return;
    }

    ESP_LOGE(TAG, "LED is not found!");
}

static void board_led_init(void)
{
    for (int i = 0; i < 3; i++) {
        gpio_reset_pin(led_state[i].pin);
        gpio_set_direction(led_state[i].pin, GPIO_MODE_OUTPUT);
        gpio_set_level(led_state[i].pin, LED_OFF);
        led_state[i].previous = LED_OFF;
    }
}

void board_init(void)
{
    board_button_init();
    board_led_init();
}
