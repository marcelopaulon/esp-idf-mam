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

#define TAG "BOARD"

#define BUTTON_IO_NUM           0
#define BUTTON_ACTIVE_LEVEL     0

extern void example_ble_mesh_send_sensor_message(uint32_t opcode, int specialProp);

static uint32_t send_opcode[] = {
    [0] = ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET,
    [1] = ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET,
    [2] = ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET,
    [3] = ESP_BLE_MESH_MODEL_OP_SENSOR_GET,
    [4] = ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET,
    [5] = ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_GET, // Set MAM or BTM-R relay
    [6] = ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET // Send DISCOVERY packet
};
static uint8_t press_count;

static void button_tap_cb(void* arg)
{
    int idx = press_count++;

    example_ble_mesh_send_sensor_message(send_opcode[idx], 0);
    press_count = press_count % ARRAY_SIZE(send_opcode);
}

static void board_button_init(void)
{
    button_handle_t btn_handle = iot_button_create(BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, button_tap_cb, "RELEASE");
    }
}

void board_init(void)
{
    board_button_init();
}
