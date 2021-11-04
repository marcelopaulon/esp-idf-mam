/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"

#include "ble_mesh_example_init.h"
#include "board.h"

#define TAG "EXAMPLE"

#define CID_ESP     0x02E5

bool receivedMhub = false;
bool *receivedMhubPtr = &receivedMhub;

bool initialized = false;

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = { 0x32, 0x10 };

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

static esp_ble_mesh_generic_property_t properties[1] = {
    [0] = {
        .id = 1,
        .user_access = 1,
        .admin_access = 1,
        .manu_access = 1,
    }
};

static esp_ble_mesh_gen_user_prop_srv_t prop_server = {
    .property_count = ARRAY_SIZE(properties),
    .properties = properties,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(prop_pub, 20, ROLE_NODE); // TODO how to deal with variable message length?

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_USER_PROP_SRV(&prop_pub, &prop_server),
    //ESP_BLE_MESH_MODEL_SENSOR_SETUP_SRV(&sensor_setup_pub, &sensor_setup_server), // TODO remove me
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

uint16_t net_idx;
uint16_t addr;
uint32_t iv_index;

bool provisioned = false;

static void prov_complete(uint16_t net_idx_t, uint16_t addr_t, uint8_t flags, uint32_t iv_index_t)
{
    net_idx = net_idx_t;
    addr = addr_t;
    iv_index = iv_index_t;
    provisioned = true;
    ESP_LOGI(TAG, "net_idx 0x%03x, addr 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags 0x%02x, iv_index 0x%08x", flags, iv_index);
    board_led_operation(LED_G, LED_OFF);
    initialized = true;
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD");
            ESP_LOGI(TAG, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_sub_add.element_addr,
                param->value.state_change.mod_sub_add.sub_addr,
                param->value.state_change.mod_sub_add.company_id,
                param->value.state_change.mod_sub_add.model_id);
            break;
        default:
            break;
        }
    }
}

static void example_ble_mesh_sensor_server_cb(esp_ble_mesh_generic_server_cb_event_t event,
                                              esp_ble_mesh_generic_server_cb_param_t *param)
{
    ESP_LOGI(TAG, "Sensor server, event %d, src 0x%04x, dst 0x%04x, model_id 0x%04x",
        event, param->ctx.addr, param->ctx.recv_dst, param->model->model_id);

    if (param->ctx.recv_dst == 65278) {
        // DISCOVERY MESSAGE
        printf("[Mobile-Hub] DISCOVERY MESSAGE RECEIVED");
    }
    else if (event == ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET) {
        printf("[Mobile-Hub] DATA RECEIVED");
    }
}

void example_ble_mesh_send_sensor_message(uint32_t opcode, int idx)
{
    /*
        static void example_ble_mesh_set_msg_common(esp_ble_mesh_client_common_param_t *common,
                                            esp_ble_mesh_node_t *node,
                                            esp_ble_mesh_model_t *model, uint32_t opcode)
        {
            
    */


    esp_ble_mesh_gen_user_property_set_t data = {0};
    esp_ble_mesh_client_common_param_t common = {0};
    esp_err_t err = ESP_OK;

    common.opcode = opcode;
    //common.model = sensor_client;
    common.ctx.net_idx = net_idx;
    //common.ctx.app_idx = 1;
    common.ctx.send_ttl = 120;
    common.ctx.send_rel = true;
    //common.msg_timeout = 0;
    //common.msg_role = 1;

    //[5] = ESP_BLE_MESH_MODEL_OP_GEN_ADMIN_PROPERTY_SET, // Set MAM or BTM-R relay
    //[6] = ESP_BLE_MESH_MODEL_OP_GEN_ADMIN_PROPERTY_STATUS // Send DISCOVERY packet
    if (idx == 5) {
        ESP_LOGW(TAG, "[Mobile-Hub] Sending Set MAM/BTM-R relay message");
        common.opcode = opcode;
        common.ctx.send_ttl = 120;
        common.ctx.addr = 65279; // 0XFF00 - 1
    }
    else if (idx == 6) {
        ESP_LOGW(TAG, "[Mobile-Hub] Sending DISCOVERY message");
        common.opcode = opcode;
        common.ctx.send_ttl = 120;
        common.ctx.addr = 65278; // 0XFF00 - 2
    }
    else {
        ESP_LOGW(TAG, "[Mobile-Hub] ERROR WHEN SENDING MESSAGE");
        return;
    }

    uint16_t length = 1;
    uint8_t status[1];
    status[0] = 3;

    err = esp_ble_mesh_server_model_send_msg(prop_server.model, &common.ctx,
            ESP_BLE_MESH_MODEL_OP_GEN_USER_PROPERTY_SET_UNACK, length, status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Sensor Descriptor Status");
    }

    ESP_LOGW(TAG, "Sending message with opcode 0x%04x", opcode);
}

void discoverySender(void *pvParameter)
{
 
    while(1) {
        if (!initialized) {
            vTaskDelay(100 / portTICK_RATE_MS);
            continue;
        }

        // Send DISCOVERY packet
        example_ble_mesh_send_sensor_message(ESP_BLE_MESH_MODEL_OP_GEN_USER_PROPERTY_SET_UNACK, 6);
        vTaskDelay(3000 / portTICK_RATE_MS);
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    //esp_ble_mesh_register_sensor_server_callback(example_ble_mesh_sensor_server_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack");
        return err;
    }

    err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh node");
        return err;
    }

    board_led_operation(LED_G, LED_ON);

    ESP_LOGI(TAG, "BLE Mesh sensor server initialized");

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    board_init();

    xTaskCreate(&discoverySender, "discoverySender", 2048,NULL,5,NULL );

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
}
