/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "settings_nvs.h"
#include "esp_bt.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#include "board.h"
#include "ble_mesh_example_init.h"

#include "ssd1306.h"
#include "font8x8_basic.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_console.h"

#define COMPILE_NODE_ID -1 // Default to -1. All nodes should have their own id in the gradys_config partition

int8_t nodeId = -1;
uint32_t messageSequence = 0;
int sessionReboots = 0;

#define TAG "EXAMPLE"
#define tag "EXAMPLE"

#define CID_ESP     0x02E5

#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT    0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER    0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_SEND      ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS    ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = { 0x32, 0x10 };

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_ENABLED, //ESP_BLE_MESH_RELAY_DISABLED,
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

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
};

static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_SEND, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER,
    vnd_op, NULL, NULL),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

uint16_t my_net_idx = 0;
uint16_t my_addr = 0;
uint16_t my_app_idx = 0;

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx 0x%03x, addr 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags 0x%02x, iv_index 0x%08x", flags, iv_index);
    board_led_operation(LED_G, LED_OFF);

    // group address = 0xC000
    esp_err_t err = esp_ble_mesh_model_subscribe_group_addr(addr, CID_ESP,
                                                  ESP_BLE_MESH_VND_MODEL_ID_SERVER, 0xC000);
    if (err) {
        ESP_LOGE(TAG, "Failed to register to group 0x%s", esp_err_to_name(err));
    }

    my_net_idx = net_idx;
    my_addr = addr;
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
            my_app_idx = param->value.state_change.appkey_add.app_idx;
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
        default:
            break;
        }
    }
}

static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_SEND) {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            char mydata[1024] = "";
            sprintf(mydata, "GRADYS-n%d-r%d-m%u", nodeId, sessionReboots, messageSequence++);
            
            ESP_LOGI(TAG, "Recv 0x%06x, tid 0x%04x", param->model_operation.opcode, tid);
            printf("%s, %s net=0x%04x app=0x%04x\n", __func__, "RECV callback", param->model_operation.ctx->net_idx, param->model_operation.ctx->app_idx);
            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    strlen(mydata)+1, (uint8_t *)mydata); // Sends data towards the Mobile-Hub
            if (err) {
                ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }
        break;
    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        if (param->model_send_comp.err_code) {
            ESP_LOGE(TAG, "Failed to send completion message 0x%06x", param->model_send_comp.opcode);
            break;
        }
        ESP_LOGI(TAG, "Send completion 0x%06x", param->model_send_comp.opcode);
        break;
    default:
        break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

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

    ESP_LOGI(TAG, "BLE Mesh Node initialized");

    return ESP_OK;
}

SSD1306_t dev;

void blinky(void *pvParameter)
{
    //int center, top, bottom;
    //char lineChar[20];

    while(1) {
        printf("Starting task!");
        /*ssd1306_clear_screen(&dev, false);
        ssd1306_contrast(&dev, 0xff);

    #if CONFIG_SSD1306_128x64
        top = 2;
        center = 3;
        bottom = 8;
        ssd1306_display_text(&dev, 0, "SSD1306 128x64", 14, false);
        ssd1306_display_text(&dev, 1, "ABCDEFGHIJKLMNOP", 16, false);
        ssd1306_display_text(&dev, 2, "abcdefghijklmnop",16, false);
        ssd1306_display_text(&dev, 3, "Hello World!!", 13, false);
        ssd1306_clear_line(&dev, 4, true);
        ssd1306_clear_line(&dev, 5, true);
        ssd1306_clear_line(&dev, 6, true);
        ssd1306_clear_line(&dev, 7, true);
        ssd1306_display_text(&dev, 4, "SSD1306 128x64", 14, true);
        ssd1306_display_text(&dev, 5, "ABCDEFGHIJKLMNOP", 16, true);
        ssd1306_display_text(&dev, 6, "abcdefghijklmnop",16, true);
        ssd1306_display_text(&dev, 7, "Hello World!!", 13, true);
    #endif // CONFIG_SSD1306_128x64
        
    #if CONFIG_SSD1306_128x32
        top = 1;
        center = 1;
        bottom = 4;
        ssd1306_display_text(&dev, 0, "SSD1306 128x32", 14, false);
        ssd1306_display_text(&dev, 1, "Hello World!!", 13, false);
        ssd1306_clear_line(&dev, 2, true);
        ssd1306_clear_line(&dev, 3, true);
        ssd1306_display_text(&dev, 2, "SSD1306 128x32", 14, true);
        ssd1306_display_text(&dev, 3, "Hello World!!", 13, true);
    #endif // CONFIG_SSD1306_128x32
        */
       vTaskDelay(30000 / portTICK_PERIOD_MS);
        continue;
        /*
        // Display Count Down
        uint8_t image[24];
        memset(image, 0, sizeof(image));
        ssd1306_display_image(&dev, top, (6*8-1), image, sizeof(image));
        ssd1306_display_image(&dev, top+1, (6*8-1), image, sizeof(image));
        ssd1306_display_image(&dev, top+2, (6*8-1), image, sizeof(image));
        for(int font=0x39;font>0x30;font--) {
            memset(image, 0, sizeof(image));
            ssd1306_display_image(&dev, top+1, (7*8-1), image, 8);
            memcpy(image, font8x8_basic_tr[font], 8);
            if (dev._flip) ssd1306_flip(image, 8);
            ssd1306_display_image(&dev, top+1, (7*8-1), image, 8);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        
        // Scroll Up
        ssd1306_clear_screen(&dev, false);
        ssd1306_contrast(&dev, 0xff);
        ssd1306_display_text(&dev, 0, "---Scroll  UP---", 16, true);
        //ssd1306_software_scroll(&dev, 7, 1);
        ssd1306_software_scroll(&dev, (dev._pages - 1), 1);
        for (int line=0;line<bottom+10;line++) {
            lineChar[0] = 0x01;
            sprintf(&lineChar[1], " Line %02d", line);
            ssd1306_scroll_text(&dev, lineChar, strlen(lineChar), false);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        
        // Scroll Down
        ssd1306_clear_screen(&dev, false);
        ssd1306_contrast(&dev, 0xff);
        ssd1306_display_text(&dev, 0, "--Scroll  DOWN--", 16, true);
        //ssd1306_software_scroll(&dev, 1, 7);
        ssd1306_software_scroll(&dev, 1, (dev._pages - 1) );
        for (int line=0;line<bottom+10;line++) {
            lineChar[0] = 0x02;
            sprintf(&lineChar[1], " Line %02d", line);
            ssd1306_scroll_text(&dev, lineChar, strlen(lineChar), false);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);

        // Page Down
        ssd1306_clear_screen(&dev, false);
        ssd1306_contrast(&dev, 0xff);
        ssd1306_display_text(&dev, 0, "---Page	DOWN---", 16, true);
        ssd1306_software_scroll(&dev, 1, (dev._pages-1) );
        for (int line=0;line<bottom+10;line++) {
            //if ( (line % 7) == 0) ssd1306_scroll_clear(&dev);
            if ( (line % (dev._pages-1)) == 0) ssd1306_scroll_clear(&dev);
            lineChar[0] = 0x02;
            sprintf(&lineChar[1], " Line %02d", line);
            ssd1306_scroll_text(&dev, lineChar, strlen(lineChar), false);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);

        // Horizontal Scroll
        ssd1306_clear_screen(&dev, false);
        ssd1306_contrast(&dev, 0xff);
        ssd1306_display_text(&dev, center, "Horizontal", 10, false);
        ssd1306_hardware_scroll(&dev, SCROLL_RIGHT);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ssd1306_hardware_scroll(&dev, SCROLL_LEFT);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ssd1306_hardware_scroll(&dev, SCROLL_STOP);
        
        // Vertical Scroll
        ssd1306_clear_screen(&dev, false);
        ssd1306_contrast(&dev, 0xff);
        ssd1306_display_text(&dev, center, "Vertical", 8, false);
        ssd1306_hardware_scroll(&dev, SCROLL_DOWN);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ssd1306_hardware_scroll(&dev, SCROLL_UP);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ssd1306_hardware_scroll(&dev, SCROLL_STOP);
        
        // Invert
        ssd1306_clear_screen(&dev, true);
        ssd1306_contrast(&dev, 0xff);
        ssd1306_display_text(&dev, center, "  Good Bye!!", 12, true);
        vTaskDelay(5000 / portTICK_PERIOD_MS);


        // Fade Out
        ssd1306_fadeout(&dev);
        
        vTaskDelay(5000 / portTICK_PERIOD_MS);*/
    }
}

static int restart(int argc, char **argv)
{
    printf("%s, %s", __func__, "Restarting");
    esp_restart();
}

static void register_restart(void)
{
    const esp_console_cmd_t cmd = {
        .command = "restart",
        .help = "Restart the program",
        .hint = NULL,
        .func = &restart,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static int setMAM(int argc, char **argv)
{
    printf("%s, %s\n", __func__, "Setting MAM");
    
    char mydata[1024] = "cmd-GRADYS-setMAM";

    esp_ble_mesh_msg_ctx_t ctx = {0};

    ctx.net_idx = my_net_idx;
    ctx.app_idx = 0;
    ctx.addr = 65275; // Send from Mobile-Hub (this node) to others, reserved address = 65275 //common.ctx.addr = 65275;
    ctx.send_ttl = 3;
    ctx.send_rel = false;
    
    esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
            &ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
            strlen(mydata)+1, (uint8_t *)mydata); // Sends data
    if (err) {
        ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
    }

    return 0;
}

static int setBTMR(int argc, char **argv)
{
    printf("%s, %s\n", __func__, "Setting BTM-R");

    char mydata[1024] = "cmd-GRADYS-setBTMR";

    esp_ble_mesh_msg_ctx_t ctx = {0};

    ctx.net_idx = my_net_idx;
    ctx.app_idx = 0;
    ctx.addr = 65276; // Send from Mobile-Hub (this node) to others, reserved address = 65276 //common.ctx.addr = 65276;
    ctx.send_ttl = 3;
    ctx.send_rel = false;
    
    esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
            &ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
            strlen(mydata)+1, (uint8_t *)mydata); // Sends data
    if (err) {
        ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
    }

    return 0;
}

static int sendDiscovery(int argc, char **argv)
{
    printf("%s, %s\n", __func__, "Sending discovery packet");

    char mydata[1024] = "cmd-GRADYS-discovery";

    esp_ble_mesh_msg_ctx_t ctx = {0};

    ctx.net_idx = my_net_idx;
    ctx.app_idx = 0;
    ctx.addr = 65278; // Send from Mobile-Hub (this node) to others, reserved address = 65278 //common.ctx.addr = 65278;
    ctx.send_ttl = 3;
    ctx.send_rel = false;
    
    esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
            &ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
            strlen(mydata)+1, (uint8_t *)mydata); // Sends data
    if (err) {
        ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
    }

    return 0;
}

static void register_mam_btmr_toggles(void)
{
    const esp_console_cmd_t cmd = {
        .command = "set-mam",
        .help = "Sets MAM as network routing algorithm",
        .hint = NULL,
        .func = &setMAM,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );

    const esp_console_cmd_t cmd2 = {
        .command = "set-btmr",
        .help = "Sets BTM-R as network routing algorithm",
        .hint = NULL,
        .func = &setBTMR,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd2) );
    
    const esp_console_cmd_t cmd3 = {
        .command = "send-discovery",
        .help = "Sends a discovery packet, as if it was a Mobile-Hub",
        .hint = NULL,
        .func = &sendDiscovery,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd3) );
}

static int resetSimulation(int argc, char **argv)
{
    printf("%s, %s net=0x%04x app=0x%04x\n", __func__, "Resetting all simulation nodes", my_net_idx, my_app_idx);

    char mydata[1024] = "cmd-GRADYS-reset";

    esp_ble_mesh_msg_ctx_t ctx = {0};

    ctx.net_idx = my_net_idx;
    ctx.app_idx = 0;
    ctx.addr = 0xC000; // Send from Mobile-Hub (this node) to others, group address = 0xC000
    ctx.send_ttl = 3;
    ctx.send_rel = false;
    
    esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
            &ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
            strlen(mydata)+1, (uint8_t *)mydata); // Sends data to all nodes
    if (err) {
        ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
    }

    return 0;
}

static int statsSimulation(int argc, char **argv)
{
    printf("%s, %s\n", __func__, "Requesting stats from Mobile-Hub");
    printf("%s, %s\n", __func__, "Requesting stats from all nodes");
    
    char mydata[1024] = "cmd-GRADYS-stats";

    esp_ble_mesh_msg_ctx_t ctx = {0};

    ctx.net_idx = my_net_idx;
    ctx.app_idx = 0;
    ctx.addr = 0xC000; // Send from Mobile-Hub (this node) to others, group address = 0xC000
    ctx.send_ttl = 3;
    ctx.send_rel = false;
    
    esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
            &ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
            strlen(mydata)+1, (uint8_t *)mydata); // Sends data to all nodes
    if (err) {
        ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
    }

    return 0;
}

static void register_simulation_cmds(void)
{
    const esp_console_cmd_t cmd = {
        .command = "sim-reset",
        .help = "Resets all simulation nodes (and its statistics)",
        .hint = NULL,
        .func = &resetSimulation,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );

    const esp_console_cmd_t cmd2 = {
        .command = "sim-stats",
        .help = "Requests stats from all nodes (including the Mobile-Hub)",
        .hint = NULL,
        .func = &statsSimulation,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd2) );
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

    ESP_LOGI(TAG, "Initializing gradys_config partition...");
    err = nvs_flash_init_partition("gradys_config");
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "Opening gradys_config partition...");
    bt_mesh_nvs_handle_t gradysconfhandle = 0;
    err = nvs_open_from_partition("gradys_config", "gradys", NVS_READWRITE, &gradysconfhandle);
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "Opening gradys_config id...");
    err = nvs_get_i8(gradysconfhandle, "node-id", &nodeId);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        nodeId = COMPILE_NODE_ID;
        err = nvs_set_i8(gradysconfhandle, "node-id", nodeId);
        ESP_LOGI(TAG, "Wrote gradys_config id %d", nodeId);
    } else {
        ESP_ERROR_CHECK(err);
        ESP_LOGI(TAG, "Loaded gradys_config id %d", nodeId);
    }

    err = nvs_get_i32(gradysconfhandle, "reboots", &sessionReboots);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        sessionReboots = 0;
        err = nvs_set_i32(gradysconfhandle, "reboots", sessionReboots);
        ESP_ERROR_CHECK(err);
        ESP_LOGI(TAG, "Wrote gradys_config reboots %d", sessionReboots);
    } else {
        ESP_ERROR_CHECK(err);
        ESP_LOGI(TAG, "Loaded gradys_config reboots %d", sessionReboots);
        sessionReboots++;
        err = nvs_set_i32(gradysconfhandle, "reboots", sessionReboots);
        ESP_ERROR_CHECK(err);
        ESP_LOGI(TAG, "Wrote gradys_config reboots %d", sessionReboots);
    }

#if CONFIG_I2C_INTERFACE
    ESP_LOGI(tag, "INTERFACE is i2c");
    ESP_LOGI(tag, "CONFIG_SDA_GPIO=%d",CONFIG_SDA_GPIO);
    ESP_LOGI(tag, "CONFIG_SCL_GPIO=%d",CONFIG_SCL_GPIO);
    ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
#endif // CONFIG_I2C_INTERFACE

#if CONFIG_SPI_INTERFACE
    ESP_LOGI(tag, "INTERFACE is SPI");
    ESP_LOGI(tag, "CONFIG_MOSI_GPIO=%d",CONFIG_MOSI_GPIO);
    ESP_LOGI(tag, "CONFIG_SCLK_GPIO=%d",CONFIG_SCLK_GPIO);
    ESP_LOGI(tag, "CONFIG_CS_GPIO=%d",CONFIG_CS_GPIO);
    ESP_LOGI(tag, "CONFIG_DC_GPIO=%d",CONFIG_DC_GPIO);
    ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
    spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO);
#endif // CONFIG_SPI_INTERFACE

#if CONFIG_FLIP
    dev._flip = true;
    ESP_LOGW(tag, "Flip upside down");
#endif

#if CONFIG_SSD1306_128x64
    ESP_LOGI(tag, "Panel is 128x64");
    ssd1306_init(&dev, 128, 64);
#endif // CONFIG_SSD1306_128x64
#if CONFIG_SSD1306_128x32
    ESP_LOGI(tag, "Panel is 128x32");
    ssd1306_init(&dev, 128, 32);
#endif // CONFIG_SSD1306_128x32

    board_init();

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
    
    //xTaskCreatePinnedToCore(&blinky, "blinky", 1024,NULL,5,NULL, APP_CPU_NUM);
    
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();

#if CONFIG_STORE_HISTORY
    initialize_filesystem();
    repl_config.history_save_path = HISTORY_PATH;
#endif
    repl_config.prompt = "esp32>";
    // init console REPL environment
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    /* Register commands */
    register_restart();
    register_mam_btmr_toggles();
    register_simulation_cmds();

    // start console REPL
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
