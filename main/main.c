#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "driver/gpio.h"

#define GATTS_TAG "BLE_LED_DEMO"

// Пины для ESP32-C3
#define LED_SWITCH_PIN GPIO_NUM_2 
#define LED_MMD_PIN    GPIO_NUM_3

    static const char device_name[] = "ESP_COC_CTRL";

// UUID для сервиса и характеристики
#define GATTS_SERVICE_UUID      0x00FF
#define GATTS_CHAR_UUID         0xFF01
#define GATTS_NUM_HANDLE        4

static uint16_t led_handle;

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// GPIO init
void init_gpios(void) {
    // PIN 2
    gpio_reset_pin(LED_SWITCH_PIN); 
    gpio_set_direction(LED_SWITCH_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(LED_SWITCH_PIN, GPIO_FLOATING); // Remove pull-up
    gpio_set_level(LED_SWITCH_PIN, 0);                

    // PIN 3
    gpio_reset_pin(LED_MMD_PIN);
    gpio_set_direction(LED_MMD_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(LED_MMD_PIN, GPIO_FLOATING);    // Remove pull-up
    gpio_set_level(LED_MMD_PIN, 0);                    
    
    ESP_LOGI(GATTS_TAG, "GPIOs 2 and 3 initialized and set to LOW");
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(GATTS_TAG, "Advertising started!");
            }
            break;
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            esp_ble_gap_set_device_name(device_name);
            
            esp_ble_adv_data_t adv_data = {
                .set_scan_rsp = false,
                .include_name = true,
                .include_txpower = true,
                .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
            };
            esp_ble_gap_config_adv_data(&adv_data);

            esp_gatt_srvc_id_t service_id = {
                .is_primary = true,
                .id.inst_id = 0x00,
                .id.uuid.len = ESP_UUID_LEN_16,
                .id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID,
            };
            esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
            break;

        case ESP_GATTS_CREATE_EVT: {
            esp_bt_uuid_t char_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid.uuid16 = GATTS_CHAR_UUID,
            };
            esp_ble_gatts_add_char(param->create.service_handle, &char_uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
                                   NULL, NULL);
            esp_ble_gatts_start_service(param->create.service_handle);
            break;
        }

        case ESP_GATTS_ADD_CHAR_EVT:
            led_handle = param->add_char.attr_handle;
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "Connected!");
            break;

        case ESP_GATTS_WRITE_EVT: {
            if (param->write.len > 0) {
                uint8_t data = param->write.value[0];
                ESP_LOGI(GATTS_TAG, "Received: %d", data);

                // GPIO 2: On/Off
                if (data == '1' || data == 1) {
                    gpio_set_level(LED_SWITCH_PIN, 1);
                    ESP_LOGI(GATTS_TAG, "Switch LED ON");
                } 
                else if (data == '0' || data == 0) {
                    gpio_set_level(LED_SWITCH_PIN, 0);
                    ESP_LOGI(GATTS_TAG, "Switch LED OFF");
                }
                // GPIO 3: Pulse 1 sec
                else if (data == '2' || data == 2) {
                    gpio_set_level(LED_MMD_PIN, 1);
                    ESP_LOGI(GATTS_TAG, "Pulse MMD ON");
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    gpio_set_level(LED_MMD_PIN, 0);
                    ESP_LOGI(GATTS_TAG, "Pulse MMD OFF");
                }
            }
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
        }
       
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "Disconnected, restarting adv...");
            esp_ble_gap_start_advertising(&adv_params);
            break;

        default:
            break;
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_gpios();

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(esp_gap_cb);
    esp_ble_gatts_app_register(0);
}
