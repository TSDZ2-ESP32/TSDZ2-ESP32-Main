/*
 * tsdz_nvs.c
 *
 *  Created on: 3 set 2019
 *      Author: Max
 */

#include "esp_partition.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "tsdz_uart.h"
#include "tsdz_data.h"

static const char *TAG = "tsdz_nvs";

static const uint8_t NVS_KEY_VAL = 0x05;

// NVS Configuration Key values
static const char* NVS_KEY = "KEY";
static const char* TSDZ_CFG_KEY ="TSDZ_CFG";
static const char* WH_OFFST_KEY    = "WH_OFFSET";
static const char* ESP32_CFG_KEY = "ESP32_CFG";

// NVS OTA Key values
static const char* STM8_FW = "STM8FW";
static const char* BOOT_PARTITION = "BOOT";
static const char* OTA_RESULT = "OTA_RESULT";


void tsdz_nvs_write_default_cfg(void);

nvs_handle my_handle;

void tsdz_nvs_init(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)  {
        const esp_partition_t* nvs_partition =
                esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
        if(!nvs_partition)
            ESP_LOGE(TAG, "FATAL ERROR: No NVS partition found");
        err = (esp_partition_erase_range(nvs_partition, 0, nvs_partition->size));
        if(err != ESP_OK)
            ESP_LOGE(TAG, "FATAL ERROR: Unable to erase the partition");
    }

    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to open NVS");
}

void tsdz_nvs_read_cfg(void) {
    esp_err_t err;
    uint8_t key;
    err = nvs_get_u8(my_handle, NVS_KEY, &key);
    if((err != ESP_OK) || (NVS_KEY_VAL != key)) {
        tsdz_nvs_write_default_cfg();
    }
    ESP_LOGI(TAG,"NVS KEY = %x", key);

    size_t len = sizeof(tsdz_cfg);
    err = nvs_get_blob(my_handle, TSDZ_CFG_KEY, &tsdz_cfg, &len);
    if ((err != ESP_OK) || (len != sizeof(tsdz_cfg))) {
        ESP_LOGE(TAG, "FATAL ERROR: Unable to read TSDZ Configuration from nvs");
    }
    err = nvs_get_u32(my_handle, WH_OFFST_KEY, &ui32_wh_x10_offset);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FATAL ERROR: Unable to read Wh offset from nvs");
        ui32_wh_x10_offset = 0;
    }
    len = sizeof(esp32_cfg);
    err = nvs_get_blob(my_handle, ESP32_CFG_KEY, &esp32_cfg, &len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FATAL ERROR: Unable to read ESP32 Configuration from nvs: 0x%x", err);
    }
}

void tsdz_nvs_update_cfg(void) {
    esp_err_t err = nvs_set_blob(my_handle, TSDZ_CFG_KEY, &tsdz_cfg, sizeof(tsdz_cfg));
    if(err != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to write Configuration to nvs");
    err = nvs_commit(my_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to commit nvs");
}

void tsdz_nvs_write_default_cfg(void) {
    ESP_LOGI(TAG, "NEW NVS KEY !! - Reset to Default Configuration");
    esp_err_t err = nvs_erase_all(my_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to erase the nvs partition: 0x%x", err);
    err = nvs_set_u8(my_handle, NVS_KEY, NVS_KEY_VAL);
    if(err != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to write nvs key: 0x%x", err);
    err = nvs_set_blob(my_handle, TSDZ_CFG_KEY, &tsdz_cfg, sizeof(tsdz_cfg));
    err |= nvs_set_blob(my_handle, ESP32_CFG_KEY, &esp32_cfg, sizeof(esp32_cfg));
    err |= nvs_set_u32 (my_handle, WH_OFFST_KEY, 0);
    if(err != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to write default configuration: 0x%x", err);
    err = nvs_commit(my_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to commit nvs: %d", err);
}

void tsdz_nvs_update_whOffset(void) {
    esp_err_t err = nvs_set_u32 (my_handle, WH_OFFST_KEY, ui32_wh_x10);
    if(err != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to write uint32 to nvs");
    err = nvs_commit(my_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to commit nvs");
}

void tsdz_update_esp32_cfg() {
    ESP_LOGI(TAG, "tsdz_update_esp32_cfg");
    esp_err_t err = nvs_set_blob(my_handle, ESP32_CFG_KEY, &esp32_cfg, sizeof(esp32_cfg));
    if(err != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to write ESP32 Configuration to nvs");
    err = nvs_commit(my_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to commit nvs");
}

esp_err_t tsdz_nvs_write_stm8s_fw(char* data, uint16_t length) {
    esp_err_t err, err2;
    err = nvs_set_blob(my_handle, STM8_FW, data, length);
    if(err != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to write STM8S firmware");
    err2 = nvs_commit(my_handle);
    if (err2 != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to commit");
    return err||err2;
}

void tsdz_nvs_write_boot_partiton(uint8_t data) {
    esp_err_t err;
    err = nvs_set_u8(my_handle, BOOT_PARTITION, data);
    if(err != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to write boot partition firmware");
    err = nvs_commit(my_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to commit");
}

void tsdz_nvs_get_ota_status(uint8_t* status) {
    esp_err_t err;
    err = nvs_get_u8(my_handle, OTA_RESULT, status);
    if(err != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to read OTA Status");
}

void tsdz_nvs_set_ota_status(uint8_t status) {
    esp_err_t err;
    err = nvs_set_u8(my_handle, OTA_RESULT, status);
    if(err != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to write OTA Statuse");
    err = nvs_commit(my_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "FATAL ERROR: Unable to commit");
}


