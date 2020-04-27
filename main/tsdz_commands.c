/*
 * tsdz_commands.c
 *
 *  Created on: 24 set 2019
 *      Author: Max
 */
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_ota_ops.h"

#include "tsdz_commands.h"
#include "tsdz_utils.h"
#include "tsdz_bt.h"
#include "tsdz_nvs.h"
#include "tsdz_data.h"
#include "tsdz_ota_esp32.h"

#define GET                         0
#define SET                         1

static int command_ota(uint8_t* data, uint16_t len, uint8_t cmdType);
static int get_app_version(void);
static int command_cadence_calib(uint8_t* value, uint16_t len);
static int command_esp32_cfg(uint8_t* value, uint16_t len);

int exec_command(uint8_t* value, uint16_t len) {
    switch (value[0]) {
        case CMD_GET_APP_VERSION:
            return get_app_version();
        case CMD_ESP_OTA:
            return command_ota(&value[1], len-1, CMD_ESP_OTA);
        case CMD_STM8S_OTA:
            return command_ota(&value[1], len-1, CMD_STM8S_OTA);
        case CMD_CADENCE_CALIBRATION:
            return command_cadence_calib(&value[1], len-1);
        case CMD_ESP32_CFG:
            return command_esp32_cfg(&value[1], len-1);
    }
    uint8_t ret_val[2] = {value[0], 0xff};
    tsdz_bt_notify_command(ret_val, 2);
    return 1;
}

// Read/Set the ESP32 configuration (DS18B20 data pin, BT update interval, LCD pin mapping)
static int command_esp32_cfg(uint8_t* value, uint16_t len) {
    if (value[0] == SET) {
        uint8_t ret_val[3] = {CMD_ESP32_CFG,SET,0};

        if (value[1] < MIN_BT_UPDTAE_DELAY || value[1] > MAX_BT_UPDATE_DELAY)
            esp32_cfg.bt_update_delay = DEFAULT_BT_UPDATE_DELAY;
        else
            esp32_cfg.bt_update_delay = value[1];

        if (value[2] < MIN_DS18B20_PIN || value[2] > MAX_DS18B20_PIN)
            esp32_cfg.ds18b20_pin = DEFAULT_DS18B20_PIN;
        else
            esp32_cfg.ds18b20_pin = value[2];

        if (value[3] <= 5) {
            esp32_cfg.log_level = value[3];
            setLogLevel();
        }

        tsdz_update_esp32_cfg();
        tsdz_bt_notify_command(ret_val, 3);
        return 0;
    } else if (value[0] == GET) {
        uint8_t ret_val[6] = {CMD_ESP32_CFG,GET,0,0,0,0};
        ret_val[3] = esp32_cfg.bt_update_delay;
        ret_val[4] = esp32_cfg.ds18b20_pin;
        ret_val[5] = esp32_cfg.log_level;
        tsdz_bt_notify_command(ret_val, 6);
        return 0;
    } else {
        uint8_t ret_val[2] = {CMD_ESP32_CFG, 0xff};
        tsdz_bt_notify_command(ret_val, 2);
        return 1;
    }
}

// Control the cadence sensor calibration procedure Start/Stop/Save
static int command_cadence_calib(uint8_t* value, uint16_t len) {
    uint8_t ret_val[3] = {CMD_CADENCE_CALIBRATION,value[0],0};
    uint16_t val;
    switch (value[0]) {
    case CALIBRATION_START:
        ui8_cadence_sensor_calibration = 1;
        break;
    case CALIBRATION_STOP:
        ui8_cadence_sensor_calibration = 0;
        break;
    case CALIBRATION_SAVE:
        val = (((uint16_t) value [2]) << 8) + ((uint16_t) value [1]);
        tsdz_cfg.ui16_cadence_sensor_pulse_high_percentage_x10 = val;
        tsdz_nvs_update_cfg();
        break;
    default:
        ret_val[2] = 0xff;
        break;
    }
    tsdz_bt_notify_command(ret_val, 3);
    return ret_val[2];
}

// Start the OTA update process for ESP32 main app, ESP32 OTA STM8S Loader and STM8S Firmware
static int command_ota(uint8_t* data, uint16_t len, uint8_t cmdType) {
    uint8_t ret_val[2] = {cmdType,0};

    if (cmdType == CMD_ESP_OTA)
        ret_val[1] = ota_esp32_start(data, len);
    else if (cmdType == CMD_STM8S_OTA) {
    	// create the Ota Data null terminated string
    	char* copy = (char*)malloc(len+1);
    	memcpy(copy, data, len);
    	copy[len] = '\0';
    	// Store the data into NVS. At the next boot, the OTA process will be started
    	if (tsdz_nvs_set_ota(copy) == ESP_OK)
    		ret_val[1] = 0;
    	else
    		ret_val[1] = 1;
    	free(copy);
    } else
        ret_val[1] = 0xff;

    tsdz_bt_notify_command(ret_val, 2);

    return ret_val[1];
}

// Get the ESP32 Firmware version (both Main App and STM8S OTA Loader)
static int get_app_version(void) {
    // Confirm partition is valid in case of OTA Update
    ota_confirm_partition();

    char ret[20];
    int len;
    ret[0] = CMD_GET_APP_VERSION;

    // App version
    const esp_app_desc_t* app_desc = esp_ota_get_app_description();
    len = snprintf(&ret[1], 19, "%d|%s", stm8_fw_version, app_desc->version);
    len++;
    if (len >= 20)
    	len = 19;

    tsdz_bt_notify_command((uint8_t*)ret, len);
    return 0;
}

