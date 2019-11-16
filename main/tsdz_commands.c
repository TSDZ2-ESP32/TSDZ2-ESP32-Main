/*
 * tsdz_commands.c
 *
 *  Created on: 24 set 2019
 *      Author: Max
 */

#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_ota_ops.h"

#include "tsdz_commands.h"
#include "tsdz_bt.h"
#include "tsdz_nvs.h"
#include "tsdz_ota.h"
#include "tsdz_data.h"

static int command_ota(uint8_t* value, uint16_t len, uint8_t cmdType);
static int get_app_version(void);
static int command_stm8_ota_status(void);
static int command_cadence_calib(uint8_t* value, uint16_t len);

int exec_command(uint8_t* value, uint16_t len) {
	switch (value[0]) {
		case CMD_GET_APP_VERSION:
			return get_app_version();
		case CMD_ESP_OTA:
			return command_ota(&value[1], len-1, CMD_ESP_OTA);
		case CMD_LOADER_OTA:
			return command_ota(&value[1], len-1, CMD_LOADER_OTA);
		case CMD_STM8S_OTA:
			return command_ota(&value[1], len-1, CMD_STM8S_OTA);
		case CMD_STM8_OTA_STATUS:
			return command_stm8_ota_status();
		case CMD_CADENCE_CALIBRATION:
			return command_cadence_calib(&value[1], len-1);
	}
	uint8_t ret_val[2] = {value[0], 0xff};
	tsdz_bt_notify_command(ret_val, 2);
	return 1;
}

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


static int command_ota(uint8_t* value, uint16_t len, uint8_t cmdType) {
	uint8_t ret_val[2] = {cmdType,0};

	if (cmdType == CMD_ESP_OTA)
		ret_val[1] = ota_start(value, len, ESP32);
	else if (cmdType == CMD_LOADER_OTA)
		ret_val[1] = ota_start(value, len, LOADER);
	else if (cmdType == CMD_STM8S_OTA)
		ret_val[1] = ota_start(value, len, STM8);
	else
		ret_val[1] = 0xff;

	tsdz_bt_notify_command(ret_val, 2);

	return ret_val[1];
}

static int command_stm8_ota_status(void) {
	uint8_t ret_val[2] = {CMD_STM8_OTA_STATUS,0xff};
	tsdz_nvs_get_ota_status(&ret_val[1]);
	tsdz_bt_notify_command(ret_val, 2);
	return 0;
}

static int get_app_version(void) {
	char ret[66];
	int len;
	ret[0] = CMD_GET_APP_VERSION;

	// App version
	const esp_app_desc_t* app_desc = esp_ota_get_app_description();
	strcpy(&ret[1], app_desc->version);
	len = strlen(app_desc->version) + 1;
	ret[len++] = '|';

	// Loader version
	const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_2, NULL);
	if (partition == NULL) {
		strcpy(&ret[len], "ERR");
		len += strlen("ERR");
	} else {
		esp_app_desc_t loader_desc;
		esp_err_t err = esp_ota_get_partition_description(partition, &loader_desc);
		if (err == ESP_OK) {
			strcpy(&ret[len], loader_desc.version);
			len += strlen(loader_desc.version);
		} else {
			strcpy(&ret[len], "EMP");
			len += strlen("EMP");
		}
	}
	tsdz_bt_notify_command((uint8_t*)ret, len);
	return 0;
}

