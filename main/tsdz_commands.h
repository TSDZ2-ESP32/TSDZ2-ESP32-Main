/*
 * tsdz_commands.h
 *
 *  Created on: 24 set 2019
 *      Author: Max
 */

#ifndef MAIN_TSDZ_COMMANDS_H_
#define MAIN_TSDZ_COMMANDS_H_

#include <stdint.h>

#define CMD_ESP_OTA 				0x01
#define CMD_GET_APP_VERSION 		0x02
#define CMD_STM8S_OTA				0x03
#define CMD_LOADER_OTA				0x04
#define CMD_ESP_OTA_STATUS			0x05
#define CMD_STM8_OTA_STATUS			0x06
#define CMD_CADENCE_CALIBRATION		0x07
#define CMD_ESP32_CFG				0x08

#define CALIBRATION_START			0
#define CALIBRATION_STOP			1
#define CALIBRATION_SAVE			2

#define GET							0
#define SET							1

int exec_command(uint8_t* value, uint16_t len);

#endif /* MAIN_TSDZ_COMMANDS_H_ */
