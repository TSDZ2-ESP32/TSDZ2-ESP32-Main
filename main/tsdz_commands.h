/*
 * tsdz_commands.h
 *
 *  Created on: 24 set 2019
 *      Author: Max
 */

#ifndef MAIN_TSDZ_COMMANDS_H_
#define MAIN_TSDZ_COMMANDS_H_

#include <stdint.h>

// Command codes (first byte of the Command Characteristic)
// The response must have as first byte the same Command Code and as second byte the result (0 means OK)
// then follows command specific data

// Command sent by the Android app to request to start the ESP32 OTA process
// Data is a formatted string with ssid,password,port separated by the '|' char. (<ssid>|<password>|<port>)
#define CMD_ESP_OTA                 0x01

// Command sent by the Android app to get app versions.
// response format is <ESP32Version>|STM8FWVersion
#define CMD_GET_APP_VERSION         0x02

// Command sent by the Android app to request to activate at the next boot the STM8 OTA
// Data is a formatted string with ssid,password,port separated by the '|' char. (<ssid>|<password>|<port>)
#define CMD_STM8S_OTA               0x03

#define CMD_ESP_OTA_STATUS          0x05
// Notification for STM8 OTA status progress
// total 3 bytes CMD_STM8_OTA_STATUS, Type, Value
// Type:  0=OTA Executed, 1=OTA Phase, 2= STM8 program progress, 3+ Error
// Value: Type=0: N.A.)
//        Type=1: 1=FW Download completed, 2=RAM routines upload completed
//		  Type=2: write progress in %
//        Type=3: Error code (verify in source code the cause)
#define CMD_STM8_OTA_STATUS         0x06

#define CMD_CADENCE_CALIBRATION     0x07

#define CMD_ESP32_CFG               0x08

#define CALIBRATION_START           0
#define CALIBRATION_STOP            1
#define CALIBRATION_SAVE            2

int exec_command(uint8_t* value, uint16_t len);

#endif /* MAIN_TSDZ_COMMANDS_H_ */
