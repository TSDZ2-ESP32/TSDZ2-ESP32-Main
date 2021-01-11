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
// total 3 bytes CMD_STM8_OTA_STATUS, Status, Value
// Status:  0=OTA Completed,
//          1=FW Download completed,
//          2=RAM routines upload completed,
//          3=Program progress % (value contains % progress)]
//          4=Error (value contains error code)
#define CMD_STM8_OTA_STATUS         0x06
// Notification sent during Motor Calibration (contains the HALL reference counters data)
#define CMD_HALL_DATA               0x07
// Command sent by the Android app to set the ESP32 Config parameters
#define CMD_ESP32_CFG               0x08
// Command sent by the Android app to override the Street Mode
#define CMD_STREET_MODE             0x09
// Command sent by the Android app to override the Assist Mode
#define CMD_ASSIST_MODE             0x0A
// Command sent by the Android app to Start/Stop Motor Test/Calibration
#define CMD_MOTOR_CALIBRATION       0x0B

#define TEST_START                  1
#define TEST_STOP                   0

int exec_command(uint8_t* value, uint16_t len);

#endif /* MAIN_TSDZ_COMMANDS_H_ */
