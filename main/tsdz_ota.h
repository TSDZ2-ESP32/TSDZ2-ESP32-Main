/*
 * tsdz_esp_ota.h
 *
 *  Created on: 24 set 2019
 *      Author: Max
 */

#ifndef MAIN_TSDZ_OTA_H_
#define MAIN_TSDZ_OTA_H_

#include <stdint.h>

#define ESP32  0
#define STM8   1
#define LOADER 2

uint8_t ota_start(uint8_t* data, uint16_t len, uint8_t what);
void ota_confirm_partition();

#endif /* MAIN_TSDZ_OTA_H_ */
