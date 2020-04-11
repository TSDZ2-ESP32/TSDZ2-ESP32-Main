/*
 * tsdz_esp_ota.h
 *
 *  Created on: 24 set 2019
 *      Author: Max
 */

#ifndef MAIN_TSDZ_OTA_ESP32_H_
#define MAIN_TSDZ_OTA_ESP32_H_

#include <stdint.h>

uint8_t ota_esp32_start(uint8_t* data, uint16_t len);
void ota_confirm_partition();

#endif /* MAIN_TSDZ_OTA_ESP32_H_ */
