/*
 * tsdz_ota_stm8.h
 *
 *  Created on: 10 apr 2020
 *      Author: Max
 */

#ifndef MAIN_TSDZ_OTA_STM8_H_
#define MAIN_TSDZ_OTA_STM8_H_

#include <stdint.h>

extern volatile int stm8_ota_status;

uint8_t ota_stm8_start(uint8_t* data, uint16_t len);
void stm8_ota_task();

#endif /* MAIN_TSDZ_OTA_STM8_H_ */
