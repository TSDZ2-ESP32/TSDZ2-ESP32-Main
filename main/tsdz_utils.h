/*
 * tsdz_utils.h
 *
 *  Created on: 19 set 2019
 *      Author: Max
 */

#ifndef MAIN_TSDZ_UTILS_H_
#define MAIN_TSDZ_UTILS_H_

#include <stdio.h>
#include <stdint.h>

void tsdz_boot_to_factory(void);
uint32_t filter(uint32_t ui32_new_value, uint32_t ui32_old_value, uint8_t ui8_alpha);
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
uint8_t crc8(uint8_t *message, uint8_t count);
void crc16(uint8_t ui8_data, uint16_t* ui16_crc);

#endif /* MAIN_TSDZ_UTILS_H_ */
