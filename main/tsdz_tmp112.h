/*
 * tsdz_tmp112.h
 *
 *  Created on: 2 feb 2020
 *      Author: Max
 */

#define TPM112_ADDR		0x49
#define SDA_PIN			11
#define SCL_PIN			12

void tsdz_tmp112_init(void);
void tsdz_tmp112_read(void);
