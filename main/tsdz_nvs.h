/*
 * tsdz_nvs.h
 *
 *  Created on: 3 set 2019
 *      Author: Max
 */

#ifndef MAIN_TSDZ_NVS_H_
#define MAIN_TSDZ_NVS_H_

void tsdz_nvs_init(void);

void tsdz_nvs_read_cfg(void);
int tsdz_nvs_update_cfg(void);

void tsdz_nvs_update_whOffset(void);
void tsdz_update_esp32_cfg(void);

#endif /* MAIN_TSDZ_NVS_H_ */
