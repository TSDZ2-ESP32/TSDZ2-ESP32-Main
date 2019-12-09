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
void tsdz_nvs_update_cfg(void);

void tsdz_nvs_update_whOffset(void);
void tsdz_update_esp32_cfg(void);

esp_err_t tsdz_nvs_write_stm8s_fw(char* data, uint16_t length);
void tsdz_nvs_write_boot_partiton(uint8_t data);

void tsdz_nvs_get_ota_status(uint8_t* status);
void tsdz_nvs_set_ota_status(uint8_t status);

#endif /* MAIN_TSDZ_NVS_H_ */
