/*
 * tsdz_wifi.h
 *
 *  Created on: 10 apr 2020
 *      Author: Max
 */

#ifndef MAIN_TSDZ_WIFI_H_
#define MAIN_TSDZ_WIFI_H_

void wifi_set_data(char* p_ssid, char* p_pwd, int p_port);
char* wifi_get_address();
int  wifi_get_port();
bool wifi_start_sta();
void tsdz_wifi_deinit(void);

#endif /* MAIN_TSDZ_WIFI_H_ */
