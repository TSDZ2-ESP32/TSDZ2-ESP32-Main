/*
 * tsdz_wifi.h
 *
 *  Created on: 10 apr 2020
 *      Author: Max
 */

#ifndef MAIN_TSDZ_WIFI_H_
#define MAIN_TSDZ_WIFI_H_

char gwAddress[16];

void wifi_init_sta(char* ssid, char* pwd);
void tsdz_wifi_deinit(void);

#endif /* MAIN_TSDZ_WIFI_H_ */
