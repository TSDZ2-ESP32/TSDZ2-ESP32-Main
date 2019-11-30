/*
 * tsdz_ds18b20.h
 *
 *  Created on: 28 ott 2019
 *      Author: SO000228
 */

#ifndef MAIN_TSDZ_DS18B20_H_
#define MAIN_TSDZ_DS18B20_H_

//#define GPIO_DS18B20_0 4  // DS18B20 signal on GPIO4
#define GPIO_DS18B20_0 25  // DS18B20 signal on GPIO4

void tzdz_ds18b20_init(void);
void tzdz_ds18b20_start(void);
void tsdz_ds18b20_read(void);

#endif /* MAIN_TSDZ_DS18B20_H_ */
