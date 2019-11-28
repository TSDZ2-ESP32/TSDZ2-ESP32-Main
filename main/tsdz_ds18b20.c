/*
 * tsdz_ds18b20.c
 *
 *  Created on: 28 ott 2019
 *      Author: Max
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"
#include "tsdz_data.h"
#include "tsdz_utils.h"
#include "tsdz_ds18b20.h"

static const char *TAG = "tsdz_ds18b20";

static uint8_t initialized = 0;
static OneWireBus * owb;
static owb_rmt_driver_info rmt_driver_info;
static DS18B20_Info * device = 0;

void tzdz_ds18b20_init(void) {
	// Create a 1-Wire bus, using the RMT timeslot driver
	owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_0);
	owb_use_crc(owb, true);  // enable CRC check for ROM code

	// Create DS18B20 devices on the 1-Wire bus
	device = ds18b20_malloc();
	ds18b20_init_solo(device, owb);          // only one device on bus
	ds18b20_use_crc(device, true);           // enable CRC check for temperature readings
	bool result = ds18b20_set_resolution(device, DS18B20_RESOLUTION_12_BIT);

	if (result && device->init) {
		initialized = 1;
		ESP_LOGI(TAG,"DS18B20 sensor init done");
	} else {
		initialized = -1;
		ESP_LOGW(TAG,"DS18B20 sensor not found");
	}
}

void tzdz_ds18b20_start(void) {
	if (initialized == 0)
		tzdz_ds18b20_init();
	if (initialized == 1)
		ds18b20_convert(device);
}

void tsdz_ds18b20_read(void) {
	if (initialized != 1)
		return;
	float t;
	DS18B20_ERROR error = ds18b20_read_temp(device, &t);
	if (error == DS18B20_OK)
		tsdz_status.ui16_motor_temperaturex10 = filter(t*10, tsdz_status.ui16_motor_temperaturex10, 50);
	else
		ESP_LOGE(TAG,"tsdz_read_temp: %d", error);
}
