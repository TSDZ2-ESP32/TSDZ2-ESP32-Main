/*
 * main.c
 *
 *  Created on: 10 set 2019
 *      Author: Max
 */


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "tsdz_uart.h"
#include "tsdz_nvs.h"
#include "tsdz_bt.h"
#include "tsdz_data.h"
#include "tsdz_ota.h"
#include "tsdz_utils.h"
#include "tsdz_ds18b20.h"

#define APP_MAIN "app_main"

void mainTask(void * pvParameters);

TaskHandle_t mainTaskHandle = NULL;

void app_main(void)
{
	tsdz_nvs_init();
	tsdz_nvs_read_cfg();
	ESP_LOGI(APP_MAIN, "cfg read done");

	tsdz_bt_init();
	ESP_LOGI(APP_MAIN, "bt init done");

	// wait to avoid STM8 bootloader activation
	vTaskDelay(pdMS_TO_TICKS(600));

	tsdz_uart_init();
	ESP_LOGI(APP_MAIN, "uart init done");

	//tzdz_ds18b20_init();

	xTaskCreate( mainTask, "main_task", 2048, NULL, 5, &mainTaskHandle );
	if(mainTaskHandle == NULL)
	{
		ESP_LOGE(APP_MAIN, "main_task Start Task Error\n");
		ESP_LOGE(APP_MAIN, "ESP will restart in 3 seconds\n");
		vTaskDelay(pdMS_TO_TICKS(3000));
		esp_restart();
	}
}

void mainTask(void * pvParameters)
{
	const TickType_t delay10ms = pdMS_TO_TICKS(10);

	// offset the periodic calls to avoid to call all the task on the same cycle
	static uint16_t bt_task_count   = 0;
	static uint16_t data_task_count = 1;
	static uint16_t temp_task_count = 2;

	ESP_LOGI(APP_MAIN, "Main task started");

	TickType_t xLastWakeUpTime = xTaskGetTickCount();

	while (1) {
		// delay 10 ms from previous call
		vTaskDelayUntil(&xLastWakeUpTime, delay10ms);
		tsdz_uart_task();

		// run every 250ms (25 * 10ms)
		if (++bt_task_count >= 25) {
			tsdz_bt_update();
			bt_task_count = 0;
		}
		// run every 100ms (10 * 10ms)
		if (++data_task_count >= 10) {
			tsdz_data_update();
			data_task_count = 0;
		}

		// read temperature every 1 sec (100 * 10ms)
		if (tsdz_cfg.ui8_esp32_temp_control) {
			temp_task_count++;
			if (temp_task_count==20) {
				// issue the start conversion command
				tzdz_ds18b20_start();
			} else if (temp_task_count == 100) {
				// after 750 ms the conversion (12 bit) is done and the value could be readed
				tsdz_ds18b20_read();
				temp_task_count = 0;
			}
		}
	}
}
