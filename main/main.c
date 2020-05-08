/*
 * main.c
 *
 *  Created on: 10 set 2019
 *      Author: Max
 */
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_app_format.h"
#include "esp_pm.h"
#include "esp32/pm.h"
#include "esp_bt.h"


#include "tsdz_uart.h"
#include "tsdz_nvs.h"
#include "tsdz_ota_stm8.h"
#include "tsdz_bt.h"
#include "tsdz_data.h"
#include "tsdz_utils.h"
#include "tsdz_ds18b20.h"
#include "tsdz_ota_esp32.h"
#include "tsdz_tmp112.h"


#define MAIN_LOOP_SLEEP_MS 20 // main loop sleep time in ms

static const char *TAG = "tsdz_main";

void mainTask(void * pvParameters);

TaskHandle_t mainTaskHandle = NULL;

void app_main(void)
{

    // Init NVS
	// if OTA_BOOT data is found, start ASAP the STM8 bootloader activation and the OTA process
    char* ota_data = tsdz_nvs_init();
    if (ota_data != NULL)
    	start_ota_stm8(ota_data);

    ESP_LOGI(TAG, "Start normal");


    ESP_LOGI(TAG, "PM cfg ...");
	esp_pm_config_esp32_t pm_config = {
	   .max_freq_mhz = 80,
	   .min_freq_mhz = 80,
	   .light_sleep_enable = true
	};
	esp_err_t ret;
	if((ret = esp_pm_configure(&pm_config)) != ESP_OK) {
		ESP_LOGE(TAG, "esp_pm_configure: %d", ret);
	}


    // wait 1 sec to avoid STM8 bootloader activation
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Read cfg from NVS and set log level according to NVS configured parameter
    ESP_LOGI(TAG, "Read cfg ...");
    tsdz_nvs_read_cfg();
    setLogLevel();

    // initialize the UART
    ESP_LOGI(TAG, "UART init ...");
    tsdz_uart_init();

    tsdz_bt_init();
    ESP_LOGI(TAG, "bt init done");

    tsdz_tmp112_init();
    ESP_LOGI(TAG, "TMP112 init done");

    xTaskCreate( mainTask, "main_task", 2048, NULL, 5, &mainTaskHandle );
    if (mainTaskHandle == NULL) {
        ESP_LOGE(TAG, "main_task Start Task Error\n");
        ESP_LOGE(TAG, "ESP will restart in 3 seconds\n");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }
}

void mainTask(void * pvParameters) {
    // offset the periodic calls to avoid to call all the task on the same cycle

    // send all the notification on the same connection interval to maximize modem sleep time
    static uint8_t tsdz_bt_task_count = 0, cycling_power_bt_task_count = 0;
    static uint8_t data_task_count             = 1;
    static uint8_t motor_temp_task_count       = 2;
    static uint8_t pcb_temp_task_count         = 3;

    ESP_LOGI(TAG, "Main task started");

    TickType_t xLastWakeUpTime = xTaskGetTickCount();

    while (1) {
        // delay MAIN_LOOP_SLEEP_MS ms from previous call
        vTaskDelayUntil(&xLastWakeUpTime, pdMS_TO_TICKS(MAIN_LOOP_SLEEP_MS));

        if ((cycling_power_bt_task_count % 50) == 45) {
            if (esp_bt_controller_is_sleeping())
                ESP_LOGD(TAG, "Modem Sleep ON!");
            else
                ESP_LOGD(TAG, "Modem Sleep OFF!");
        }

        // uart send/receive task
        // run every loop
        tsdz_uart_task();

        // data calculation task (battery Wh consumption)
        // run every 1s
        if (++data_task_count >= (1000 / MAIN_LOOP_SLEEP_MS)) {
            tsdz_data_update();
            data_task_count = 0;
        }

        // read motor temperature every 1 sec
        if (tsdz_cfg.ui8_esp32_temp_control) {
            motor_temp_task_count++;
            if (motor_temp_task_count == (200 / MAIN_LOOP_SLEEP_MS)) {
                // issue the start conversion command
                tzdz_ds18b20_start();
            } else if (motor_temp_task_count == (1000 / MAIN_LOOP_SLEEP_MS)) {
                // max conversion time is 750 ms. The conversion (12 bit) is done and the value could be readed
                tsdz_ds18b20_read();
                motor_temp_task_count = 0;
            }
        }

        // read PCB temperature every 1 sec
        if (++pcb_temp_task_count >= (1000 / MAIN_LOOP_SLEEP_MS)) {
            tsdz_tmp112_read();
            pcb_temp_task_count = 0;
        }

        // TSDZ BT Service notification task
        // esp32_cfg.msg_sec contains the msg/sec configured frequency
        if (++tsdz_bt_task_count >= (1000 / MAIN_LOOP_SLEEP_MS / esp32_cfg.msg_sec)) {
            tsdz_bt_update();
            tsdz_bt_task_count = 0;
        }

        // Cycling Power BT Service notification task (1 notification/sec)
        if (++cycling_power_bt_task_count >= (1000 / MAIN_LOOP_SLEEP_MS)) {
            cycling_bt_update();
            cycling_power_bt_task_count = 0;
        }
    }
}
