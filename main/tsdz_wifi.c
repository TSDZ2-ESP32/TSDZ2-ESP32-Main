/*
 * tsdz_wifi.c
 *
 *  Created on: 10 apr 2020
 *      Author: Max
 */
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_err.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#define MAXIMUM_RETRY 10

static const char *TAG = "tsdz_wifi";

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about one event
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;

static int s_retry_num = 0;
static uint8_t disconnect = 0;
char gwAddress[16];

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START && !disconnect) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED && !disconnect) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        strncpy(gwAddress, ip4addr_ntoa(&event->ip_info.gw), 16);
        ESP_LOGI(TAG, "got ip - gw addrerss is:%s", gwAddress);
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(char* ssid, char* pwd) {
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "prova",
            .password = "prova3"
        },
    };
    strcpy((char*)wifi_config.sta.ssid, ssid);
    strcpy((char*)wifi_config.sta.password, pwd);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );

    ESP_LOGI(TAG, "Starting wifi ...");
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
            wifi_config.sta.ssid, wifi_config.sta.password);

    ESP_LOGI(TAG, "Wait for WiFi....");

    vTaskDelay(5000/portTICK_PERIOD_MS);
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);

    ESP_LOGI(TAG, "Connected to AP, freemem=%d", esp_get_free_heap_size());
}

void tsdz_wifi_deinit(void) {
	ESP_LOGI(TAG, "WiFi deinit....");
	disconnect = 1;
	esp_wifi_disconnect();
	vTaskDelay(200 / portTICK_PERIOD_MS);
	esp_wifi_stop();
	vTaskDelay(200 / portTICK_PERIOD_MS);
	esp_wifi_deinit();
	vTaskDelay(200 / portTICK_PERIOD_MS);
}
