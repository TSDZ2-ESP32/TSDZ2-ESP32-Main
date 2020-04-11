/*
 * tsdz_esp_ota.c
 *
 *  Created on: 23 set 2019
 *      Author: Max
 */


#include "tsdz_ota_esp32.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_err.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_http_client.h"

#include "tsdz_bt.h"
#include "tsdz_nvs.h"
#include "tsdz_commands.h"
#include "tsdz_wifi.h"

static const char *TAG = "tsdz_ota";

#define READ_BUFFER_SIZE 32*1024

#define CONN_MAX_RETRY 10

static esp_ota_handle_t update_handle = 0;
static const esp_partition_t *partition = NULL;
extern TaskHandle_t mainTaskHandle;

static char* ssid;
static char* pwd;
static int   port;

static uint8_t disconnect = 0;

static esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    switch(evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        if (!esp_http_client_is_chunked_response(evt->client)) {
            // Write out data
            // printf("%.*s", evt->data_len, (char*)evt->data);
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

// OTA_0 and OTA_1 are the partition for the main app, OTA_2 is the partition of the STM8 Loader App
static void esp_ota_download()
{
    ESP_LOGI(TAG, "esp_ota_download - ssid:%s, pwd:%s, port:%d", ssid, pwd, port);
    wifi_init_sta(ssid, pwd);

    esp_err_t err;
    uint8_t ret[2] = {CMD_ESP_OTA_STATUS, 0};

    char *buffer = malloc(READ_BUFFER_SIZE);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Cannot malloc http receive buffer");
        ret[1] = 1;
        goto exit;
    }

	partition = esp_ota_get_boot_partition();
	switch (partition->subtype) {
	case ESP_PARTITION_SUBTYPE_APP_OTA_0:
		partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);
		if (partition == NULL) {
			ESP_LOGE(TAG, "OTA 1 partition not found!");
			ret[1] = 2;
			goto exit;
		}
		break;
	default:
		partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
		if (partition == NULL) {
			ESP_LOGE(TAG, "OTA 0 partition not found!");
			ret[1] = 2;
			goto exit;
		}
		break;
	}

    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x", partition->subtype, partition->address);

    char url[32];
    snprintf(url, 32, "http://%s:%d", gwAddress, port);

    esp_http_client_config_t config = {
        .url = url,
        .event_handler = _http_event_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    if ((err = esp_http_client_open(client, 0)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        ret[1] = 3;
        goto exit;
    }

    int binary_len = esp_http_client_fetch_headers(client);
    if (binary_len < 0) {
        ESP_LOGE(TAG, "esp_http_client_fetch_headers Error");
        ret[1] = 4;
        goto exit;
    }
    if (binary_len > partition->size) {
        ESP_LOGE(TAG, "Image size too big");
        ret[1] = 4;
        goto exit;
    }
    ESP_LOGI(TAG, "Writing %d bytes to partiton %s", binary_len, partition->label);

    err = esp_ota_begin(partition, binary_len, &update_handle);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Error: esp_ota_begin failed! err=0x%x", err);
        ret[1] = 5;
        goto exit;
    }

    int total_read_len = 0, read_len;
    while (((binary_len > 0) && (total_read_len < binary_len)) ||
            ((binary_len == 0) && !esp_http_client_is_complete_data_received(client))) {
        read_len = esp_http_client_read(client, buffer, READ_BUFFER_SIZE);
        if (read_len < 0) {
            ESP_LOGE(TAG, "Error read data");
            ret[1] = 6;
            esp_ota_end(update_handle);
            goto exit;
        }
        total_read_len += read_len;
        if (total_read_len > partition->size) {
            ESP_LOGE(TAG, "Image size too big");
            ret[1] = 4;
            esp_ota_end(update_handle);
            goto exit;
        }
        err = esp_ota_write(update_handle, (const void *)buffer, read_len);
        if(err != ESP_OK) {
            ESP_LOGE(TAG, "Error: esp_ota_write failed! err=0x%x", err);
            ret[1] = 7;
            esp_ota_end(update_handle);
            goto exit;
        }

    }

    if(esp_ota_end(update_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed!");
        ret[1] = 8;
        goto exit;
    }

	err = esp_ota_set_boot_partition(partition);
	if(err != ESP_OK) {
		ESP_LOGE(TAG, "esp_ota_set_boot_partition failed! err=0x%x", err);
		ret[1] = 9;
		goto exit;
	}

    exit:
    if (buffer != NULL)
        free(buffer);
    ESP_LOGI(TAG, "Disconnecting Wifi...");
    disconnect = 1;
    esp_wifi_disconnect();
    vTaskDelay(300 / portTICK_PERIOD_MS);
    esp_wifi_stop();
    vTaskDelay(300 / portTICK_PERIOD_MS);
    esp_wifi_deinit();
    vTaskDelay(300 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Sending end OTA Status...");
    tsdz_bt_notify_command(ret, 2);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "System restart!");
    esp_restart();
}

uint8_t ota_esp32_start(uint8_t* data, uint16_t len) {
    port = 0;
    char *copy, *r;
    copy = r = (char*)malloc(len+1);
    memcpy(r, data, len);
    r[len] = '\0';
    ssid = strsep(&r, "|");
    pwd  = strsep(&r, "|");
    port = atoi(strsep(&r, "|"));
    ESP_LOGI(TAG, "ota_esp32_start: ssid:%s pwd:%s port:%d", ssid, pwd, port);

    if (ssid == NULL || pwd == NULL || port == 0) {
        ESP_LOGE(TAG, "ota_start - Command parameters Error");
        free(copy);
        return 1;
    }
    vTaskSuspend(mainTaskHandle);
    xTaskCreate(esp_ota_download, "ota_task", 3072, NULL, 5, NULL);
    return 0;
}

// Called form the BT get_app_version command.
// This method confirm the partition as valid after an OTA update.
// N.B. After an OTA update, the Android app must call get_app_version command when the ESP32
// reconnects after the reboot.
// If during the first boot there are no get_app_version requests, the boot partition is
// automatically restored to the previous partition at the next boot.
void ota_confirm_partition() {
    const esp_partition_t* p = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(p, &ota_state) == ESP_OK)
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
            esp_ota_mark_app_valid_cancel_rollback();
}
