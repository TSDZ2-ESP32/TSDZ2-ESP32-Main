/*
 * tsdz_esp_ota.c
 *
 *  Created on: 23 set 2019
 *      Author: Max
 */


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

#include "tsdz_ota.h"

#include "tsdz_bt.h"
#include "tsdz_nvs.h"
#include "tsdz_commands.h"

static const char *TAG = "tsdz_esp_ota";

#define READ_BUFFER_SIZE 32*1024

#define EXAMPLE_ESP_MAXIMUM_RETRY 10

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about one event
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;

static int s_retry_num = 0;
static esp_ota_handle_t update_handle = 0;
static const esp_partition_t *partition = NULL;
extern TaskHandle_t mainTaskHandle;

static char *ssid;
static char *pwd;
static char *url;
static uint8_t updateType;
static uint8_t disconnect = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START && !disconnect) {
		esp_wifi_connect();
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED && !disconnect) {
		if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
			esp_wifi_connect();
			xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
			s_retry_num++;
			ESP_LOGI(TAG, "retry to connect to the AP");
		}
		ESP_LOGI(TAG,"connect to the AP fail");
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI(TAG, "got ip:%s",
				ip4addr_ntoa(&event->ip_info.ip));
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
}

void wifi_init_sta(char* ssid, char* pwd)
{
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
	ESP_ERROR_CHECK(esp_wifi_start() );

	ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
			wifi_config.sta.ssid, wifi_config.sta.password);

	ESP_LOGI(TAG, "Wait for WiFi....");

	vTaskDelay(5000/portTICK_PERIOD_MS);
	xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);

	ESP_LOGI(TAG, "Connected to AP, freemem=%d", esp_get_free_heap_size());
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
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
static void esp_ota_download(const char* url)
{
	esp_err_t err;
	uint8_t ret[2] = {CMD_ESP_OTA_STATUS, 0};

	char *buffer = malloc(READ_BUFFER_SIZE);
	if (buffer == NULL) {
		ESP_LOGE(TAG, "Cannot malloc http receive buffer");
		ret[1] = 1;
		goto exit;
	}

	if (updateType == ESP32) {
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
	} else {
		partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_2, NULL);
		if (partition == NULL) {
			ESP_LOGE(TAG, "OTA 3 partition not found!");
			ret[1] = 2;
			goto exit;
		}
	}

	ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
			partition->subtype, partition->address);

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
			goto exit;
		}
		err = esp_ota_write(update_handle, (const void *)buffer, read_len);
		if(err != ESP_OK) {
			ESP_LOGE(TAG, "Error: esp_ota_write failed! err=0x%x", err);
			ret[1] = 7;
			goto exit;
		}
		total_read_len += read_len;
	}

	if(esp_ota_end(update_handle) != ESP_OK) {
		ESP_LOGE(TAG, "esp_ota_end failed!");
		ret[1] = 8;
		goto exit;
	}

	if (updateType == ESP32) {
		err = esp_ota_set_boot_partition(partition);
		if(err != ESP_OK) {
			ESP_LOGE(TAG, "esp_ota_set_boot_partition failed! err=0x%x", err);
			ret[1] = 9;
			goto exit;
		}
	}

	exit:

	ESP_LOGI(TAG, "Disconnecting Wifi...");
	disconnect = 1,
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

static void stm8_ota_download(const char* url)
{
	esp_err_t err;
	uint8_t ret[2] = {CMD_STM8_OTA_STATUS, 0};

	char *buffer = malloc(READ_BUFFER_SIZE);
	if (buffer == NULL) {
		ESP_LOGE(TAG, "Cannot malloc http receive buffer");
		ret[1] = 1;
		goto exit;
	}

	esp_http_client_config_t config = {
			.url = url,
			.event_handler = _http_event_handler,
	};
	esp_http_client_handle_t client = esp_http_client_init(&config);

	if ((err = esp_http_client_open(client, 0)) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
		ret[1] = 2;
		goto exit;
	}

	int binary_len = esp_http_client_fetch_headers(client);
	if (binary_len > 1024*32) {
		ESP_LOGE(TAG, "STM8 FW size wrong: %d bytes", binary_len);
		ret[1] = 3;
		goto exit;
	}
	ESP_LOGI(TAG, "Reading FW data: %d bytes", binary_len);

	int total_read_len = 0, read_len;
	while (((binary_len > 0) && (total_read_len < binary_len)) ||
			((binary_len == 0) && !esp_http_client_is_complete_data_received(client))) {
		read_len = esp_http_client_read(client, &buffer[total_read_len], READ_BUFFER_SIZE-total_read_len);
		if (read_len < 0) {
			ESP_LOGE(TAG, "Error read data");
			ret[1] = 4;
			goto exit;
		}
		total_read_len += read_len;
	}
	if (total_read_len > 1024*32) {
		ESP_LOGE(TAG, "STM8 FW size wrong: %d bytes", total_read_len);
		ret[1] = 5;
		goto exit;
	}

	err = tsdz_nvs_write_stm8s_fw(buffer, total_read_len);
	if(err != ESP_OK) {
		ret[1] = 6;
		goto exit;
	}
	ESP_LOGI(TAG, "FW data written to NVS");

	// Save to NVS current boot partition
	partition = esp_ota_get_boot_partition();
	tsdz_nvs_write_boot_partiton(partition->subtype);

	partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_2, NULL);
	err = esp_ota_set_boot_partition(partition);
	if(err != ESP_OK) {
		ESP_LOGE(TAG, "esp_ota_set_boot_partition failed! err=0x%x", err);
		ret[1] = 7;
		goto exit;
	}

	exit:
	ESP_LOGI(TAG, "Disconnecting Wifi...");
	disconnect = 1,
	esp_wifi_disconnect();
	vTaskDelay(300 / portTICK_PERIOD_MS);
	esp_wifi_stop();
	vTaskDelay(300 / portTICK_PERIOD_MS);
	esp_wifi_deinit();
	vTaskDelay(300 / portTICK_PERIOD_MS);

	// Initialize STM8 OTA Status to error on phase 2
	// will be overwritten by the OTA app in case of success
	tsdz_nvs_set_ota_status(0x81);

	ESP_LOGI(TAG, "Sending end OTA Status...");
	tsdz_bt_notify_command(ret, 2);

	ESP_LOGI(TAG, "Prepare to restart system!");
	while (1) {
		ESP_LOGI(TAG, "Waiting Power Off...");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

uint8_t checkLoaderPartition() {
	esp_err_t err;
	esp_app_desc_t app_desc;

	partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_2, NULL);
	if (partition == NULL) {
		ESP_LOGE(TAG, "OTA 2 partition not found!");
		return 0;
	}
	err = esp_ota_get_partition_description(partition, &app_desc);
	return (err == ESP_OK);
}

void ota_task(void * pvParameters) {
	ESP_LOGI(TAG, "ota_task - ssid:%s, pwd:%s, url:%s", ssid, pwd, url);
	wifi_init_sta(ssid, pwd);
	if (updateType == STM8)
		stm8_ota_download(url);
	else
		esp_ota_download(url);
}

uint8_t ota_start(uint8_t* data, uint16_t len, uint8_t what) {
	if ((what == STM8) && !checkLoaderPartition()) {
		ESP_LOGE(TAG, "STM8 Loader Partition not valid");
		return 2;
	}
	char* copy = (char*)malloc(len+1);
	memcpy(copy, data, len);
	copy[len] = '\0';
	ssid = strsep(&copy, "|");
	pwd  = strsep(&copy, "|");
	url  = strsep(&copy, "|");
	if (ssid == NULL || pwd == NULL || url == NULL) {
		ESP_LOGE(TAG, "ota_start - Command parameters Error");
		free(copy);
		return 1;
	}
	vTaskSuspend(mainTaskHandle);
	updateType = what;
	xTaskCreate(ota_task, "ota_task", 3072, NULL, 5, NULL);
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
