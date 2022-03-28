/*
 * tsdz_tmp112.c
 *
 *  Created on: 13 feb 2020
 *      Author: Max
 */
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "esp_log.h"
#include "driver/i2c.h"
#include "tsdz_tmp112.h"
#include "tsdz_data.h"

static const char *TAG = "tsdz_tmp112";

static uint8_t initialized = 0;
static i2c_config_t conf;

void tsdz_tmp112_init(void) {
    esp_err_t err;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_PIN;
    conf.scl_io_num = SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 100000;
    err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,"I2C drive installation error: %d", err);
        return;
    }

    err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
	if (err != ESP_OK) {
		ESP_LOGW(TAG,"i2c_driver_install error: %d", err);
	}

    // TMP112 Configuration data
    // Select configuration register(0x01)
    // Continous Conversion mode, 12-Bit Resolution, Fault Queue is 1(0x60) 0110 0000
    // Polarity low, Thermostat in Comparator mode, Disables Shutdown mode(0xA0) 1010 0000
    uint8_t config[3] = {0x01,0x60,0xA0};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,"i2c_master_start error: %d", err);
    }
    err = i2c_master_write_byte(cmd, (TPM112_ADDR << 1) | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,"i2c_master_write_byte error: %d", err);
    }
    err = i2c_master_write(cmd, config, 3, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,"i2c_master_write error: %d", err);
    }
    err = i2c_master_stop(cmd);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,"i2c_master_stop error: %d", err);
    }
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,"TMP112 sensor not found: %d", err);
        i2c_cmd_link_delete(cmd);
        return;
    }
    i2c_cmd_link_delete(cmd);

    ESP_LOGI(TAG,"TMP112 Sensor initialized");
    vTaskDelay(pdMS_TO_TICKS(10));

    // now reset again to 0 to the pointer register
    cmd = i2c_cmd_link_create();
    err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,"i2c_master_start error: %d", err);
    }
    err = i2c_master_write_byte(cmd, (TPM112_ADDR << 1) | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,"i2c_master_write_byte 1 error: %d", err);
    }
    err = i2c_master_write_byte(cmd, 0, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,"i2c_master_write_byte 2 error: %d", err);
    }
    err = i2c_master_stop(cmd);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,"i2c_master_stop error: %d", err);
    }
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS);
    if (err == ESP_OK) {
        ESP_LOGI(TAG,"TMP112 Sensor ready!");
        initialized = 1;
    } else {
        ESP_LOGW(TAG,"i2c_master_cmd_begin error: %d", err);
    }
    i2c_cmd_link_delete(cmd);
}

void tsdz_tmp112_read(void) {
    if (!initialized)
        return;

    uint8_t data[2] = {0};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TPM112_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 5 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (err == ESP_OK) {
        // Convert the data to 12-bits
        int16_t temp = (data[0] * 256 + data[1]) / 16;
        if(temp > 2047)
            temp -= 4096;
        //float cTemp = temp * 0.0625; // temperature in Celsius
        tsdz_data.i16_pcb_temperaturex10 = (int16_t)(temp * 5 / 8);
    } else {
        ESP_LOGW(TAG,"TMP112 read error: %d", err);
        tsdz_data.i16_pcb_temperaturex10 = -999;
    }
}
