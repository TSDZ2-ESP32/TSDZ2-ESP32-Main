/*
 * tsdz_uart.c
 *
 *  Created on: 2 set 2019
 *      Author: Max
 */

#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/task.h"
#include "tsdz_data.h"
#include "tsdz_uart.h"
#include "tsdz_utils.h"


#define DEBUG_MSG_ID				0x7C

#define SPECIAL                     0xED
#define STX                         0xEE
#define ETX                         0xEF


char senBuf[256];

uint8_t lcd_recived_msg[LCD_OEM_MSG_BYTES];
uint8_t lcd_rx_counter = 0;
uint8_t lcd_state_machine = 0;

uint8_t ct_received_msg[CT_OS_MSG_BYTES];
uint8_t ct_rx_counter = 0;
uint8_t ct_state_machine = 0;

uint8_t lcd_send_msg[CT_OEM_MSG_BYTES];
uint8_t ct_send_msg[LCD_OS_MSG_BYTES];

uint8_t usb_message[256];
uint8_t usb_message_length;
uint8_t usb_rx_counter = 0;
uint8_t usb_state_machine = 0;

#define TAG "tsdz_urt"

bool lcdMessageReceived(void);
bool ctMessageReceived(void);
bool checkCRC(uint8_t *message, uint8_t count);
bool checkCRC16(uint8_t *message, uint8_t count);
char* bytesToHex(uint8_t* bytes, uint8_t n);


void tsdz_uart_init(void) {

	/* Configure parameters of an UART driver,
	 * communication pins and install the driver */
	uart_config_t uart_config = {
			.baud_rate = 9600,
			.data_bits = UART_DATA_8_BITS,
			.parity    = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};

	esp_err_t err;
	err = uart_param_config(LCD_UART, &uart_config);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_param_config LCD_UART error=%d", err);
	}
	err = uart_set_pin(LCD_UART, LCD_TX_PIN, LCD_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_set_pin LCD_UART error=%d", err);
	}
	err = uart_driver_install(LCD_UART, 256, 256, 0, NULL, 0);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_driver_install LCD_UART error=%d", err);
	}

	err = uart_param_config(CT_UART, &uart_config);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_param_config CT_UART error=%d", err);
	}
	err = uart_set_pin(CT_UART, CT_TX_PIN, CT_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_set_pin CT_UART error=%d", err);
	}
	err = uart_driver_install(CT_UART, 256, 256, 0, NULL, 0);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_driver_install CT_UART error=%d", err);
	}
}

// the OEM LCD sends about 13 msg/sec but the messages to the controller
// are sent at a frequency of 10 msg/sec like the official LCD3
void tsdz_uart_task(void) {
	static bool lcdMsgReceived = false;

	// Read messages coming from LCD
	if (lcdMessageReceived()) {
		processLcdMessage(lcd_recived_msg);
		lcdMsgReceived = true;
	}

	// Read messages from Controller and send to LCD and controller
	if (ctMessageReceived()) {
		processControllerMessage(ct_received_msg);
		getLCDMessage(lcd_send_msg);
		uart_write_bytes(LCD_UART, (char*)lcd_send_msg, (size_t)CT_OEM_MSG_BYTES);
		//ESP_LOGI(TAG, "LCD Message Sent: %s", bytesToHex(lcd_send_msg,CT_OEM_MSG_BYTES));
		if (lcdMsgReceived) {
			getControllerMessage(ct_send_msg);
			uart_write_bytes(CT_UART, (char*)ct_send_msg, (size_t)LCD_OS_MSG_BYTES);
			lcdMsgReceived = false;
			//ESP_LOGI(TAG, "Controller Message Sent: %s", bytesToHex(ct_send_msg,LCD_OS_MSG_BYTES));
		}
	}
}

bool lcdMessageReceived(void) {
	uint8_t byte_received;
	size_t available;
	int received;

	//uart_get_buffered_data_len(UART_NUM_0, &available);
	uart_get_buffered_data_len(LCD_UART, &available);
	while (available > 0) {
		// received = uart_read_bytes(UART_NUM_0, byte_received, 1, 0);
		received = uart_read_bytes(LCD_UART, &byte_received, 1, 0);
		if (received > 0)
			switch (lcd_state_machine) {
			case 0:
				if (byte_received != LCD_MSG_ID) // see if we get start package byte
					break;
				lcd_rx_counter = 1;
				lcd_state_machine = 1;
				lcd_recived_msg[0] = byte_received;
				break;

			case 1:
				// save received byte and increment index for next byte
				lcd_recived_msg[lcd_rx_counter++] = byte_received;

				// reset if it is the last byte of the package and index is out of bounds
				if (lcd_rx_counter >= LCD_OEM_MSG_BYTES) {
					lcd_state_machine = 0;
					lcd_rx_counter = 0;
					if (checkCRC(lcd_recived_msg, LCD_OEM_MSG_BYTES)) {
						ESP_LOGI(TAG, "lcdMessageReceived: %s", bytesToHex(lcd_recived_msg,LCD_OEM_MSG_BYTES));
						//ESP_LOGI(TAG, "LCD Message Received");
						return true;
					} else
						ESP_LOGI(TAG, "lcdMessageReceived: %s", bytesToHex(lcd_recived_msg,LCD_OEM_MSG_BYTES));
						ESP_LOGE(TAG,"LCD-CRC-ERROR");
				}
				break;
			}
		uart_get_buffered_data_len(LCD_UART, &available);
	}
	return false;
}

bool ctMessageReceived(void) {
	uint8_t byte_received;
	size_t available;
	int received;

	uart_get_buffered_data_len(CT_UART, &available);
	while (available > 0) {
		received = uart_read_bytes(CT_UART, &byte_received, 1, 0);
		if (received > 0)
			switch (ct_state_machine) {
			case 0:
				if (byte_received != CT_MSG_ID) {// see if we get start package byte
					break;
				}
				ct_rx_counter = 1;
				ct_state_machine = 1;
				ct_received_msg[0] = byte_received;
				break;

			case 1:
				// save received byte and increment index for next byte
				ct_received_msg[ct_rx_counter++] = byte_received;

				// reset if it is the last byte of the package and index is out of bounds
				if (ct_rx_counter >= CT_OS_MSG_BYTES) {
					ct_state_machine = 0;
					ct_rx_counter = 0;
					if (checkCRC16(ct_received_msg, CT_OS_MSG_BYTES)) {
						//ESP_LOGI(TAG, "ctMessageReceived: %s", bytesToHex(ct_received_msg,CT_OS_MSG_BYTES));
						ESP_LOGI(TAG, "CT Message Received");
						return true;
					} else
						ESP_LOGE(TAG,"CONTROLLER-CRC-ERROR %s", bytesToHex(ct_received_msg,CT_OS_MSG_BYTES));
				}
				break;
			}
		uart_get_buffered_data_len(CT_UART, &available);
	}
	return false;
}

// return true if the crc8 of the message is correct
bool checkCRC(uint8_t *message, uint8_t count) {
	if (crc8(message, count - 1) == message[count - 1])
		return true;
	return false;
}

bool checkCRC16(uint8_t *message, uint8_t count) {
	uint16_t ui16_crc_rx = 0xffff;
	uint8_t ui8_i;

	for (ui8_i = 0; ui8_i < count-2; ui8_i++)
	{
		crc16 (message[ui8_i], &ui16_crc_rx);
	}

	// if CRC is ok read the package
	if (((((uint16_t) message [count - 1]) << 8) + ((uint16_t) message [count - 2])) == ui16_crc_rx)
		return true;
	else
		return false;
}

const  char HEX_ARRAY[] = "0123456789ABCDEF";
char sb[256];
char* bytesToHex(uint8_t* bytes, uint8_t n) {
	int i,j;
	for (j = 0, i = 0; j < n; j++) {
		sb[i++] = HEX_ARRAY[bytes[j] >> 4];
		sb[i++] = HEX_ARRAY[bytes[j] & 0x0F];
		sb[i++] = 0x20;
	}
	sb[i++] = 0;
	return sb;
}

