/*
 * tsdz_ota_stm8.c
 *
 *  Created on: 10 apr 2020
 *      Author: Max
 */
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "tsdz_uart.h"
#include "tsdz_nvs.h"
#include "tsdz_bt.h"
#include "tsdz_utils.h"
#include "tsdz_wifi.h"
#include "tsdz_commands.h"

#include "stm8s/E_W_ROUTINEs_32K_ver_10.h"
#include "stm8s/E_W_ROUTINEs_32K_ver_12.h"
#include "stm8s/E_W_ROUTINEs_32K_ver_13.h"
#include "stm8s/E_W_ROUTINEs_32K_ver_14.h"


static const char* TAG = "tsdz_ota_stm8";


#define MAX_FW_SIZE 32768

// BSL command codes
#define GET     0x00      //< gets version and commands supported by the BSL
#define READ    0x11      //< read up to 256 bytes of memory
#define ERASE   0x43      //< erase flash program memory/data EEPROM sectors
#define WRITE   0x31      //< write up to 128 bytes to RAM or flash
#define GO      0x21      //< jump to a specified address e.g. flash

// BSL return codes
#define SYNCH   0x7F      //< Synchronization byte
#define ACK     0x79      //< Acknowledge
#define NACK    0x1F      //< No acknowledge
#define BUSY    0xAA      //< Busy flag status

volatile int stm8_ota_status = 0;

/**
  \fn uint32_t send_port(uart_port_t uartNum, uint8_t uartMode, uint32_t lenTx, char *Tx)

  \param[in] uartNum    handle to comm port
  \param[in] uartMode   UART bootloader mode: 0=duplex, 1=1-wire reply, 2=2-wire reply
  \param[in] lenTx      number of bytes to send
  \param[in] Tx         array of bytes to send

  \return number of sent bytes

  send data via comm port. Use this function to facilitate serial communication
  on different platforms, e.g. Win32 and Posix.
  If uartMode==1 (1-wire interface), read back LIN echo
*/
static uint32_t send_port(uart_port_t uartNum, uint8_t uartMode, uint32_t lenTx, char *Tx) {
	int  numChars;

	numChars = uart_write_bytes(uartNum, Tx, (size_t)lenTx);

	// return number of sent bytes
	return((uint32_t) numChars);
}

/**
  \fn uint32_t receive_port(uart_port_t uartNum, uint8_t uartMode, uint32_t lenRx, char *Rx)

  \param[in]  uartNum     handle to comm port
  \param[in]  uartMode  UART bootloader mode: 0=duplex, 1=1-wire reply, 2=2-wire reply
  \param[in]  lenRx     number of bytes to receive
  \param[out] Rx        array containing bytes received

  \return number of received bytes

  receive data via comm port. Use this function to facilitate serial communication
  on different platforms, e.g. Win32 and Posix
  If uartMode==2 (UART reply mode with 2-wire interface), reply each byte from STM8 -> SLOW
*/
static uint32_t receive_port(uart_port_t uartNum, uint8_t uartMode, uint32_t lenRx, char *Rx, uint32_t timeout) {
	int numChars, numTmp;
	uint32_t i;

	// for UART reply mode with 2-wire interface echo each received bytes -> SLOW
	if (uartMode==2) {
		// echo each byte as it is received
		numChars = 0;
		for (i=0; i<lenRx; i++) {
			// ReadFile(uartNum, Rx+i, 1, &numTmp, NULL);
			numTmp = uart_read_bytes(uartNum, (uint8_t*)Rx+i, 1, pdMS_TO_TICKS(timeout));
			if (numTmp == 1) {
				numChars++;
				send_port(uartNum, uartMode, 1, Rx+i);
			} else
				break;
		} // loop i
	} // uartMode==2
	// UART duplex mode or 1-wire interface -> receive all bytes in single block -> fast
	else {
		//ReadFile(uartNum, Rx, lenRx, &numChars, NULL);
		numChars = uart_read_bytes(uartNum, (uint8_t*)Rx, lenRx, pdMS_TO_TICKS(timeout));
	}

	// return number of bytes received
	return((uint32_t) numChars);
} // receive_port


/**
  \fn uint8_t bsl_sync(uart_port_t ptrPort)

  \param[in]  ptrPort        handle to communication port

  \return synchronization status (0=ok, 1=fail)

  synchronize with microcontroller bootloader. For UART synchronize baudrate.
*/
uint8_t bsl_sync(uart_port_t ptrPort) {

	int   count = 0;
	int   len;
	char  Rx[2];
	char  Tx[] = {SYNCH};

	// purge UART input buffer
	uart_flush_input(ptrPort);

	count = 0;
	do {
		len  = send_port(ptrPort, 0, 1, Tx);
		if (len != 1)
			ESP_LOGE(TAG, "Error in 'bsl_sync()': sending command failed (expect 1, sent %d)", len);

		// receive response
		len = receive_port(ptrPort, 0, 1, Rx, 900);
		if ((len == 1) && (Rx[0] == Tx[0])) {              // check for 1-wire echo
			ESP_LOGI(TAG, "bsl_sync - 1-wire echo");
			len = receive_port(ptrPort, 0, 1, Rx, 900);
		}

		// increase retry counter
		count++;

		// avoid flooding the STM8
		//SLEEP(10);
		vTaskDelay(pdMS_TO_TICKS(10));

	} while ((count<10) && ((len!=1) || ((Rx[0]!=ACK) && (Rx[0]!=NACK))));

	// check if ok
	if ((len==1) && (Rx[0]==ACK))
		ESP_LOGI(TAG, "bsl_sync - done (ACK) !!!");
	else if ((len==1) && (Rx[0]==NACK))
		ESP_LOGI(TAG, "bsl_sync - done (NACK)");
	else if (len==1)
		ESP_LOGE(TAG, "Error in 'bsl_sync()': wrong response 0x%02x from BSL", (uint8_t) (Rx[0]));
	else
		ESP_LOGE(TAG, "Error in 'bsl_sync()': no response from BSL");

	// purge PC input buffer
	uart_flush_input(ptrPort);
	vTaskDelay(pdMS_TO_TICKS(50));          // seems to be required for some reason

	// need to reply ACK first to revert bootloader
	char c = ACK;
	send_port(CT_UART, 0, 1, &c);

	// return success
	if ((len==1) && (Rx[0]==ACK))
		return(1);
	else
		return(0);
}


/**
  \fn uint8_t bsl_memCheck(uart_port_t ptrPort, uint8_t physInterface, uint8_t uartMode, uint32_t addr, uint8_t verbose)

  \param[in]  ptrPort        handle to communication port
  \param[in]  uartMode       UART bootloader mode: 0=duplex, 1=1-wire, 2=2-wire reply
  \param[in]  addr           address to check

  \return communication status (0=ok, 1=fail)

  check if microcontrolles address exists. Specifically read 1B from microcontroller
  memory via READ command. If it fails, memory doesn't exist. Used to get STM8 type
*/
uint8_t bsl_memCheck(uart_port_t ptrPort, uint8_t uartMode, uint32_t addr) {
	int       i, lenTx, lenRx, len;
	char      Tx[10], Rx[10];

	// init receive buffer
	for (i=0; i<10; i++)
		Rx[i] = 0;

	/////
	// send read command
	/////

	// construct command
	lenTx = 2;
	Tx[0] = READ;
	Tx[1] = (Tx[0] ^ 0xFF);
	lenRx = 1;

	// send command
	len = send_port(ptrPort, uartMode, lenTx, Tx);
	if (len != lenTx) {
		ESP_LOGE(TAG, "Error in 'bsl_memCheck()': sending command failed (expect %d, sent %d)", lenTx, len);
		return 0;
	}

	// receive response
	len = receive_port(ptrPort, uartMode, lenRx, Rx, 200);
	if (len != lenRx) {
		ESP_LOGE(TAG, "Error in 'bsl_memCheck()': ACK1 timeout (expect %d, received %d)", lenRx, len);
		return 0;
	}

	// check acknowledge
	if (Rx[0]!=ACK) {
		ESP_LOGE(TAG, "Error in 'bsl_memCheck()': ACK1 failure (expect 0x%02x, received 0x%02x)", (uint8_t) ACK, (uint8_t) (Rx[0]));
		return 0;
	}

	/////
	// send address
	/////

	// construct address + checksum (XOR over address)
	lenTx = 5;
	Tx[0] = (char) (addr >> 24);
	Tx[1] = (char) (addr >> 16);
	Tx[2] = (char) (addr >> 8);
	Tx[3] = (char) (addr);
	Tx[4] = (Tx[0] ^ Tx[1] ^ Tx[2] ^ Tx[3]);
	lenRx = 1;

	// send command
	len = send_port(ptrPort, uartMode, lenTx, Tx);
	if (len != lenTx) {
		ESP_LOGE(TAG, "Error in 'bsl_memCheck()': sending address failed (expect %d, sent %d)", lenTx, len);
		return 0;
	}

	// receive response
	len = receive_port(ptrPort, uartMode, lenRx, Rx, 200);
	if (len != lenRx) {
		ESP_LOGE(TAG, "Error in 'bsl_memCheck()': ACK2 timeout (expect %d, received %d)", lenRx, len);
		return 0;
	}

	// check acknowledge -> on NACK memory cannot be read -> return 0
	if (Rx[0]!=ACK) {
		return(0);
	}

	/////
	// send number of bytes to read
	/////

	// construct number of bytes + checksum
	lenTx = 2;
	Tx[0] = 1-1;            // -1 from BSL
	Tx[1] = (Tx[0] ^ 0xFF);
	lenRx = 2;

	// send command
	len = send_port(ptrPort, uartMode, lenTx, Tx);
	if (len != lenTx) {
		ESP_LOGE(TAG, "Error in 'bsl_memCheck()': sending range failed (expect %d, sent %d)", lenTx, len);
		return 0;
	}

	// receive response
	len = receive_port(ptrPort, uartMode, lenRx, Rx, 3000);
	if (len != lenRx) {
		ESP_LOGE(TAG, "Error in 'bsl_memCheck()': data timeout (expect %d, received %d)", lenRx, len);
		return 0;
	}

	// check acknowledge
	if (Rx[0]!=ACK) {
		ESP_LOGE(TAG, "Error in 'bsl_memCheck()': ACK3 failure (expect 0x%02x, received 0x%02x)", (uint8_t) ACK, (uint8_t) (Rx[0]));
		return 0;
	}

	// memory read succeeded -> memory exists
	return(1);

} // bsl_memCheck


/**
  \fn uint8_t bsl_getInfo(uart_port_t ptrPort, uint8_t physInterface, uint8_t uartMode, int *flashsize, uint8_t *vers, uint8_t *family, uint8_t verbose)

  \param[in]  ptrPort        handle to communication port
  \param[in]  uartMode       UART bootloader mode: 0=duplex, 1=1-wire, 2=2-wire reply
  \param[out] flashsize      size of flashsize in kB (required for correct W/E routines)
  \param[out] vers           BSL version number (required for correct W/E routines)

  \return communication status (0=ok, 1=fail)

  query microcontroller type and BSL version info. This information is required
  to select correct version of flash write/erase routines
*/
uint8_t bsl_getInfo(uart_port_t ptrPort, uint8_t uartMode, int *flashsize, uint8_t *vers) {

	int   i;
	int   lenTx, lenRx, len;
	char  Tx[3], Rx[10];

	// print message
	ESP_LOGI(TAG, "  get device info ...");

	// init receive buffer
	for (i=0; i<10; i++)
		Rx[i] = 0;

	// purge input buffer
	//flush_port(ptrPort);
	uart_flush_input(ptrPort);
	//SLEEP(50);              // seems to be required for some reason
	vTaskDelay(pdMS_TO_TICKS(50));

	/////////
	// determine device flash size for selecting w/e routines (flash starts at PFLASH_START)
	/////////

	// check if adress in flash exists. Check highest flash address to determine size
	if (bsl_memCheck(ptrPort, uartMode, 0x00FFFF))  // medium density (32kB)
		*flashsize = 32;
	else {
		ESP_LOGE(TAG, "Error in 'bsl_getInfo()': cannot identify device");
		return 1;
	}

	/////////
	// get BSL version
	/////////

	// construct command
	lenTx = 2;
	Tx[0] = GET;
	Tx[1] = (Tx[0] ^ 0xFF);
	lenRx = 9;

	// send command
	len = send_port(ptrPort, uartMode, lenTx, Tx);
	if (len != lenTx) {
		ESP_LOGE(TAG, "Error in 'bsl_getInfo()': sending command failed (expect %d, sent %d)", lenTx, len);
		return 1;
	}

	// receive response
	len = receive_port(ptrPort, uartMode, lenRx, Rx, 1000);
	if (len != lenRx) {
		ESP_LOGE(TAG, "Error in 'bsl_getInfo()': ACK timeout (expect %d, received %d)", lenRx, len);
		return 1;
	}

	// check 2x ACKs
	if (Rx[0]!=ACK) {
		ESP_LOGE(TAG, "Error in 'bsl_getInfo()': start ACK failure (expect 0x%02x, read 0x%02x)", (uint8_t) ACK, (uint8_t) (Rx[0]));
		return 1;
	}
	if (Rx[8]!=ACK) {
		ESP_LOGE(TAG, "Error in 'bsl_getInfo()': end ACK failure (expect 0x%02x, read 0x%02x)", (uint8_t) ACK, (uint8_t) (Rx[8]));
		return 1;
	}

	// check if command codes are correct (just to be sure)
	if (Rx[3] != GET) {
		ESP_LOGE(TAG, "Error in 'bsl_getInfo()': wrong GET code (expect 0x%02x, received 0x%02x)", (uint8_t) GET, (uint8_t) (Rx[3]));
		return 1;
	}
	if (Rx[4] != READ) {
		ESP_LOGE(TAG, "Error in 'bsl_getInfo()': wrong READ code (expect 0x%02x, received 0x%02x)", (uint8_t) READ, (uint8_t) (Rx[4]));
		return 1;
	}
	if (Rx[5] != GO) {
		ESP_LOGE(TAG, "Error in 'bsl_getInfo()': wrong GO code (expect 0x%02x, received 0x%02x)", (uint8_t) GO, (uint8_t) (Rx[5]));
		return 1;
	}
	if (Rx[6] != WRITE) {
		ESP_LOGE(TAG, "Error in 'bsl_getInfo()': wrong WRITE code (expect 0x%02x, received 0x%02x)", (uint8_t) WRITE, (uint8_t) (Rx[6]));
		return 1;
	}
	if (Rx[7] != ERASE) {
		ESP_LOGE(TAG, "Error in 'bsl_getInfo()': wrong ERASE code (expect 0x%02x, received 0x%02x)", (uint8_t) ERASE, (uint8_t) (Rx[7]));
		return 1;
	}

	// print BSL data
	// ESP_LOGI(TAG,"    version 0x%02x", (uint8_t) (Rx[2]));
	// ESP_LOGI(TAG,"    command codes:");
	// ESP_LOGI(TAG,"      GET   0x%02x", (uint8_t) (Rx[3]));
	// ESP_LOGI(TAG,"      READ  0x%02x", (uint8_t) (Rx[4]));
	// ESP_LOGI(TAG,"      GO    0x%02x", (uint8_t) (Rx[5]));
	// ESP_LOGI(TAG,"      WRITE 0x%02x", (uint8_t) (Rx[6]));
	// ESP_LOGI(TAG,"      ERASE 0x%02x", (uint8_t) (Rx[7]));

	// copy version number
	*vers = Rx[2];

	// print message
	ESP_LOGI(TAG,"done (STM8S; %dkB flash; BSL v%x.%x)", *flashsize, (((*vers)&0xF0)>>4), ((*vers) & 0x0F));

	// avoid compiler warnings
	return(0);
} // bsl_getInfo


/**
  \fn uint8_t bsl_memWrite(uart_port_t ptrPort, uint8_t physInterface, uint8_t uartMode, uint16_t *imageBuf, uint32_t addrStart, uint32_t addrStop, uint8_t verbose)

  \param[in]  ptrPort        handle to communication port
  \param[in]  uartMode       UART bootloader mode: 0=duplex, 1=1-wire, 2=2-wire reply
  \param[out] imageBuf       memory image of data to write (16-bit array. HB!=0 indicates content)
  \param[in]  addrStart      first address to write to
  \param[in]  addrStop       last address to write to

  \return communication status (0=ok, 1=fail)

  upload data to microcontroller memory via WRITE command
*/
uint8_t bsl_memWrite(uart_port_t ptrPort, uint8_t uartMode, const uint8_t *imageBuf, const uint32_t addrStart, const uint32_t addrStop, const bool sendProgress) {

	uint32_t         numData;
	uint32_t		 countBytes, countBlock;    		  // size of memory image
	const uint32_t   maxBlock = 128;                      // max. length of write block
	char             Tx[128+4], Rx[16];                  // communication buffers
	int              lenTx, lenRx, len;                   // frame lengths
	uint8_t          chk;                                 // frame checksum

	uint8_t ret[3] = {CMD_STM8_OTA_STATUS,3,0};

	numData = addrStop - addrStart+1;
	// update min/max addresses and number of bytes to write (HB!=0x00) for printout
	//get_image_size(imageBuf, addrStart, addrStop, &addrStart, &addrStop, &numData);

	// print message
    if (numData > 1024)
    	ESP_LOGI(TAG,"  write %1.1fkB in 0x%04x to 0x%04x ", (float) numData/1024.0, (int) addrStart, (int) addrStop);
    else
    	ESP_LOGI(TAG,"  write %dB in 0x%04x to 0x%04x ", (int) numData, (int) addrStart, (int) addrStop);


	// init receive buffer
	for (int i=0; i<16; i++)
		Rx[i] = 0;


	// loop over specified address range
	// Write only defined bytes (HB!=0x00) and align to 128 to minimize write time (see UM0560 section 3.4)
	countBytes = 0;
	countBlock = 0;
	uint32_t addr = addrStart;
	while (addr <= addrStop) {
		// end address reached -> done
		if (addr > addrStop)
		  break;

		// set length of next data block: max 128B and align with 128 for speed (see UM0560 section 3.4)
		int lenBlock = 1;
		while ((lenBlock < maxBlock) && ((addr+lenBlock) <= addrStop) && ((addr+lenBlock) % maxBlock)) {
			lenBlock++;
		}

		/////
		// send write command
		/////

		// construct command
		lenTx = 2;
		Tx[0] = WRITE;
		Tx[1] = (Tx[0] ^ 0xFF);
		lenRx = 1;

		// send command
		len = send_port(ptrPort, uartMode, lenTx, Tx);
		if (len != lenTx) {
			ESP_LOGE(TAG, "Error in 'bsl_memWrite()': sending command failed (expect %d, sent %d)", lenTx, len);
			return 1;
		}

		// receive response
		len = receive_port(ptrPort, uartMode, lenRx, Rx, 1500);
		if (len != lenRx) {
			ESP_LOGE(TAG, "Error in 'bsl_memWrite()': ACK1 timeout (expect %d, received %d)", lenRx, len);
			return 1;
		}		// check acknowledge
		if (Rx[0]!=ACK) {
			ESP_LOGE(TAG, "Error in 'bsl_memWrite()': ACK1 failure (expect 0x%02x, received 0x%02x)", (uint8_t) ACK, (uint8_t) (Rx[0]));
			return 1;
		}

		/////
		// send address
		/////

		// construct address + checksum (XOR over address)
		lenTx = 5;
		Tx[0] = (char) (addr >> 24);
		Tx[1] = (char) (addr >> 16);
		Tx[2] = (char) (addr >> 8);
		Tx[3] = (char) (addr);
		Tx[4] = (Tx[0] ^ Tx[1] ^ Tx[2] ^ Tx[3]);
		lenRx = 1;

		// send command
		len = send_port(ptrPort, uartMode, lenTx, Tx);
		if (len != lenTx) {
			ESP_LOGE(TAG, "Error in 'bsl_memWrite()': sending address failed (expect %d, sent %d)", lenTx, len);
			return 1;
		}

		// receive response
		len = receive_port(ptrPort, uartMode, lenRx, Rx, 1500);
		if (len != lenRx) {
			ESP_LOGE(TAG, "Error in 'bsl_memWrite()': ACK2 timeout (expect %d, received %d)", lenRx, len);
			return 1;
		}		// check acknowledge
		if (Rx[0]!=ACK) {
			ESP_LOGE(TAG, "Error in 'bsl_memWrite()': ACK2 failure (expect 0x%02x, received 0x%02x)", (uint8_t) ACK, (uint8_t) (Rx[0]));
			return 1;
		}

		/////
		// send number of bytes and data
		/////

		// construct number of bytes + data + checksum
		lenTx = 0;
		Tx[lenTx++] = lenBlock-1;     // -1 from BSL
		chk         = lenBlock-1;
		for (int j=0; j<lenBlock; j++) {
			Tx[lenTx] = imageBuf[addr-addrStart+j];  // only LB, HB indicates "defined"
			chk ^= Tx[lenTx];
			lenTx++;
			countBytes++;
		}
		Tx[lenTx++] = chk;
		lenRx = 1;

		// send command
		len = send_port(ptrPort, uartMode, lenTx, Tx);
		if (len != lenTx) {
			ESP_LOGE(TAG, "Error in 'bsl_memWrite()': sending data failed (expect %d, sent %d)", lenTx, len);
			return 1;
		}

		// receive response
		len = receive_port(ptrPort, uartMode, lenRx, Rx, 3000);
		if (len != lenRx) {
			ESP_LOGE(TAG, "Error in 'bsl_memWrite()': ACK3 timeout (expect %d, received %d)", lenRx, len);
			return 1;
		}		// check acknowledge
		if (Rx[0]!=ACK) {
			ESP_LOGE(TAG, "Error in 'bsl_memWrite()': ACK3 failure (expect 0x%02x, received 0x%02x)", (uint8_t) ACK, (uint8_t) (Rx[0]));
			return 1;
		}

		// print progress
		if (((++countBlock) % 8) == 0) {
			if (sendProgress) {
				// send progress notification
				ret[2] = (100 * countBytes) / numData;
				tsdz_bt_notify_command(ret, 3);
			}
			if (numData > 1024)
				ESP_LOGI(TAG, "write %1.1fkB / %1.1fkB in 0x%04x to 0x%04x ", (float) countBytes/1024.0, (float) numData/1024.0, (int) addrStart, (int) addrStop);
			else
				ESP_LOGI(TAG, "write %dB / %dB in 0x%04x to 0x%04x ", (int) countBytes, (int) numData, (int) addrStart, (int) addrStop);
		}

		// go to next potential block
		addr += lenBlock;
	} // loop over address range

	// send 100% progress notification
	if (sendProgress) {
		ret[2] = 100;
		tsdz_bt_notify_command(ret, 3);
	}
	// print message
	if (numData > 1024)
		ESP_LOGI(TAG, "%c  write %1.1fkB / %1.1fkB in 0x%04x to 0x%04x ... done   \n", '\r', (float) countBytes/1024.0, (float) numData/1024.0, (int) addrStart, (int) addrStop);
	else
		ESP_LOGI(TAG, "%c  write %dB / %dB in 0x%04x to 0x%04x ... done   \n", '\r', (int) countBytes, (int) numData, (int) addrStart, (int) addrStop);

  return(0);
} // bsl_memWrite

/**
  \fn uint8_t bsl_jumpTo(uart_port_t ptrPort, uint8_t physInterface, uint8_t uartMode, uint32_t addr, uint8_t verbose)

  \param[in]  ptrPort        handle to communication port
  \param[in]  uartMode       UART bootloader mode: 0=duplex, 1=1-wire, 2=2-wire reply
  \param[in]  addr           address to jump to

  \return communication status (0=ok, 1=fail)

  jump to address and continue code execution. Generally RAM or flash starting address
*/
uint8_t bsl_jumpTo(uart_port_t ptrPort, uint8_t uartMode, uint32_t addr) {
  int       i;
  int       lenTx, lenRx, len;
  char      Tx[10], Rx[10];

  ESP_LOGI(TAG, "  jump to address 0x%04x ... ", addr);

  // init receive buffer
  for (i=0; i<10; i++)
    Rx[i] = 0;

  /////
  // send go command
  /////

  // construct command
  lenTx = 2;
  Tx[0] = GO;
  Tx[1] = (Tx[0] ^ 0xFF);
  lenRx = 1;

  // send command
  len = send_port(ptrPort, uartMode, lenTx, Tx);
  if (len != lenTx) {
	  ESP_LOGE(TAG,"Error in 'bsl_jumpTo()': sending command failed (expect %d, sent %d)", lenTx, len);
	  return 1;
  }

  // receive response
  len = receive_port(ptrPort, uartMode, lenRx, Rx, 1500);
  if (len != lenRx) {
	  ESP_LOGE(TAG,"Error n 'bsl_jumpTo()': ACK1 timeout (expect %d, received %d)", lenRx, len);
	  return 1;
  }

  // check acknowledge
  if (Rx[0]!=ACK) {
	  ESP_LOGE(TAG,"Error in 'bsl_jumpTo()': ACK1 failure (expect 0x%02x, received 0x%02x)", (uint8_t) ACK, (uint8_t) (Rx[0]));
	  return 1;
  }

  /////
  // send address
  /////

  // construct address + checksum (XOR over address)
  lenTx = 5;
  Tx[0] = (char) (addr >> 24);
  Tx[1] = (char) (addr >> 16);
  Tx[2] = (char) (addr >> 8);
  Tx[3] = (char) (addr);
  Tx[4] = (Tx[0] ^ Tx[1] ^ Tx[2] ^ Tx[3]);
  lenRx = 1;

  // send command
  len = send_port(ptrPort, uartMode, lenTx, Tx);
  if (len != lenTx) {
	  ESP_LOGE(TAG,"Error in 'bsl_jumpTo()': sending address failed (expect %d, sent %d)", lenTx, len);
	  return 1;
  }

  // receive response
  len = receive_port(ptrPort, uartMode, lenRx, Rx, 1500);
  if (len != lenRx) {
	  ESP_LOGE(TAG,"Error in 'bsl_jumpTo()': ACK2 timeout (expect %d, received %d)", lenRx, len);
	  return 1;
  }

  // check acknowledge
  if (Rx[0]!=ACK) {
	  ESP_LOGE(TAG,"Error in 'bsl_jumpTo()': ACK2 failure (expect 0x%02x, received 0x%02x)", (uint8_t) ACK, (uint8_t) (Rx[0]));
	  return 1;
  }

  ESP_LOGI(TAG, "bsl_jumpTo done");

  // avoid compiler warnings
  return(0);

} // bsl_jumpTo


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

void stm8_ota_task() {
    uint8_t ret[3] = {CMD_STM8_OTA_STATUS,0,0};

    // Wait for stm8 bootloader activation
    // Note: STM8 Bootloader has only 1 sec detection window after boot
    vTaskDelay(pdMS_TO_TICKS(300));

    // Send SYNC to STM8 Bootloader
	if (!bsl_sync(CT_UART)) {
		ESP_LOGE(TAG, "STM8 Bootloader Sync failed");
		ret[2] = 0;
		goto error;
	}
	ESP_LOGI(TAG, "STM8 Bootloader Sync Done!");

	// connect to the Access Point
    if (!wifi_start_sta()) {
    	ESP_LOGE(TAG, "Connection to WiFi AP failed");
		ret[2] = 13;
		goto error;
	}
    ESP_LOGI(TAG, "WiFi connected");

    // set URL
    char url[32];
    snprintf(url, 32, "http://%s:%d", wifi_get_address(), wifi_get_port());
    ESP_LOGI(TAG, "stm8_ota_task: url:%s", url);

    // Allocate memory for Firmware Image
    char* firmwareImage = malloc(MAX_FW_SIZE);
    if (firmwareImage == NULL) {
		ESP_LOGE(TAG, "Cannot allocate memory for firmware image");
		ret[2] = 2;
		goto error;	
    }
    
    // connect to url
    esp_http_client_config_t config = {
		.url = url,
		.event_handler = _http_event_handler,
	};
    esp_err_t err;
	esp_http_client_handle_t client = esp_http_client_init(&config);
	if ((err = esp_http_client_open(client, 0)) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
		ret[2] = 3;
		goto error;
	}

	int binary_len = esp_http_client_fetch_headers(client);
	if (binary_len < 0) {
		ESP_LOGE(TAG, "esp_http_client_fetch_headers Error");
		ret[2] = 4;
		goto error;
	}
	if (binary_len > 1024*32) {
		ESP_LOGE(TAG, "STM8 FW size wrong: %d bytes", binary_len);
		ret[2] = 5;
		goto error;
	}
	ESP_LOGI(TAG, "Reading FW data: %d bytes", binary_len);

	// Firmware download
	uint32_t total_read = 0;
	int read_len;
	while (((binary_len > 0) && (total_read < binary_len)) ||
			((binary_len == 0) && !esp_http_client_is_complete_data_received(client))) {
		if (total_read >= 1024*32) {
			ESP_LOGE(TAG, "STM8 FW size too big");
			ret[2] = 6;
			goto error;
		}
		read_len = esp_http_client_read(client, &firmwareImage[total_read], MAX_FW_SIZE-total_read);
		if (read_len < 0) {
			ESP_LOGE(TAG, "Error read data");
			ret[2] = 7;
			goto error;
		}
		total_read += read_len;
	}    
    
	// Firmware download completed. Deinit Wifi
	tsdz_wifi_deinit();
    
    ret[1] = 1;
	ESP_LOGI(TAG, "Sending end OTA Status, Phase 1 OK");
	tsdz_bt_notify_command(ret, 3);

	int flashsize;
	uint8_t bslVers;
	if (bsl_getInfo(CT_UART, 2, &flashsize, &bslVers)) {
		ret[2] = 8;
		goto error;
	}

	// Send update routines to STM8 RAM
	uint32_t startAddr = 0;
    uint32_t endAddr = 0;
    const uint8_t  *ptrRAM = NULL;          // pointer to array with RAM routines

    if (bslVers==0x10) {
		ESP_LOGI(TAG, "header STM8_Routines_E_W_ROUTINEs_32K_ver_1_0_s19");
		ptrRAM = E_W_ROUTINEs_32K_ver_10;
		startAddr = E_W_ROUTINEs_32K_ver_10_start;
		endAddr = E_W_ROUTINEs_32K_ver_10_end;
	} else if (bslVers==0x12) {
		ESP_LOGI(TAG, "header STM8_Routines_E_W_ROUTINEs_32K_ver_1_2_s19");
		ptrRAM = E_W_ROUTINEs_32K_ver_12;
		startAddr = E_W_ROUTINEs_32K_ver_12_start;
		endAddr = E_W_ROUTINEs_32K_ver_12_end;
	} else if (bslVers==0x13) {
		ESP_LOGI(TAG, "header STM8_Routines_E_W_ROUTINEs_32K_ver_1_3_s19");
		ptrRAM = E_W_ROUTINEs_32K_ver_13;
		startAddr = E_W_ROUTINEs_32K_ver_13_start;
		endAddr = E_W_ROUTINEs_32K_ver_13_end;
	} else if (bslVers==0x14) {
		ESP_LOGI(TAG, "header STM8_Routines_E_W_ROUTINEs_32K_ver_1_4_s19");
		ptrRAM = E_W_ROUTINEs_32K_ver_14;
		startAddr = E_W_ROUTINEs_32K_ver_14_start;
		endAddr = E_W_ROUTINEs_32K_ver_14_end;
	} else {
		ESP_LOGE(TAG, "Error: Unknown bootloader version: 0x%02x", bslVers);
		ret[2] = 9;
		goto error;
	}

    if (bsl_memWrite(CT_UART, 2, ptrRAM, startAddr, endAddr, false)){
		ret[2] = 10;
		goto error;
	}
    ESP_LOGI(TAG, "STM8 Routintes send done");

    ret[1] = 2;
	tsdz_bt_notify_command(ret, 3);

    // Write new firmware to STM8 memory
    startAddr = 0x8000;
    uint32_t jumpAddr = 0x8000;

	if (bsl_memWrite(CT_UART, 2, (uint8_t*)firmwareImage, startAddr, startAddr+total_read-1, true)){
		ret[2] = 11;
		goto error;
	}
    ESP_LOGI(TAG, "STM8S Firmware upgraded!");

    if (bsl_jumpTo(CT_UART, 2, jumpAddr)) {
		ret[2] = 12;
		goto error;
    }
    ESP_LOGI(TAG, "New STM8 FW started!");

	ret[1] = 0;
	ret[2] = 0;
	tsdz_bt_notify_command(ret, 3);
	ESP_LOGI(TAG, "Reboot in 1 sec.");
	vTaskDelay(pdMS_TO_TICKS(1000));
	esp_restart();
	
  error:
	ret[1] = 4;
	tsdz_bt_notify_command(ret, 3);
	ESP_LOGI(TAG, "Reboot in 1 sec.");
	vTaskDelay(pdMS_TO_TICKS(1000));
	esp_restart();
}

uint8_t ota_stm8_start(uint8_t* data, uint16_t len) {
    char *unmodified_copy, *copy;
    char *ssid;
    char *pwd;
    int port = 0;

    copy = (char*)malloc(len+1);
    memcpy(copy, data, len);
    copy[len] = '\0';
    unmodified_copy = copy;

    ssid = strsep(&copy, "|");
    pwd = strsep(&copy, "|");
    port = atoi(strsep(&copy, "|"));
    ESP_LOGI(TAG, "ota_esp32_start: ssid:%s pwd:%s port:%d", ssid, pwd, port);

    if (ssid == NULL || pwd == NULL || port == 0) {
        ESP_LOGE(TAG, "ota_start - Command parameters Error");
        free(unmodified_copy);
        return 1;
    }

    wifi_set_data(ssid, pwd, port);
    free(unmodified_copy);

    stm8_ota_status = 1;
    return 0;
}


