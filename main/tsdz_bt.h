/*
 * tsdz_bt.h
 *
 *  Created on: 10 set 2019
 *      Author: Max
 */

#ifndef MAIN_TSDZ_BT_H_
#define MAIN_TSDZ_BT_H_


enum
{
    IDX_SVC,

    IDX_CHAR_STATUS,
    IDX_CHAR_VAL_STATUS,
    IDX_CHAR_CFG_STATUS,

    IDX_CHAR_DEBUG,
    IDX_CHAR_VAL_DEBUG,
    IDX_CHAR_CFG_DEBUG,

    IDX_CHAR_CONFIG,
    IDX_CHAR_VAL_CONFIG,

    IDX_CHAR_COMMAND,
    IDX_CHAR_VAL_COMMAND,
    IDX_CHAR_CFG_COMMAND,
    HRS_IDX_NB,
};

volatile uint8_t btCommandReady;

void tsdz_bt_init(void);
void tsdz_bt_update(void);
void tsdz_bt_notify_command(uint8_t* value, uint8_t length);
void tsdz_bt_stop(void);

#endif /* MAIN_TSDZ_BT_H_ */
