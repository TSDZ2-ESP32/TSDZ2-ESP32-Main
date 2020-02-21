/*
 * tsdz_utils.c
 *
 *  Created on: 19 set 2019
 *      Author: Max
 */

#include <stdint.h>

uint32_t filter(uint32_t ui32_new_value, uint32_t ui32_old_value, uint8_t ui8_alpha)
{
    if (ui8_alpha < 101)
    {
        uint32_t ui32_filtered_value = (((100 - ui8_alpha) * ui32_new_value) + (ui8_alpha * ui32_old_value)) / 100;

        if ((ui32_filtered_value == ui32_old_value) && (ui32_filtered_value < ui32_new_value)) { ++ui32_filtered_value; }

        return ui32_filtered_value;
    }
    else
    {
        return 0;
    }
}


int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
    // if input min is smaller than output min, return the output min value
    if (x < in_min) { return out_min; }

    // if input max is bigger than output max, return the output max value
    else if (x > in_max) { return out_max; }

    // map the input to the output range, round up if mapping bigger ranges to smaller ranges
    else  if ((in_max - in_min) > (out_max - out_min)) { return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min; }

    // map the input to the output range, round down if mapping smaller ranges to bigger ranges
    else { return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }
}


// return true if the crc8 of the message is correct
uint8_t crc8(uint8_t *message, uint8_t count) {
    uint8_t crc = 0;
    uint8_t i;
    for (i = 0; i < count; i++)
        crc += message[i];
    return crc;
}


// from here: https://github.com/FxDev/PetitModbus/blob/master/PetitModbus.c
/*
 * Function Name        : CRC16
 * @param[in]           : ui8_data  - Data to Calculate CRC
 * @param[in/out]       : ui16_crc   - Anlik CRC degeri
 * @How to use          : First initial data has to be 0xFFFF.
 */
void crc16(uint8_t ui8_data, uint16_t* ui16_crc)
{
    unsigned int i;

    *ui16_crc = *ui16_crc ^(uint16_t) ui8_data;

    for (i = 8; i > 0; i--)
    {
        if (*ui16_crc & 0x0001) { *ui16_crc = (*ui16_crc >> 1) ^ 0xA001; }
        else { *ui16_crc >>= 1; }
    }
}
