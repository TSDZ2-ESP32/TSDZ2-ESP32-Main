/*
 * tsdz_data.h
 *
 *  Created on: 2 set 2019
 *      Author: Max
 */

#ifndef MAIN_TSDZ_DATA_H_
#define MAIN_TSDZ_DATA_H_

#include <stdio.h>
#include <stdint.h>



// OEM Display Status byte masks
#define OEM_ASSIST_LEVEL0            0x10
#define OEM_ASSIST_LEVEL1            0x40
#define OEM_ASSIST_LEVEL2            0x02
#define OEM_ASSIST_LEVEL3            0x04
#define OEM_ASSIST_LEVEL4            0x08


#pragma pack(1)
typedef struct _esp32_cfg {
    volatile uint8_t msg_sec;  // Android UI update speed
    volatile uint8_t ds18b20_pin;  // DS18B20 temperature sensor input pin
    volatile uint8_t log_level;
    volatile uint8_t lock_enabled; // bike lock at power on
} struct_esp32_cfg;

#pragma pack(1)
typedef struct _tsdz_cfg {
    volatile uint8_t ui8_motor_inductance_x1048576;
    volatile uint8_t ui8_motor_temperature_min_value_to_limit;
    volatile uint8_t ui8_motor_temperature_max_value_to_limit;
    volatile uint8_t ui8_motor_acceleration;
    volatile uint8_t ui8_hall_offset_adj;
    volatile uint8_t ui8_max_speed;
    volatile uint8_t ui8_street_max_speed;
    volatile uint8_t ui8_pedal_torque_per_10_bit_ADC_step_x100;
    volatile uint8_t ui8_optional_ADC_function;
    volatile uint8_t ui8_assist_without_pedal_rotation_threshold;
    volatile uint8_t ui8_lights_configuration;
    volatile uint16_t ui16_wheel_perimeter;
    volatile uint8_t ui8_cruise_mode_enabled;
    volatile uint16_t ui16_battery_voltage_reset_wh_counter_x10;
    volatile uint8_t ui8_battery_max_current;
    volatile uint8_t ui8_target_max_battery_power_div25;
    volatile uint8_t ui8_battery_cells_number;
    volatile uint16_t ui16_battery_pack_resistance_x1000;
    volatile uint16_t ui16_battery_low_voltage_cut_off_x10;
    volatile uint8_t ui8_li_io_cell_overvolt_x100;
    volatile uint8_t ui8_li_io_cell_full_bars_x100;
    volatile uint8_t ui8_li_io_cell_one_bar_x100;
    volatile uint8_t ui8_li_io_cell_empty_x100;
    volatile uint8_t ui8_phase_angle_adj;
    volatile uint8_t ui8_street_mode_power_limit_enabled;
    volatile uint8_t ui8_street_mode_throttle_enabled;
    volatile uint8_t ui8_street_mode_power_limit_div25;
    volatile uint8_t ui8_street_mode_speed_limit;
    volatile uint8_t ui8_esp32_temp_control;
    volatile uint8_t ui8_cadence_assist_level[4];
    volatile uint8_t ui8_power_assist_level[4];
    volatile uint8_t ui8_torque_assist_level[4];
    volatile uint8_t ui8_eMTB_assist_sensitivity[4];
    volatile uint8_t ui8_walk_assist_level[4];
    volatile uint8_t ui8_torque_offset_fix;
    volatile uint16_t ui16_torque_offset_value;
    volatile uint8_t ui8_hall_ref_angles[6];
    volatile uint8_t ui8_hall_offsets[6];
} struct_tsdz_cfg;

#pragma pack(1)
typedef struct _tsdz_status {
    volatile uint8_t ui8_riding_mode; // bit 7 Street Mode enabled flag
    volatile uint8_t ui8_assist_level;
    volatile uint16_t ui16_wheel_speed_x10;
    volatile uint8_t ui8_pedal_cadence_RPM;
    volatile int16_t i16_motor_temperaturex10;
    volatile uint16_t ui16_pedal_power_x10;
    volatile uint16_t ui16_battery_voltage_x1000;
    volatile uint8_t ui8_battery_current_x10;
    volatile uint8_t ui8_system_state;
    volatile uint16_t ui16_battery_wh;
    volatile uint8_t ui8_rxc_errors;
    volatile uint8_t ui8_rxl_errors;
    volatile uint8_t ui8_status3;
    volatile uint8_t ui8_status4;
    volatile uint8_t ui8_status5;
} struct_tsdz_status;

#pragma pack(1)
typedef struct _tsdz_debug {
    volatile uint8_t ui8_adc_throttle;
    volatile uint8_t ui8_throttle;
    volatile uint16_t ui16_adc_pedal_torque_sensor;
    volatile uint8_t ui8_duty_cycle;
    volatile uint16_t ui16_motor_speed_erps;
    volatile uint8_t ui8_foc_angle;
    volatile uint16_t ui16_pedal_torque_x100;
    volatile uint8_t ui8_fw_hall_cnt_offset;
    volatile int16_t i16_pcb_temperaturex10;
    volatile uint8_t ui8_debugFlags;
    volatile uint8_t ui8_debug1;
    volatile uint8_t ui8_debug2;
    volatile uint8_t ui8_debug3;
    volatile uint8_t ui8_debug4;
    volatile uint8_t ui8_debug5;
    volatile uint8_t ui8_debug6;
} struct_tsdz_debug;

#pragma pack(1)
typedef struct _tsdz_hall {
    volatile uint8_t  ui8_cmd;
    volatile uint16_t ui16_hall_1;
    volatile uint16_t ui16_hall_2;
    volatile uint16_t ui16_hall_3;
    volatile uint16_t ui16_hall_4;
    volatile uint16_t ui16_hall_5;
    volatile uint16_t ui16_hall_6;
} struct_tsdz_hall;

extern uint8_t bike_locked;

extern const uint32_t bt_passkey;
extern struct_esp32_cfg esp32_cfg;
extern struct_tsdz_cfg tsdz_cfg;
extern struct_tsdz_status tsdz_status;
extern struct_tsdz_debug tsdz_debug;

// Hall Calibration data message
extern uint8_t hall_calib_data_valid;
extern struct_tsdz_hall tsdz_hall;

extern uint8_t stm8_fw_version;
extern uint32_t ui32_wh_x10_offset;
extern uint32_t ui32_wh_x10;
extern uint32_t wheel_revolutions;
extern uint16_t crank_revolutions;
extern volatile uint8_t ui8_app_street_mode;
extern volatile uint8_t ui8_app_assist_mode;
extern volatile uint8_t ui8_app_assist_parameter;
extern volatile uint8_t ui8_app_rotor_angle_adj;

void tsdz_data_update();
void processLcdMessage(const uint8_t lcd_oem_message[]);
void getLCDMessage(uint8_t ct_oem_message[]);
void processControllerMessage(const uint8_t ct_os_message[]);
void getControllerMessage(uint8_t lcd_os_message[]);
int tsdz_update_cfg(struct_tsdz_cfg *new_cfg);
void tsdz_data_disconnect_actions();

#endif /* MAIN_TSDZ_DATA_H_ */
