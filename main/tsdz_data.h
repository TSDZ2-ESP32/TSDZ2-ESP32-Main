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



// OSF optional ADC function
#define NOT_IN_USE                                0
#define TEMPERATURE_CONTROL                       1
#define THROTTLE_CONTROL                          2

// OSF cadence sensor modes
#define STANDARD_MODE                             0
#define ADVANCED_MODE                             1
#define CALIBRATION_MODE                          2

// OSF riding modes
#define OFF_MODE                                  0
#define POWER_ASSIST_MODE                         1
#define TORQUE_ASSIST_MODE                        2
#define CADENCE_ASSIST_MODE                       3
#define eMTB_ASSIST_MODE                          4
#define WALK_ASSIST_MODE                          5
#define CRUISE_MODE                               6
#define CADENCE_SENSOR_CALIBRATION_MODE           7

// OSF error codes
#define NO_ERROR                                  0
#define ERROR_MOTOR_BLOCKED                       1
#define ERROR_TORQUE_SENSOR                       2
#define ERROR_BRAKE_APPLIED_DURING_POWER_ON       3  // currently not used
#define ERROR_THROTTLE_APPLIED_DURING_POWER_ON    4  // currently not used
#define ERROR_NO_SPEED_SENSOR_DETECTED            5  // currently not used
#define ERROR_LOW_CONTROLLER_VOLTAGE              6  // controller works with no less than 15 V so give error code if voltage is too low
#define ERROR_CADENCE_SENSOR_CALIBRATION          7
#define ERROR_OVERVOLTAGE                         8
#define ERROR_TEMPERATURE_LIMIT                   9
#define ERROR_TEMPERATURE_MAX                     10

#define WALK_ASSIST_THRESHOLD_SPEED_X10         80 // 8.0 Km/h
#define CRUISE_THRESHOLD_SPEED_X10                 90 // 9.0 Km/h

// OEM Display Status byte masks
#define OEM_ASSIST_LEVEL0            0x10
#define OEM_ASSIST_LEVEL1            0x40
#define OEM_ASSIST_LEVEL2            0x02
#define OEM_ASSIST_LEVEL3            0x04
#define OEM_ASSIST_LEVEL4            0x08

// OEM Display error codes
// N.B.: E01, E05, E07 are not available on XH18 display
#define OEM_NO_ERROR                            0
#define OEM_ERROR_TORQUE_SENSOR                    2 // E02
#define OEM_ERROR_CADENCE_SENSOR_CALIBRATION    3 // E03
#define OEM_ERROR_MOTOR_BLOCKED                    4 // E04
#define OEM_ERROR_OVERTEMPERATURE                6 // E06
#define OEM_ERROR_OVERVOLTAGE                    8 // E08


#define BATTERY_OVERVOLTAGE                1
#define BATTERY_UNDERVOLTAGE            2

#define MIN_BT_UPDTAE_DELAY             20 // 200 ms (5 notification/sec)
#define MAX_BT_UPDATE_DELAY             100 // 1000 ms (1 notification/sec)
#define DEFAULT_BT_UPDATE_DELAY            25 // 250 ms (4 notification/sec)
#define MIN_DS18B20_PIN                     3
#define MAX_DS18B20_PIN                     31
#define DEFAULT_DS18B20_PIN                4

#pragma pack(1)
typedef struct _esp32_cfg
{
    volatile uint8_t bt_update_delay;
    volatile uint8_t ds18b20_pin;
    volatile uint8_t log_level;
} struct_esp32_cfg;

#pragma pack(1)
typedef struct _tsdz_cfg
{
    volatile uint8_t ui8_motor_type;
    volatile uint8_t ui8_motor_temperature_min_value_to_limit;
    volatile uint8_t ui8_motor_temperature_max_value_to_limit;
    volatile uint8_t ui8_motor_acceleration;
    volatile uint8_t ui8_cadence_sensor_mode;
    volatile uint16_t ui16_cadence_sensor_pulse_high_percentage_x10;
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
    volatile uint8_t ui8_street_mode_enabled;
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
} struct_tsdz_cfg;

#pragma pack(1)
typedef struct _tsdz_status
{
    volatile uint8_t ui8_riding_mode;
    volatile uint8_t ui8_assist_level;
    volatile uint16_t ui16_wheel_speed_x10;
    volatile uint8_t ui8_pedal_cadence_RPM;
    volatile int16_t i16_motor_temperaturex10;
    volatile uint16_t ui16_pedal_power_x10;
    volatile uint16_t ui16_battery_voltage_x1000;
    volatile uint8_t ui8_battery_current_x10;
    volatile uint8_t ui8_controller_system_state;
    volatile uint8_t ui8_braking;
    volatile uint16_t ui16_battery_wh;
    volatile uint8_t ui8_street_mode_enabled;
} struct_tsdz_status;

#pragma pack(1)
typedef struct _tsdz_debug
{
    volatile uint8_t ui8_adc_throttle;
    volatile uint8_t ui8_throttle;
    volatile uint16_t ui16_adc_pedal_torque_sensor;
    volatile uint8_t ui8_duty_cycle;
    volatile uint16_t ui16_motor_speed_erps;
    volatile uint8_t ui8_foc_angle;
    volatile uint16_t ui16_pedal_torque_x100;
    volatile uint16_t ui16_cadence_sensor_pulse_high_percentage_x10;
    volatile int16_t i16_pcb_temperaturex10;
} struct_tsdz_debug;


extern const uint32_t           bt_passkey;
extern struct_esp32_cfg         esp32_cfg;
extern struct_tsdz_cfg          tsdz_cfg;
extern struct_tsdz_status       tsdz_status;
extern struct_tsdz_debug        tsdz_debug;
extern uint8_t                  stm8_fw_version;
extern uint32_t                 ui32_wh_x10_offset;
extern uint32_t                 ui32_wh_x10;
extern volatile uint8_t         ui8_cadence_sensor_calibration;


void tsdz_data_update();
void processLcdMessage(const uint8_t lcd_oem_message[]);
void getLCDMessage(uint8_t ct_oem_message[]);
void processControllerMessage(const uint8_t ct_os_message[]);
void getControllerMessage(uint8_t lcd_os_message[]);
int tsdz_update_cfg(struct_tsdz_cfg *new_cfg);

#endif /* MAIN_TSDZ_DATA_H_ */
