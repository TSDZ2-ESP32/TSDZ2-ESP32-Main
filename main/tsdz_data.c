/*
 * tsdz_data.c
 *
 *  Created on: 2 set 2019
 *      Author: Max
 */
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "main.h"
#include "tsdz_data.h"
#include "tsdz_commands.h"
#include "tsdz_nvs.h"
#include "tsdz_uart.h"
#include "tsdz_utils.h"
#include "tsdz_ota_stm8.h"

static const char *TAG = "tsdz_data";

// add "target_link_libraries(${COMPONENT_TARGET} "-u bt_passkey")"to CMakeLists.txt to ensure that the parameter
// is located correctly after the header at the beginning of the bin app image file
const __attribute__((section(".rodata_custom_desc"))) uint32_t bt_passkey = CONFIG_BT_PIN;


// System Configuration parameters
// Readed from NVS at startup
struct_esp32_cfg esp32_cfg = {
    .msg_sec = DEFAULT_MSG_SEC,
    .ds18b20_pin = DEFAULT_DS18B20_PIN,
    .log_level = 3,
    .lock_enabled = 0
};

struct_tsdz_data tsdz_data = {
    .ui8_riding_mode = OFF_MODE,
    .ui8_assist_level = 0,
    .ui8_system_state = 0,
    .ui16_wheel_speed_x10 = 0,
    .ui8_pedal_cadence_RPM = 0,
    .ui16_pedal_torque_x100 = 0,
    .i16_motor_temperaturex10 = -999,
    .i16_pcb_temperaturex10 = -999,
    .ui16_battery_voltage_x1000 = 0,
    .ui8_battery_current_x10 = 0,
    .ui16_battery_wh = 0,
    .ui8_adc_throttle = 0,
    .ui8_throttle = 0,
    .ui16_adc_pedal_torque_sensor = 0,
    .ui8_duty_cycle = 0,
    .ui16_motor_speed_erps = 0,
    .ui8_foc_angle = 0,
    .ui8_fw_hall_cnt_offset = 0,
    .ui8_TorqueSmoothPct = 0,
    .ui8_TorqueAVG = 0,
    .ui8_TorqueMin = 0,
    .ui8_TorqueMax = 0,
    .ui8_rxc_errors = 0,
    .ui8_rxl_errors = 0,
    .ui8_debugFlags = 0,
    .ui8_debug1 = 0,
    .ui8_debug2 = 0,
    .ui8_debug3 = 0,
    .ui8_debug4 = 0,
    .ui8_debug5 = 0,
    .ui8_debug6 = 0
};

uint8_t hall_calib_data_valid = 0;
struct_tsdz_hall tsdz_hall = {
    .ui8_cmd = CMD_HALL_DATA,
    .ui16_hall_1 = 0,
    .ui16_hall_2 = 0,
    .ui16_hall_3 = 0,
    .ui16_hall_4 = 0,
    .ui16_hall_5 = 0,
    .ui16_hall_6 = 0
};

// Motor Configuration parameters (can be updated by the Android App)
// These are the initialization values stored into NVS at first startup.
// Then the values are overwritten at the startup with the values stored into NVS
struct_tsdz_cfg tsdz_cfg = {
    .ui8_foc_angle_multiplicator = 27,
    .ui8_motor_temperature_min_value_to_limit = 70,
    .ui8_motor_temperature_max_value_to_limit = 85,
    .ui8_motor_acceleration = 25,
    .ui8_hall_offset_adj = 0,
    .ui8_max_speed = 45,
    .ui8_street_max_speed = 25,
    .ui8_pedal_torque_per_10_bit_ADC_step_x100 = 67,
    .ui8_optional_ADC_function = 0,
    .ui8_assist_without_pedal_rotation_threshold = 0,
	.ui8_lights_configuration = 0,
    .ui16_wheel_perimeter = 2300,
    .ui8_cruise_mode_enabled = 0,
    .ui16_battery_voltage_reset_wh_counter_x10 = 416, // 41.6V
    .ui8_battery_max_current = 15,
    .ui8_target_max_battery_power_div25 = 20,
    .ui8_battery_cells_number = 10,
    .ui16_battery_pack_resistance_x1000 = 180,
    .ui16_battery_low_voltage_cut_off_x10 = 290,
    .ui8_li_io_cell_overvolt_x100 = 225, // mV*10-200 (225 -> 200+225 -> 425 -> 4.25V)
    .ui8_li_io_cell_full_bars_x100 = 200,
    .ui8_li_io_cell_one_bar_x100 = 130,
    .ui8_li_io_cell_empty_x100 = 100,
    .ui8_phase_angle_adj = 0,
    .ui8_street_mode_power_limit_enabled = 0,
    .ui8_street_mode_throttle_enabled = 0,
    .ui8_street_mode_power_limit_div25 = 10,
    .ui8_street_mode_speed_limit = 25,
    .ui8_esp32_temp_control = 0,
    .ui8_cadence_assist_level = {50,75,100,125}, // W/2: 100W, 150W, 200W, 250W
    .ui8_power_assist_level = {25,50,100,150}, // %/2: 50%, 100%, 200%, 300%
    .ui8_torque_assist_level = {15,40,65,90},
    .ui8_eMTB_assist_sensitivity = {6,10,14,18},
    .ui8_walk_assist_level = {40,55,70,85},
    .ui8_flags = 2, // bit 0: Torque offset fix en/dis, bit 1: field weakening en/dis, bit 2: torque smooth en/dis, bit 3: street mode on startup en/dis
    .ui16_torque_offset_value = 0,
	.ui8_hall_ref_angles = {217, 4, 47, 89, 132, 175}, // Array order: Hall states 6, 2, 3, 1, 5, 4
    .ui8_hall_offsets = {44, 23, 44, 23, 44, 23},       // Array order: Hall states 6, 2, 3, 1, 5, 4
    .ui8_torque_smooth_min = 6,
    .ui8_torque_smooth_max = 12
};

uint8_t stm8_fw_version = -1;
uint8_t bike_locked = 0;

// local OEM LCD values
static uint8_t ui8_oem_wheel_diameter;
static uint8_t ui8_oem_lights;

// local wh calculation variables
static uint16_t       ui16_battery_power_filtered_x10 = 0;
static uint32_t       ui32_wh_sum_x10 = 0;

static uint8_t ui8_message_ID = 0;
static uint8_t ui8_BatteryLevel = 0;
static uint8_t ui8_BatteryError = 0;

// global system variables
uint32_t            ui32_wh_x10 = 0;
uint32_t            ui32_wh_x10_offset = 0;
uint32_t            wheel_revolutions;
uint16_t            crank_revolutions;
volatile uint8_t    ui8_app_street_mode = STREET_MODE_LCD_MASTER;
volatile uint8_t    ui8_app_assist_mode = APP_ASSIST_MODE_LCD_MASTER;
volatile uint8_t    ui8_app_assist_parameter = 0;
volatile uint8_t    ui8_app_rotor_angle_adj = 0;


void update_battery();
void update_energy(void);
int validHallRefAngles(volatile uint8_t* values);

void tsdz_data_disconnect_actions() {
	if (ui8_app_assist_mode == APP_ASSIST_MODE_MOTOR_CALIB) {
		ui8_app_assist_mode = APP_ASSIST_MODE_LCD_MASTER;
		ESP_LOGI(TAG, "ui8_app_assist_mode = %d", ui8_app_assist_mode);
	}
}

// called every 100 ms
void tsdz_data_update() {
    static uint32_t lastUpdateValue;

    // calculate Wh consumption
    update_energy();

    // save Wh consumed every 5 Wh increment
    if ((lastUpdateValue/50) != (ui32_wh_x10/50)) {
        lastUpdateValue = ui32_wh_x10;
        tsdz_nvs_update_whOffset();
    }
}

void processLcdMessage(const uint8_t lcd_oem_message[]) {
    switch(lcd_oem_message[1] & 0x5E) {
        case OEM_ASSIST_LEVEL4:
            tsdz_data.ui8_assist_level = 4;
            break;
        case OEM_ASSIST_LEVEL3:
            tsdz_data.ui8_assist_level = 3;
            break;
        case OEM_ASSIST_LEVEL2:
            tsdz_data.ui8_assist_level = 2;
            break;
        case OEM_ASSIST_LEVEL1:
            tsdz_data.ui8_assist_level = 1;
            break;
        case OEM_ASSIST_LEVEL0:
            tsdz_data.ui8_assist_level = 0;
            break;
    }

    // wheel diameter
    ui8_oem_wheel_diameter = lcd_oem_message[3];

    // check if long hold down button
    if (lcd_oem_message[1] & 0x20) {
        if ((tsdz_data.ui16_wheel_speed_x10 < WALK_ASSIST_THRESHOLD_SPEED_X10) &&
                  (tsdz_data.ui8_riding_mode != CRUISE_MODE)) {
            tsdz_data.ui8_riding_mode = WALK_ASSIST_MODE;
            goto skip;
        } else if ((tsdz_cfg.ui8_cruise_mode_enabled) &&
                   (tsdz_data.ui16_wheel_speed_x10 > CRUISE_THRESHOLD_SPEED_X10) &&
                   (tsdz_data.ui8_riding_mode != WALK_ASSIST_MODE)) {
            tsdz_data.ui8_riding_mode = CRUISE_MODE;
            goto skip;
        }
    }

    // check the riding mode
    switch (ui8_app_assist_mode) {
        case APP_ASSIST_MODE_FORCE_POWER:
            tsdz_data.ui8_riding_mode = POWER_ASSIST_MODE;
            break;
        case APP_ASSIST_MODE_FORCE_EMTB:
            tsdz_data.ui8_riding_mode = eMTB_ASSIST_MODE;
            break;
        case APP_ASSIST_MODE_FORCE_TORQUE:
            tsdz_data.ui8_riding_mode = TORQUE_ASSIST_MODE;
            break;
        case APP_ASSIST_MODE_FORCE_CADENCE:
            tsdz_data.ui8_riding_mode = CADENCE_ASSIST_MODE;
            break;
        default:
            switch (ui8_oem_wheel_diameter) {
                case 27:
                    tsdz_data.ui8_riding_mode = TORQUE_ASSIST_MODE;
                    break;
                case 28:
                    tsdz_data.ui8_riding_mode = CADENCE_ASSIST_MODE;
                    break;
                case 29:
                    tsdz_data.ui8_riding_mode = eMTB_ASSIST_MODE;
                    break;
                default:
                    tsdz_data.ui8_riding_mode = POWER_ASSIST_MODE;
                    break;
            }
            break;
    }


    skip:

    // light status
    ui8_oem_lights = lcd_oem_message[1] & 0x01;

    // VLCD5: max speed
    // 25Km/h ON  : sends 26 (Km/h)
    // 25Km/h OFF : sends 45 (Km/h)
    switch (ui8_app_street_mode) {
        case STREET_MODE_FORCE_OFF:
            tsdz_data.ui8_riding_mode &= 0x7f; // clear Street mode flag
            break;
        case STREET_MODE_FORCE_ON:
            tsdz_data.ui8_riding_mode |= 0x80; // set Street mode flag
            break;
        case STREET_MODE_LCD_MASTER:
            if (lcd_oem_message[5] > 26)
                tsdz_data.ui8_riding_mode &= 0x7f; // clear Street mode flag
            else
                tsdz_data.ui8_riding_mode |= 0x80; // set Street mode flag
            break;
    }
}

void getLCDMessage(uint8_t ct_oem_message[]) {

    // used to set the Temperature Error blink speed according to the temperature
    static uint8_t temperatureError = 0;
    static uint8_t temperatureErrorOn = 0;
    static uint8_t temperatureErrorCounter = 0;

    uint8_t  ui8_working_status = 0;
    uint8_t  ui8_error_code = OEM_NO_ERROR;

    // start up byte
    ct_oem_message[0] = CT_MSG_ID;

    // battery level
    ct_oem_message[1] = ui8_BatteryLevel;

    // undervoltage flag
    if (ui8_BatteryError == BATTERY_UNDERVOLTAGE) {
        ui8_working_status |= 0x01;
    }
    // hold display on flag
    if ((tsdz_data.ui8_duty_cycle > 10) || (tsdz_data.ui16_wheel_speed_x10 > 10))
        ui8_working_status |= 0x04;
    ct_oem_message[2] = ui8_working_status;

    // reserved
    ct_oem_message[3] = 0x46;
    ct_oem_message[4] = 0x46;

    // system status
    if(((tsdz_data.ui8_system_state & CONTROLLER_ERROR_MASK) == ERROR_MOTOR_BLOCKED)
            || ((tsdz_data.ui8_system_state & CONTROLLER_ERROR_MASK) == ERROR_BATTERY_OVERCURRENT)
            || (bike_locked != 0))
        ui8_error_code = OEM_ERROR_MOTOR_BLOCKED;
    else if ((tsdz_data.ui8_system_state & CONTROLLER_ERROR_MASK) == ERROR_TORQUE_SENSOR)
        ui8_error_code = OEM_ERROR_TORQUE_SENSOR;
    else if (tsdz_data.ui8_system_state & ERROR_CONTROLLER_COMMUNICATION)
        ui8_error_code = OEM_ERROR_CONTROLLER_FAILURE;
    else if (ui8_BatteryError == BATTERY_OVERVOLTAGE)
        ui8_error_code = OEM_ERROR_OVERVOLTAGE;
    if (tsdz_cfg.ui8_esp32_temp_control || tsdz_cfg.ui8_optional_ADC_function == TEMPERATURE_CONTROL) {
        if (tsdz_data.i16_motor_temperaturex10 >= tsdz_cfg.ui8_motor_temperature_max_value_to_limit*10) {
            // Temperature Error fixed
            temperatureError = 1;
            temperatureErrorOn = 1;
            temperatureErrorCounter = 0;
            ui8_error_code = OEM_ERROR_OVERTEMPERATURE;
        } else if (tsdz_data.i16_motor_temperaturex10 >= tsdz_cfg.ui8_motor_temperature_min_value_to_limit*10) {
            // Temperature Error blinking slow to fast
            if (temperatureError) {
                temperatureErrorCounter++;
                int counter = map(tsdz_data.i16_motor_temperaturex10,
                        tsdz_cfg.ui8_motor_temperature_min_value_to_limit*10,
                        tsdz_cfg.ui8_motor_temperature_max_value_to_limit*10,
                        15,
                        2);
                if (temperatureErrorCounter > counter) {
                    temperatureErrorCounter = 0;
                    temperatureErrorOn = !temperatureErrorOn;
                }
                if (temperatureErrorOn)
                    ui8_error_code = OEM_ERROR_OVERTEMPERATURE;
            } else {
                temperatureError = 1;
                temperatureErrorOn = 1;
                temperatureErrorCounter = 0;
                ui8_error_code = OEM_ERROR_OVERTEMPERATURE;
            }
        } else {
            temperatureError = 0;
        }
    }
    ct_oem_message[5] = ui8_error_code;

    // wheel speed
    // wait 4 sec from startup to send the speed. Max num of ticks for OEM LCD is 0x0707
    if (tsdz_data.ui16_wheel_speed_x10 == 0 || (xTaskGetTickCount() < pdMS_TO_TICKS(4000))) {
        ct_oem_message[6] = 0x07;
        ct_oem_message[7] = 0x07;
    } else {
        // calculate the nr. of clock ticks of OEM LCD for one OEM wheel revolution (1 tick is 1/500 sec)
        // (3600/(ui16_wheel_speed_x10 * 100000)) * (ui8_oem_wheel_diameter * 25.4 * pi) *    500)
        //              (sec/mm)                  *          (mm/rev)                    * (ticks/sec) = ticks/rev
        // and then using integer division round.
        // result = (dividend + (divisor / 2)) / divisor   (Eg. 16 / 10 -> 2  and  14 / 10 -> 1)
        uint32_t tmp = ((ui8_oem_wheel_diameter * 1436336) + (tsdz_data.ui16_wheel_speed_x10 * 500)) / (tsdz_data.ui16_wheel_speed_x10 * 1000);
        // limit max value to 0x0707 (0 Km/h for OEM LCD)
        if (tmp > 0x0707)
            tmp = 0x0707;
        ct_oem_message[6] = (uint8_t) tmp;
        ct_oem_message[7] = (uint8_t) (tmp >> 8);
    }

    ct_oem_message[8] = crc8(ct_oem_message, CT_OEM_MSG_BYTES - 1);
}

void processControllerMessage(const uint8_t ct_os_message[]) {
    // update System Status
    // Bit 3: brake state
    // Bit 4-7: Controller Status
    if (ct_os_message[1] & 0x08)
        tsdz_data.ui8_system_state |= BRAKE_STATE_BIT;
    else
        tsdz_data.ui8_system_state &= ~BRAKE_STATE_BIT;
    // Controller status
    tsdz_data.ui8_system_state = (tsdz_data.ui8_system_state & (~CONTROLLER_ERROR_MASK)) | ((ct_os_message[1] >> 4) & CONTROLLER_ERROR_MASK);

    // battery current x10
    tsdz_data.ui8_battery_current_x10 = ct_os_message[4];

    // battery voltage
    // add voltage dropout due to battery internal resistance
    uint32_t tmp = (uint32_t)tsdz_data.ui8_battery_current_x10 * (uint32_t)tsdz_cfg.ui16_battery_pack_resistance_x1000 / (uint32_t)10;
    tsdz_data.ui16_battery_voltage_x1000 = (((uint16_t) ct_os_message[3]) << 8) + ((uint16_t) ct_os_message[2]) + (uint16_t)tmp;

    // calculate battery Power filtered for Wh calculation
    uint32_t ui32_battery_power_temp_x10 = ((uint32_t) tsdz_data.ui16_battery_voltage_x1000 * tsdz_data.ui8_battery_current_x10) / 1000;
    ui16_battery_power_filtered_x10 = filter(ui32_battery_power_temp_x10, ui16_battery_power_filtered_x10, 72);

    // wheel speed
    // bit 0..10; Speedx10 (Km/h)
    // bit 11..15; FOC Angle
    tsdz_data.ui16_wheel_speed_x10 = (((uint16_t) (ct_os_message[6] & 0x07)) << 8) + ((uint16_t) ct_os_message[5]);
    // FOC angle
    tsdz_data.ui8_foc_angle = ct_os_message[6] >> 3;

    // pedal cadence
    tsdz_data.ui8_pedal_cadence_RPM = ct_os_message[7];

    // pedal torque x100
    tsdz_data.ui16_pedal_torque_x100 = (((uint16_t) ct_os_message[9]) << 8) + ((uint16_t) ct_os_message[8]);
    // Human power x10 calculated from torque and cadence
    //tsdz_data.ui16_pedal_power_x10 = ((uint32_t)tsdz_data.ui16_pedal_torque_x100 * tsdz_data.ui8_pedal_cadence_RPM) / 96;

    // motor temperature
    if (!tsdz_cfg.ui8_esp32_temp_control && (tsdz_cfg.ui8_optional_ADC_function == TEMPERATURE_CONTROL))
        tsdz_data.i16_motor_temperaturex10 = ct_os_message[10]*10;

    // Calculate Battery Level and check for Battery Over/Under-voltage
    // Update System Status for Battery errors and Temperature
    update_battery();
    if ((tsdz_data.ui8_system_state & CONTROLLER_ERROR_MASK) == NO_ERROR) {
        if (ui8_BatteryError == BATTERY_OVERVOLTAGE)
            tsdz_data.ui8_system_state |= ERROR_OVERVOLTAGE;
        else if (tsdz_cfg.ui8_esp32_temp_control || tsdz_cfg.ui8_optional_ADC_function == TEMPERATURE_CONTROL) {
            if (tsdz_data.i16_motor_temperaturex10 >= tsdz_cfg.ui8_motor_temperature_max_value_to_limit*10)
                tsdz_data.ui8_system_state |= ERROR_TEMPERATURE_MAX;
            else if (tsdz_data.i16_motor_temperaturex10 >= tsdz_cfg.ui8_motor_temperature_min_value_to_limit*10)
                tsdz_data.ui8_system_state |= ERROR_TEMPERATURE_LIMIT;
        }
    }

    // PWM duty_cycle
    tsdz_data.ui8_duty_cycle = ct_os_message[11];

    // motor speed in ERPS
    tsdz_data.ui16_motor_speed_erps = (((uint16_t) ct_os_message[13]) << 8) + ((uint16_t) ct_os_message[12]);

    // Field Weakening Hall counter offset
    tsdz_data.ui8_fw_hall_cnt_offset = ct_os_message[14];

    switch (ct_os_message[1] & 0x03) {
        case 0: // Normal Message
            hall_calib_data_valid = 0;

            stm8_fw_version = ct_os_message[15];

            // ADC pedal torque
            tsdz_data.ui16_adc_pedal_torque_sensor = (((uint16_t) ct_os_message[17]) << 8) + ((uint16_t) ct_os_message[16]);

            // value from optional ADC channel
            tsdz_data.ui8_adc_throttle = ct_os_message[18];

            // throttle or temperature control mapped from 0 to 255
            // TODO: this field is useless and is not used in the Android app
            tsdz_data.ui8_throttle = ct_os_message[19];

            // Torque Smooth % of last 2 sec
            tsdz_data.ui8_TorqueSmoothPct = ct_os_message[20];

            ESP_LOGI(TAG, "msg[21]=%d", ct_os_message[21]);
            switch (ct_os_message[21]) {
                case 0: // Wheel and crank revolutions
                    // wheel revolutions
                    wheel_revolutions = (((uint32_t) ct_os_message[24]) << 16)
                            + (((uint32_t) ct_os_message[23]) << 8)
                            + ((uint32_t) ct_os_message[22]);

                    // crank revolutions
                    crank_revolutions = (((uint16_t) ct_os_message[26]) << 8) + ((uint16_t) ct_os_message[25]);
                    break;
                case 1: // pedal revolution torque values
                	tsdz_data.ui8_debugFlags = 0;
                    tsdz_data.ui8_TorqueAVG = ct_os_message[22];
                    tsdz_data.ui8_TorqueMin = ct_os_message[23];
                    tsdz_data.ui8_TorqueMax = ct_os_message[24];
                    break;
                case 2: // PWM Time debug data
                    tsdz_data.ui8_debugFlags = 0x20;
                    tsdz_data.ui8_debug1 = ct_os_message[22];
                    tsdz_data.ui8_debug2 = ct_os_message[23];
                    tsdz_data.ui8_debug3 = ct_os_message[24];
                    tsdz_data.ui8_debug4 = ct_os_message[25];
                    tsdz_data.ui8_debug5 = ct_os_message[26];
                    break;
                case 3: // Hall errors debug data
                    tsdz_data.ui8_debugFlags = 0x40;
                    tsdz_data.ui8_debug1 = ct_os_message[22];
                    tsdz_data.ui8_debug2 = ct_os_message[23];
                    break;
            }
            ESP_LOGI(TAG, "dbg=%d", tsdz_data.ui8_debugFlags);
            break;
        case 1: // Motor Hall Calibration Message
            tsdz_hall.ui16_hall_1 = (((uint16_t) ct_os_message[16]) << 8) + ((uint16_t) ct_os_message[15]);
            tsdz_hall.ui16_hall_2 = (((uint16_t) ct_os_message[18]) << 8) + ((uint16_t) ct_os_message[17]);
            tsdz_hall.ui16_hall_3 = (((uint16_t) ct_os_message[20]) << 8) + ((uint16_t) ct_os_message[19]);
            tsdz_hall.ui16_hall_4 = (((uint16_t) ct_os_message[22]) << 8) + ((uint16_t) ct_os_message[21]);
            tsdz_hall.ui16_hall_5 = (((uint16_t) ct_os_message[24]) << 8) + ((uint16_t) ct_os_message[23]);
            tsdz_hall.ui16_hall_6 = (((uint16_t) ct_os_message[26]) << 8) + ((uint16_t) ct_os_message[25]);
            hall_calib_data_valid = 1;
            break;
    }
}

void getControllerMessage(uint8_t lcd_os_message[]) {

    // start up byte
    lcd_os_message[0] = LCD_MSG_ID;

    // message ID
    lcd_os_message[1] = ui8_message_ID;

    // riding mode
    // if bike locked reset to OFF_Mode
    if (bike_locked)
        lcd_os_message[2] = OFF_MODE;
    else if (ui8_app_assist_mode == APP_ASSIST_MODE_MOTOR_CALIB)
        lcd_os_message[2] = MOTOR_CALIBRATION_MODE;
    else if (tsdz_data.ui8_assist_level == 0)
        lcd_os_message[2] = OFF_MODE;
    else
        lcd_os_message[2] = tsdz_data.ui8_riding_mode & 0x7f; // mask bit 7 used as Street mode flag

    // riding mode parameter
    switch (lcd_os_message[2]) {
        case POWER_ASSIST_MODE:
            if (tsdz_data.ui8_assist_level > 0) {
                lcd_os_message[3] = tsdz_cfg.ui8_power_assist_level[tsdz_data.ui8_assist_level - 1];
            } else {
                lcd_os_message[3] = 0;
            }
            break;
        case CADENCE_ASSIST_MODE:
            if (tsdz_data.ui8_assist_level > 0) {
                lcd_os_message[3] = tsdz_cfg.ui8_cadence_assist_level[tsdz_data.ui8_assist_level - 1];
            } else {
                lcd_os_message[3] = 0;
            }
            break;
        case TORQUE_ASSIST_MODE:
            if (tsdz_data.ui8_assist_level > 0) {
                lcd_os_message[3] = tsdz_cfg.ui8_torque_assist_level[tsdz_data.ui8_assist_level - 1];
            } else {
                lcd_os_message[3] = 0;
            }
            break;
        case eMTB_ASSIST_MODE:
            if (tsdz_data.ui8_assist_level > 0) {
                lcd_os_message[3] = tsdz_cfg.ui8_eMTB_assist_sensitivity[tsdz_data.ui8_assist_level - 1];
            } else {
                lcd_os_message[3] = 0;
            }
            break;
        case WALK_ASSIST_MODE:
            if (tsdz_data.ui8_assist_level > 0) {
                lcd_os_message[3] = tsdz_cfg.ui8_walk_assist_level[tsdz_data.ui8_assist_level - 1];
            } else {
                lcd_os_message[3] = 0;
            }
            break;
        case MOTOR_CALIBRATION_MODE:
            lcd_os_message[3] = ui8_app_assist_parameter;
            break;
        case CRUISE_MODE:
        case OFF_MODE:
        default:
            lcd_os_message[3] = 0;
            break;
    }

    // set lights state (bit 7) and light configuration (bit 0-6)
    lcd_os_message[4] = tsdz_cfg.ui8_lights_configuration & 0x7f;
    if (ui8_oem_lights)
        lcd_os_message[4] |= 0x80;

    switch (ui8_message_ID) {
        case 0:
            // battery low voltage cut off x10
            lcd_os_message[5] = (uint8_t) (tsdz_cfg.ui16_battery_low_voltage_cut_off_x10 & 0xff);
            lcd_os_message[6] = (uint8_t) (tsdz_cfg.ui16_battery_low_voltage_cut_off_x10 >> 8);

            // max battery current in amps
            if (tsdz_cfg.ui8_esp32_temp_control) {
                lcd_os_message[7] = map((uint32_t) tsdz_data.i16_motor_temperaturex10,
                        (uint32_t) tsdz_cfg.ui8_motor_temperature_min_value_to_limit * 10,
                        (uint32_t) tsdz_cfg.ui8_motor_temperature_max_value_to_limit * 10,
                        (uint32_t) tsdz_cfg.ui8_battery_max_current,
                        (uint32_t) 0);
            } else
                lcd_os_message[7] = tsdz_cfg.ui8_battery_max_current;

            // battery power limit (based on Street Mode enable status)
            if ((tsdz_data.ui8_riding_mode & 0x80) && tsdz_cfg.ui8_street_mode_power_limit_enabled) {
                lcd_os_message[8] = tsdz_cfg.ui8_street_mode_power_limit_div25;
            } else {
                lcd_os_message[8] = tsdz_cfg.ui8_target_max_battery_power_div25;
            }

            // set motor inductance
            lcd_os_message[9] = tsdz_cfg.ui8_foc_angle_multiplicator;

            // motor acceleration adjustment
            lcd_os_message[10] = tsdz_cfg.ui8_motor_acceleration;
            break;

        case 1:
            // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            #ifdef DEADTIME_TEST
            if (lcd_os_message[2] == MOTOR_CALIBRATION_MODE)
                lcd_os_message[5] = ui8_app_rotor_angle_adj;
            else
            #endif
            // free for future use
                lcd_os_message[5] = 0;

            // lcd_os_message[5]:
            // Bit 7: STM8 OTA Reboot
            // Bit 0: Field Weakening enable flag
            lcd_os_message[6] = 0;
            if (stm8_ota_status > 0) {
                lcd_os_message[6] |= 0x80;
                stm8_ota_status++;
            }
            if (tsdz_cfg.ui8_flags & 0x02)
                lcd_os_message[6] |= 1;

            // wheel perimeter
            lcd_os_message[7] = (uint8_t) (tsdz_cfg.ui16_wheel_perimeter & 0xff);
            lcd_os_message[8] = (uint8_t) (tsdz_cfg.ui16_wheel_perimeter >> 8);

            // wheel max speed (based on Street Mode enable status)
            if (tsdz_data.ui8_riding_mode & 0x80)
                lcd_os_message[9] = tsdz_cfg.ui8_street_max_speed;
            else
                lcd_os_message[9] = tsdz_cfg.ui8_max_speed;

            // assist without pedal rotation threshold
            lcd_os_message[10] = tsdz_cfg.ui8_assist_without_pedal_rotation_threshold;
            break;
        case 2:
            // optional ADC function, disable throttle if is disabled in Street Mode
            if ((tsdz_data.ui8_riding_mode & 0x80) && !tsdz_cfg.ui8_street_mode_throttle_enabled && tsdz_cfg.ui8_optional_ADC_function == THROTTLE_CONTROL) {
                lcd_os_message[5] = 0;
            } else {
                lcd_os_message[5] = tsdz_cfg.ui8_optional_ADC_function;
            }

            // motor over temperature min value limit
            lcd_os_message[6] = tsdz_cfg.ui8_motor_temperature_min_value_to_limit;
            // motor over temperature max value limit
            lcd_os_message[7] = tsdz_cfg.ui8_motor_temperature_max_value_to_limit;

            // Torque offset Fix
            lcd_os_message[8] = (uint8_t) (tsdz_cfg.ui16_torque_offset_value & 0xff);
            lcd_os_message[9] = (uint8_t) ((uint8_t)(tsdz_cfg.ui16_torque_offset_value >> 8) & 0x7f);
            // Set bit 15 if torque offset fix is enabled
            if (tsdz_cfg.ui8_flags & 0x01)
            	lcd_os_message[9] |= 0x80;

            // pedal torque ADC conversion factor
            lcd_os_message[10] = tsdz_cfg.ui8_pedal_torque_per_10_bit_ADC_step_x100;
            break;

        case 3:
        	if (validHallRefAngles(tsdz_cfg.ui8_hall_ref_angles)) {
        	    uint8_t adj;
                // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                #ifndef DEADTIME_TEST
        	    if (lcd_os_message[2] == MOTOR_CALIBRATION_MODE)
                    adj = ui8_app_rotor_angle_adj;
                else
                #endif
                    adj = tsdz_cfg.ui8_phase_angle_adj;
				lcd_os_message[5]  = tsdz_cfg.ui8_hall_ref_angles[0] + adj;
				lcd_os_message[6]  = tsdz_cfg.ui8_hall_ref_angles[1] + adj;
				lcd_os_message[7]  = tsdz_cfg.ui8_hall_ref_angles[2] + adj;
				lcd_os_message[8]  = tsdz_cfg.ui8_hall_ref_angles[3] + adj;
				lcd_os_message[9]  = tsdz_cfg.ui8_hall_ref_angles[4] + adj;
				lcd_os_message[10] = tsdz_cfg.ui8_hall_ref_angles[5] + adj;
        	} else {
				lcd_os_message[5] = 0;
				lcd_os_message[6] = 0;
				lcd_os_message[7] = 0;
				lcd_os_message[8] = 0;
				lcd_os_message[9] = 0;
				lcd_os_message[10] = 0;
        	}
            break;
        case 4:
            // Hall counter offset
            lcd_os_message[5]  = tsdz_cfg.ui8_hall_offsets[0] + tsdz_cfg.ui8_hall_offset_adj;
            lcd_os_message[6]  = tsdz_cfg.ui8_hall_offsets[1] + tsdz_cfg.ui8_hall_offset_adj;
            lcd_os_message[7]  = tsdz_cfg.ui8_hall_offsets[2] + tsdz_cfg.ui8_hall_offset_adj;
            lcd_os_message[8]  = tsdz_cfg.ui8_hall_offsets[3] + tsdz_cfg.ui8_hall_offset_adj;
            lcd_os_message[9]  = tsdz_cfg.ui8_hall_offsets[4] + tsdz_cfg.ui8_hall_offset_adj;
            lcd_os_message[10] = tsdz_cfg.ui8_hall_offsets[5] + tsdz_cfg.ui8_hall_offset_adj;
            break;
        case 5:
            if (tsdz_cfg.ui8_flags & 0x04)
                // Torque smooth enable
                lcd_os_message[5] = 1;
            else
                // Torque smooth disable
                lcd_os_message[5] = 0;
            lcd_os_message[6] = tsdz_cfg.ui8_torque_smooth_min;
            lcd_os_message[7] = tsdz_cfg.ui8_torque_smooth_max;
            break;
    }

    // prepare crc of the package
    uint16_t ui16_crc_tx = 0xffff;
    uint8_t ui8_i;

    for (ui8_i = 0; ui8_i < LCD_OS_MSG_BYTES-2; ui8_i++) {
        crc16 (lcd_os_message[ui8_i], &ui16_crc_tx);
    }

    lcd_os_message[LCD_OS_MSG_BYTES-2] = (uint8_t) (ui16_crc_tx & 0xff);
    lcd_os_message[LCD_OS_MSG_BYTES-1] = (uint8_t) (ui16_crc_tx >> 8) & 0xff;

    if (++ui8_message_ID > 5)
    	ui8_message_ID = 0;
}

int validHallRefAngles(volatile uint8_t* values) {
	int i;
	for (i=0; i<6; i++) {
		if (values[i] != 0)
			break;
	}
	// all values 0 -> OK (reset staus)
	if (i == 6) return 1;

	return ((values[0] >= 213)
			&& (values[0] <= 233)
			&& ((uint8_t)(values[1] - values[0]) >= 33)
			&& ((uint8_t)(values[1] - values[0]) <= 53)
			&& ((uint8_t)(values[2] - values[1]) >= 33)
			&& ((uint8_t)(values[2] - values[1]) <= 53)
			&& ((uint8_t)(values[3] - values[2]) >= 33)
			&& ((uint8_t)(values[3] - values[2]) <= 53)
			&& ((uint8_t)(values[4] - values[3]) >= 33)
			&& ((uint8_t)(values[4] - values[3]) <= 53)
			&& ((uint8_t)(values[5] - values[4]) >= 33)
			&& ((uint8_t)(values[5] - values[4]) <= 53)
			&& ((uint8_t)(values[0] - values[5]) >= 33)
			&& ((uint8_t)(values[0] - values[5]) <= 53));
}


int tsdz_update_cfg(struct_tsdz_cfg *new_cfg) {
    if (memcmp(new_cfg, &tsdz_cfg, sizeof(struct _tsdz_cfg)) == 0 ) {
        ESP_LOGI(TAG,"tsdz_update_cfg NO CHANGE");
        return 0;
    } else {
        if ((new_cfg->ui8_foc_angle_multiplicator > 50) ||
                (new_cfg->ui8_optional_ADC_function > 2) ||
                (new_cfg->ui8_battery_cells_number > 15) ||
                (new_cfg->ui8_cruise_mode_enabled > 1) ||
                (new_cfg->ui8_street_mode_power_limit_enabled > 1) ||
				!validHallRefAngles(new_cfg->ui8_hall_ref_angles) ||
				((new_cfg->ui8_phase_angle_adj > 10) && (new_cfg->ui8_phase_angle_adj < 246)) ||
                ((new_cfg->ui8_hall_offset_adj > 10) && (new_cfg->ui8_hall_offset_adj < 246))) {
            ESP_LOGI(TAG,"tsdz_update_cfg VALUES OUT OF RANGE");
            return 1;
        }
        memcpy(&tsdz_cfg, new_cfg, sizeof(struct _tsdz_cfg));
        if (tsdz_nvs_update_cfg()) {
            ESP_LOGE(TAG,"tsdz_update_cfg Failed!");
            return 1;
        } else {
            ESP_LOGI(TAG,"tsdz_update_cfg OK");
            return 0;
        }
    }
}


void update_energy(void)
{
    static uint8_t ui8_wh_reset;

    // reset watt-hour value if battery voltage is over threshold set from user, but only do this once for every power on
    if (((tsdz_data.ui16_battery_voltage_x1000) > (tsdz_cfg.ui16_battery_voltage_reset_wh_counter_x10 * 100)) && (!ui8_wh_reset))
    {
        ui8_wh_reset = 1;
        ui32_wh_x10_offset = 0;
    }

    ui32_wh_sum_x10 += ui16_battery_power_filtered_x10;

    // calculate watt-hours since last full charge
    ui32_wh_x10 = ui32_wh_x10_offset + (ui32_wh_sum_x10 / 3600);
    tsdz_data.ui16_battery_wh = (uint16_t)(ui32_wh_x10 / 10);
}

// Calculate battery level between 0x00 and 0x0C and check battery under/over-voltage
void update_battery() {
    // if cell voltage is less than 2V -> overvoltage
    if ((tsdz_data.ui16_battery_voltage_x1000 / 10 / (uint16_t)tsdz_cfg.ui8_battery_cells_number) <= 200) {
        ui8_BatteryLevel = 0;
        ui8_BatteryError = BATTERY_UNDERVOLTAGE;
        return;
    }

    uint16_t ui16_cell_voltage_x100 = (tsdz_data.ui16_battery_voltage_x1000 / 10 / (uint16_t)tsdz_cfg.ui8_battery_cells_number)-200;

    ui8_BatteryError = NO_ERROR;
    if (ui16_cell_voltage_x100 >= tsdz_cfg.ui8_li_io_cell_overvolt_x100) {
        // level full + overvoltage
        ui8_BatteryError = BATTERY_OVERVOLTAGE;
        ui8_BatteryLevel = 0x0C;
    } else if (ui16_cell_voltage_x100 >= tsdz_cfg.ui8_li_io_cell_full_bars_x100) {
        // level full
        ui8_BatteryLevel = 0x0C;
    } else if (ui16_cell_voltage_x100 <= tsdz_cfg.ui8_li_io_cell_empty_x100) {
        // level 0 (1 bar blinking)
        ui8_BatteryError = BATTERY_UNDERVOLTAGE; // battery undervoltage
        ui8_BatteryLevel = 0x00;
    } else if (ui16_cell_voltage_x100 < tsdz_cfg.ui8_li_io_cell_one_bar_x100) {
        // level 1
        ui8_BatteryLevel = 0x00;
    } else {
        ui8_BatteryLevel = map(ui16_cell_voltage_x100,
                tsdz_cfg.ui8_li_io_cell_one_bar_x100,
                tsdz_cfg.ui8_li_io_cell_full_bars_x100,
                1,
                11);
    }
}
