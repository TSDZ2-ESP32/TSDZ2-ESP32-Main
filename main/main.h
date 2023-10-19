/*
 * main.h
 *
 *  Created on: 12 gen 2021
 *      Author: SO000228
 */

#ifndef MAIN_MAIN_H_
#define MAIN_MAIN_H_

// OSF optional ADC function
#define NOT_IN_USE                                0
#define TEMPERATURE_CONTROL                       1
#define THROTTLE_CONTROL                          2

// OSF riding modes
#define OFF_MODE                                0
#define POWER_ASSIST_MODE                       1
#define TORQUE_ASSIST_MODE                      2
#define CADENCE_ASSIST_MODE                     3
#define eMTB_ASSIST_MODE                        4
#define WALK_ASSIST_MODE                        5
#define CRUISE_MODE                             6
#define MOTOR_CALIBRATION_MODE                  7


// Codes of tsdz_status.ui8_controller_system_state
// The bits 0-5 contain the motors error code (0 to 31)
// Bit 6 contains the brake state (0:off, 1:on)
// Bits 7-8 contain the communication status with LCD and Controller (0:ok, 1:fail)
#define CONTROLLER_ERROR_MASK                     0x0f
#define NO_ERROR                                  0
#define ERROR_MOTOR_BLOCKED                       1
#define ERROR_TORQUE_SENSOR                       2
#define ERROR_BATTERY_OVERCURRENT                 7
#define ERROR_OVERVOLTAGE                         8
#define ERROR_TEMPERATURE_LIMIT                   9
#define ERROR_TEMPERATURE_MAX                     10
#define ERROR_COMMUNICATION_MASK                  0xc0 // Mask bits for Communication errors
#define ERROR_CONTROLLER_COMMUNICATION            0x80 // Bit 7 set if controller communication is missing
#define ERROR_LCD_COMMUNICATION                   0x40 // Bit 6 set if lcd communication is missing
#define BRAKE_STATE_BIT                           0x20 // Bit 5 set if brake is On
#define ERROR_CONTROLLER_UART_BIT                 0x10 // Bit 4 set if the Controller has reception problems

// OEM Display error codes
// N.B.: E01, E05, E07, E09 are not available on XH18 display
#define OEM_NO_ERROR                            0
#define OEM_ERROR_TORQUE_SENSOR                 2 // E02
#define OEM_ERROR_CONTROLLER_FAILURE            3 // E03
#define OEM_ERROR_MOTOR_BLOCKED                 4 // E04
#define OEM_ERROR_OVERTEMPERATURE               6 // E06
#define OEM_ERROR_OVERVOLTAGE                   8 // E08

#define BATTERY_OVERVOLTAGE             1
#define BATTERY_UNDERVOLTAGE            2

#define MIN_MSG_SEC                     1 // (1 notification/sec)
#define MAX_MSG_SEC                     5 // (5 notification/sec)
#define DEFAULT_MSG_SEC                 3 // (2 notification/sec)
#define MIN_DS18B20_PIN                 3
#define MAX_DS18B20_PIN                 31
#define DEFAULT_DS18B20_PIN             4

#define STREET_MODE_LCD_MASTER      0
#define STREET_MODE_FORCE_OFF       1
#define STREET_MODE_FORCE_ON        2

#define APP_ASSIST_MODE_LCD_MASTER      0
#define APP_ASSIST_MODE_FORCE_POWER     1
#define APP_ASSIST_MODE_FORCE_EMTB      2
#define APP_ASSIST_MODE_FORCE_TORQUE    3
#define APP_ASSIST_MODE_FORCE_CADENCE   4
#define APP_ASSIST_MODE_MOTOR_CALIB     5

#define WALK_ASSIST_THRESHOLD_SPEED_X10           80 // 8.0 Km/h
#define CRUISE_THRESHOLD_SPEED_X10                90 // 9.0 Km/h

#endif /* MAIN_MAIN_H_ */
