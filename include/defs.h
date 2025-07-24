/**
 * @file defs.h
 * @brief necessary miscellaneous defines for various tasks and functions
 */

#ifndef DEFS_H
#define DEFS_H

#include <Arduino.h>

/*!< To select the telemetry transfer method used */
/*!< note: u can use wifi and xbee at the same time, so both of these handles can be set */
/*!< at the same time */
#define MQTT 0                                 /*!< set this to 1 if using MQTT for telemetry transfer. Set to 0 if you want to use beacons*/
#define TEST 0                                 /*!< set to 1 to enable test mode - allows data transmission even when disarmed */
#define XBEE 0                                 /*!< set to 1 if using XBEE for telemetry transfer */

extern bool use_beacon_mode;
#define BAUDRATE        115200
#define GPS_BAUD_RATE   9600                     /*!< baud rate for the GPS module. Change accordingly */
#define XBEE_BAUD_RATE  9600                    /*!< baud rate for the XBEE HP module. Change accordingly */

/* debug parameters for use during testing - set to 0 for production */
#define DEBUGGING 1                           /*!< allow debugging to terminal. Set to 0 pre flight to disable serial terminal printing and improve speed  */
#define LOG_TO_MEMORY 0                      /*!< allow data logging to memory. Set to 1 to log data to external flash memory. Must be set during flight */
#define DEBUG_TO_TERMINAL 1                 /*!< allow create task that print data to terminal. Set to 0 before flight  */

#if DEBUGGING
    #define debug(x) Serial.print(x)
    #define debugln(x) Serial.println(x)
    #define debugf(x, y) Serial.printf(x, y)
#else
    #define debug(x)
    #define debugln(x)
    #define debugf(x, y)
#endif // DEBUG

/* end of debug parameters */

/* MPU config parameters */
#define MPU_ADDRESS 0x68
#define MPU_ACCEL_RANGE 16
#define GYRO_RANGE 1000 /* 1000 deg/s */
#define WIRE_SEND_STOP 0

/* other pins */
#define GREEN_LED_PIN         15
#define RED_LED_PIN       4
#define BUZZER_PIN          33
#define SET_TEST_MODE_PIN    14     /*!< Pin to set the flight computer to TEST mode */
#define SET_RUN_MODE_PIN     13      /*!< Pin to set the flight computer to RUN mode */
#define SD_CS_PIN           26
#define REMOTE_SWITCH       27
#define DROGUE_PIN           12     /*!< Pin to fire the drogue chute ejection charge */
#define MAIN_CHUTE_EJECT_PIN 25     /*!< Pin to fire the main chute ejection charge */

// Ejection timing (milliseconds)
#define PYRO_CHARGE_TIME             5000   /*!< Time to keep drogue ejection pin HIGH (ms) */
#define MAIN_DESCENT_PYRO_CHARGE_TIME 5000  /*!< Time to keep main ejection pin HIGH (ms) */

// Ejection flags (set to 1 when deployed)
extern volatile uint8_t DROGUE_DEPLOY_FLAG;        /*!< Set to 1 when drogue is deployed */
extern volatile uint8_t MAIN_CHUTE_EJECT_FLAG;     /*!< Set to 1 when main chute is deployed */

/* timing constant */
#define SETUP_DELAY 300
#define TASK_DELAY 10

/*!< Flight data constants  */
#define ALTITUDE 1525.0 // altitude of iPIC building, JKUAT, Juja. TODO: Change to launch site altitude
#define LAUNCH_DETECTION_THRESHOLD 10         /*!< altitude in meters, above which we register that we have launched  */
#define LAUNCH_DETECTION_ALTITUDE_WINDOW 20  /*!< Window in meters where we register a launch */
#define APOGEE_DETECTION_THRESHOLD 3         /*!< value in meters for detecting apogee */
#define MAIN_EJECTION_HEIGHT 500            /*!< height to eject the main chute  */
#define DROGUE_EJECTION_HEIGHT  1000             /*!< height to eject the drogue chute - ideally it should be at apogee  */
#define SEA_LEVEL_PRESSURE 101325            /*!< sea level pressure to be used for altitude calculations */
#define BASE_ALTITUDE 1417                   /*!< this value is the altitude at rocket launch site - adjust accordingly */

/*!<  tasks constants */
#define STACK_SIZE 2048                     /*!< task stack size in words - increased from 1024 to prevent stack overflows */
#define ALTIMETER_QUEUE_LENGTH 10           /*!< length of the altimeter queue */
#define GYROSCOPE_QUEUE_LENGTH 10           /*!< length of the gyroscope queue */
#define GPS_QUEUE_LENGTH 24                 /*!< length of the gps queue */
#define TELEMETRY_DATA_QUEUE_LENGTH  10     /*!< length of the telemetry data queue */
#define FILTERED_DATA_QUEUE_LENGTH 10       /*!< length of the filtered data queue */
#define FLIGHT_STATES_QUEUE_LENGTH 1        /*!< length of the flight states queue */
#define CONSUME_TASK_DELAY    100           /*!< Task delay in ms - increased to prevent watchdog timeouts */

/* MQTT constants */
// MQTT server IP and port are now configured dynamically via WiFiManager
// Default values: IP="192.168.100.248", Port=1883
// Use wifi_config.getBaseStationIP() and wifi_config.getMQTTPort() to access current values
const char MQTT_TELEMETRY_TOPIC[30] = "n4/flight-computer-1";             /* make this topic unique to every rocket */
const char MQTT_ARMING_TOPIC[30] = "n4/commands";             /* make this topic unique to every rocket */

// Legacy defines - kept for backward compatibility but values are now dynamic
#define MQTT_PORT 1883                              /*!< Default MQTT broker port - actual port configured via WiFiManager */

// #define BROKER_IP_ADDRESS_LENGTH    20      /*!< length of broker ip address string */
// #define MQTT_TOPIC_LENGTH           10      /*!< length of mqtt topic string */

/* WIFI credentials */
// const char* SSID = "Galaxy";             /*!< WIFi SSID */
// const char* PASSWORD = "luwa2131";       /*!< WiFi password */

#define CALLIBRATION_READINGS       200         /*!< number of readings to take while calibrating the sensor */

#define GPS_TX 17                           /*!< GPS TX pin */
#define GPS_RX 16                           /*!< GPS RX pin */

/* File systems defines */
#define MB_SIZE_DIVISOR 1048576
#define FORMAT_SPIFFS_IF_FAILED 1

#define PREFLIGHT_BIT 0
#define POWERED_FLIGHT_BIT 1
#define APOGEE_BIT 2

#define STATE_CHANGE_DELAY 20
#endif // DEFS_H

