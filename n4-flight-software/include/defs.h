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
#define MQTT 1                                 /*!< set this to 1 if using MQTT for telemetry transfer */
#define XBEE 1                                 /*!< set to 1 if using XBEE for telemetry transfer */

#define GPS_BAUD_RATE 9600                     /*!< baud rate for the GPS module. Change accordingly */
#define XBEE_BAUD_RATE 9600                    /*!< baud rate for the XBEE HP module. Change accordingly */

/* debug parameters for use during testing - set to 0 for production */
#define DEBUGGING 1                           /*!< allow debugging to terminal. Set to 0 pre flight to disable serial terminal printing and improve speed  */
#define LOG_TO_MEMORY 0                       /*!< allow data logging to memory. Set to 1 to log data to external flash memory. Must be set during flight */
#define DEBUG_TO_TERMINAL 0                   /*!< allow create task that prints data to terminal. Set o 0 before flight  */

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

/* timing constant */
#define SETUP_DELAY 300
#define TASK_DELAY 10

/*!< Flight data constants  */
#define LAUNCH_DETECTION_THRESHOLD 5         /*!< altitude in meters, above which we register that we have launched  */
#define LAUNCH_DETECTION_ALTITUDE_WINDOW 20  /*!< Window in meters where we register a launch */
#define APOGEE_DETECTION_THRESHOLD 5         /*!< value in meters for detecting apogee */
#define MAIN_EJECTION_HEIGHT 1000            /*!< height to eject the main chute  */
#define DROGUE_EJECTION_HEIGHT               /*!< height to eject the drogue chute - ideally it should be at apogee  */
#define SEA_LEVEL_PRESSURE 101325            /*!< sea level pressure to be used for altitude calculations */
#define BASE_ALTITUDE 1417                   /*!< this value is the altitude at rocket launch site - adjust accordingly */

/*!<  tasks constants */
#define STACK_SIZE 1024                     /*!< task stack size in words */
#define ALTIMETER_QUEUE_LENGTH 10           /*!< length of the altimeter queue */
#define GYROSCOPE_QUEUE_LENGTH 10           /*!< length of the gyroscope queue */
#define GPS_QUEUE_LENGTH 24                 /*!< length of the gps queue */
#define TELEMETRY_DATA_QUEUE_LENGTH  10     /*!< length of the telemetry data queue */
#define FILTERED_DATA_QUEUE_LENGTH 10       /*!< length of the filtered data queue */
#define FLIGHT_STATES_QUEUE_LENGTH 1        /*!< length of the flight states queue */
#define CONSUME_TASK_DELAY    10

/* MQTT constants */
//const char MQTT_SERVER[30] = "192.168.1.101";
// const char MQTT_SERVER[30] = "broker.emqx.io";
const char MQTT_SERVER[30] = "192.168.1.113";
const char MQTT_TOPIC[30] = "n4/flight-computer-1";             /* make this topic unique to every rocket */
#define MQTT_PORT 1883                               /*!< MQTT broker port */

#define BROKER_IP_ADDRESS_LENGTH    20      /*!< length of broker ip address string */
#define MQTT_TOPIC_LENGTH           10      /*!< length of mqtt topic string */

/* WIFI credentials */
// const char* SSID = "Galaxy";             /*!< WIFi SSID */
// const char* PASSWORD = "luwa2131";       /*!< WiFi password */

#define CALLIBRATION_READINGS       200         /*!< number of readings to take while calibrating the sensor */

#define GPS_TX 17                           /*!< GPS TX pin */
#define GPS_RX 16                           /*!< GPS RX pin */

#define MB_SIZE_DIVISOR 1048576



#define PREFLIGHT_BIT 0
#define POWERED_FLIGHT_BIT 1
#define APOGEE_BIT 2

#define STATE_CHANGE_DELAY 50

#endif // DEFS_H

