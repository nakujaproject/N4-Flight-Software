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
#define MQTT 0                                 /*!< Set to 1 for normal mode (MQTT + Beacon switching), 0 for STRICT BEACON MODE (no WiFi/MQTT) */
#define TEST 1                                /*!< set to 1 to enable test mode - allows data transmission even when disarmed */
#define XBEE 0                               /*!< set to 1 if using XBEE for telemetry transfer */

// 🔥 STRICT BEACON MODE CONTROL
// When MQTT = 0: Only beacon communication, no WiFi AP, no MQTT switching allowed
// When MQTT = 1: Normal dynamic switching between MQTT and beacon modes
extern bool use_mqtt_mode;                       /*!< Enable MQTT transmission */
extern bool use_beacon_mode;                    /*!< Enable beacon transmission */
extern bool auto_fallback_enabled;             /*!< Enable automatic fallback to beacon when MQTT fails */
extern bool communication_mode_locked;        /*!< Lock mode changes during critical flight phases */
extern bool is_system_armed;                 /*!< Global armed state for both MQTT and beacon modes */

// Communication failure detection
#define MQTT_FAILURE_TIMEOUT 10000              /*!< 10 seconds without MQTT success = failure */
#define MQTT_RETRY_ATTEMPTS 3                  /*!< Number of MQTT retry attempts before fallback */
#define AUTO_FALLBACK_HYSTERESIS 30000        /*!< 30 seconds before auto-switching back to MQTT */

// Command definitions for dynamic mode switching
#define CMD_MQTT_MODE "MQTT_MODE"
#define CMD_BEACON_MODE "BEACON_MODE"
#define CMD_AUTO_FALLBACK_ON "AUTO_FALLBACK_ON"
#define CMD_AUTO_FALLBACK_OFF "AUTO_FALLBACK_OFF"
#define CMD_GET_MODE "GET_MODE"
#define CMD_ARM "ARM"
#define CMD_DISARM "DISARM"
#define CMD_RESET "RESET"

// Communication status tracking structure
typedef struct {
    uint32_t last_mqtt_success;
    uint32_t last_beacon_success;
    uint32_t mqtt_failure_count;
    uint32_t beacon_failure_count;
    bool mqtt_connection_stable;
    bool beacon_connection_stable;
    const char* current_mode;
    const char* last_command_source;
} communication_status_t;

extern communication_status_t comm_status;
#define BAUDRATE        115200
#define GPS_BAUD_RATE   9600                     /*!< baud rate for the GPS module. Change accordingly */
#define XBEE_BAUD_RATE  9600                    /*!< baud rate for the XBEE HP module. Change accordingly */

/* debug parameters for use during testing - set to 0 for production */
#define DEBUGGING 1                           /*!< allow debugging to terminal. Set to 0 pre flight to disable serial terminal printing and improve speed  */
#define LOG_TO_MEMORY 1                      /*!< allow data logging to memory. Set to 1 to log data to external flash memory. Must be set during flight */
#define DEBUG_TO_TERMINAL 1                 /*!< allow create task that print data to terminal. Set to 0 before flight  */

// Flash memory logging configuration to prevent blocking
#define ENABLE_FLASH_LOGGING 1              /*!< Set to 1 to enable flash logging for data storage */
#define ENABLE_QUEUE_LOGGING 1              /*!< Set to 1 to use queue-based logging instead of direct writes */
#define LOG_QUEUE_SIZE 50                    /*!< Size of logging queue - prevents blocking when flash is busy */
#define LOG_TASK_PRIORITY 1                  /*!< Low priority for logging task to not interfere with telemetry */
#define LOG_TASK_STACK_SIZE 4096            /*!< Dedicated stack for logging task */
#define LOG_WRITE_TIMEOUT_MS 10             /*!< Maximum time to wait for flash write (non-blocking) */

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
#define APOGEE_DETECTION_THRESHOLD 2         /*!< value in meters for detecting apogee - reduced for 120m simulation */
#define MAIN_EJECTION_HEIGHT 50             /*!< height to eject the main chute - 50m from ground for 120m simulation */
#define DROGUE_EJECTION_HEIGHT  1000             /*!< height to eject the drogue chute - ideally it should be at apogee  */
#define SEA_LEVEL_PRESSURE 101325            /*!< sea level pressure to be used for altitude calculations */
#define BASE_ALTITUDE 1417                   /*!< this value is the altitude at rocket launch site - adjust accordingly */

// Enhanced flight state detection using Kalman filtered data
#define ARM_ALTITUDE_THRESHOLD 50            /*!< minimum relative altitude (filtered) required for arming in meters */
#define DROGUE_DEPLOY_DELAY_MS 1000          /*!< delay after apogee detection before drogue deployment in milliseconds - 1 second for 120m simulation */
#define USE_KALMAN_FOR_STATE_DETECTION 1    /*!< set to 1 to use Kalman filtered altitude for all state decisions */

// Flight operation modes
enum OPERATION_MODE {
    SAFE_MODE = 0, /* Pyro-charges are disarmed  */
    ARMED_MODE      /* Pyro charges are armed and ready to deploy on apogee --see docs for more-- */
};

// MAC address declarations for global access
#include <stdint.h>

// MAC address definitions (now defined here for global access)
// Alternative MAC addresses (commented for reference):
// const uint8_t ROCKET_MAC[6] = {0x10, 0x06, 0x1c, 0xa6, 0x11, 0xf0};
// const uint8_t BASE_MAC[6]   = {0xf4, 0x65, 0x0b, 0x48, 0x5c, 0xf8};
// const uint8_t BASE_MAC[6]   = {0x10, 0x06, 0x1c, 0xa6, 0x11, 0xf0};

// Current active MAC addresses:
//const uint8_t ROCKET_MAC[6] = {0x10, 0x06, 0x1c, 0xa6, 0x18, 0x20};
//const uint8_t BASE_MAC[6]   = {0x10, 0x06, 0x1c, 0xa6, 0x11, 0xf0};
const uint8_t ROCKET_MAC[6] = {0x84, 0x1f, 0xe8, 0x36, 0x73, 0xac};
const uint8_t BASE_MAC[6]   = {0x08, 0xd1, 0xf9, 0x15, 0xa2, 0x0c};
// Externs for communication manager to restore WiFi/MQTT
class WIFIConfig;
extern WIFIConfig wifi_config;
extern void MQTTInit(const char* broker_IP, int broker_port);
extern void MQTT_Reconnect();

/*!<  tasks constants */
#define STACK_SIZE 2048                     /*!< task stack size in words - increased from 1024 to prevent stack overflows */
#define ALTIMETER_QUEUE_LENGTH 10           /*!< length of the altimeter queue */
#define GYROSCOPE_QUEUE_LENGTH 10           /*!< length of the gyroscope queue */
#define GPS_QUEUE_LENGTH 24                 /*!< length of the gps queue */
#define TELEMETRY_DATA_QUEUE_LENGTH  10     /*!< length of the telemetry data queue */
#define FILTERED_DATA_QUEUE_LENGTH 10       /*!< length of the filtered data queue */
#define FLIGHT_STATES_QUEUE_LENGTH 1        /*!< length of the flight states queue */
#define CONSUME_TASK_DELAY    100           /*!< Task delay in ms - increased to prevent watchdog timeouts */

// Logger queue management - prevents blocking main telemetry tasks
#define LOG_TO_MEM_QUEUE_LENGTH 100         /*!< Large queue for flash logging to prevent data loss */
extern QueueHandle_t log_to_mem_queue_handle;  /*!< Queue handle for non-blocking flash logging */

// Memory safety for strict beacon mode
#define BEACON_MODE_SAFETY_CHECKS 1         /*!< Enable additional memory checks in beacon-only mode */

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
