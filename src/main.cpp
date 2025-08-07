#include "communication_manager.h"
/**
 * @file main.cpp
 * @author Edwin Mwiti
 * @version N4
 * @date July 15 2024
 * 
 * @brief This contains the main driver code for the flight computer
 * 
 * 0x5765206D6179206D616B65206F757220706C616E73202C
 * 0x62757420476F642068617320746865206C61737420776F7264
 * 
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h> // TODO: ADD A MQTT SWITCH - TO USE MQTT OR NOT
#include <TinyGPSPlus.h>  // handle GPS
#include <SFE_BMP180.h>     // For BMP180
#include <FS.h>             // File system functions
#include <SD.h>             // SD card logging function
#include <SPIFFS.h>         // SPIFFS file system function
#include <esp_task_wdt.h>   // Watchdog timer functions
#include "defs.h"           // misc defines
#include "mpu.h"            // for reading MPU6050
#include "CustomSerialFlash.h"    // Handling external SPI flash memory
#include "logger.h"         // system logging
#include "data_types.h"     // definitions of data types used
#include "states.h"         // state machine states
#include "system_logger.h"  // system logging functions
#include "system_log_levels.h"  // system logging log levels
#include "wifi-config.h"    // handle wifi connection
#include "kalman_filter.h"  // handle kalman filter functions
#include "ring_buffer.h"    // for apogee detection
#include "espnow_beacon_transmitter.h"
#include "communication_manager.h"  // smart communication management
#include "sd_logger.h" // SD- Card logging

/**
 * flight states
 * these states are to be used for flight
**/
// OPERATION_MODE enum now defined in defs.h

/* state machine variables*/
uint8_t operation_mode = 0;                                         /*!< Tells whether software is in safe or flight mode - FLIGHT_MODE=1, SAFE_MODE=0 */
bool is_system_armed = false;                                       /*!< Global armed state for both MQTT and beacon modes */

// 🔥 ISOLATED COMMUNICATION SYSTEM - Global variables for independent mode control
bool use_mqtt_mode = true;                                          /*!< Enable MQTT transmission - starts with MQTT mode */
bool use_beacon_mode = false;                                       /*!< Enable beacon transmission - starts disabled */
bool auto_fallback_enabled = true;                                  /*!< Enable automatic fallback to beacon when MQTT fails */
bool communication_mode_locked = false;                             /*!< Lock mode changes during critical flight phases */
communication_status_t comm_status = {0};                           /*!< Communication status tracking */

/* non-task function prototypes definition */
void initDynamicWIFI();
void drogueChuteDeploy();
void mainChuteDeploy();
float kalmanFilter(float z);
void checkRunTestToggle();
void non_blocking_buzz(uint16_t interval);
void blocking_buzz(uint16_t interval);
double altimeter_get_pressure();
void mqtt_command_processor(const char*, const char*);
void arm_pyros();
void disarm_pyros();
void chutesInit();   
void espnowCommandTask(void* pvParameters);
SDLogger sdLogger(SD_CS_PIN);

// 🔥 GLOBAL COMMUNICATION MANAGER - External declaration (defined in communication_manager.cpp)
extern CommunicationManager comm_manager;

// Global telemetry buffer for seamless mode switching
static char global_telemetry_buffer[256];
static bool telemetry_data_ready = false;
static uint32_t last_telemetry_time = 0;

// Function to update global telemetry buffer for seamless switching
void updateGlobalTelemetryBuffer(const char* buffer) {
    strncpy(global_telemetry_buffer, buffer, sizeof(global_telemetry_buffer) - 1);
    global_telemetry_buffer[sizeof(global_telemetry_buffer) - 1] = '\0';
    telemetry_data_ready = true;
    last_telemetry_time = millis();
}

// Function to get the latest telemetry for immediate transmission during mode switch
bool getLatestTelemetryBuffer(char* buffer, size_t buffer_size) {
    if (telemetry_data_ready && (millis() - last_telemetry_time < 1000)) { // Data fresh within 1 second
        strncpy(buffer, global_telemetry_buffer, buffer_size - 1);
        buffer[buffer_size - 1] = '\0';
        return true;
    }
    return false;
}

// Create transmitter instance
ESPNowBeaconTransmitter transmitter(ROCKET_MAC, BASE_MAC);



void arm_pyros() {
    digitalWrite(REMOTE_SWITCH, HIGH);
    // Small delay to ensure pin state is stable
    vTaskDelay(pdMS_TO_TICKS(10));
}

void chutesInit() {
    pinMode(DROGUE_PIN, OUTPUT);
    pinMode(MAIN_CHUTE_EJECT_PIN, OUTPUT);
    pinMode(REMOTE_SWITCH, OUTPUT); // remote switch to arm pyros
    digitalWrite(DROGUE_PIN, LOW); // set drogue pin LOW
    digitalWrite(MAIN_CHUTE_EJECT_PIN, LOW); // set main chute pin LOW
    digitalWrite(REMOTE_SWITCH, LOW); // set remote switch LOW
}

/**
 *
 */
void disarm_pyros() {
    digitalWrite(REMOTE_SWITCH, LOW);
    // Small delay to ensure pin state is stable
    vTaskDelay(pdMS_TO_TICKS(10));
}

/* state machine variables*/
ARMED_FLIGHT_STATE current_state = ARMED_FLIGHT_STATE::PRE_FLIGHT_GROUND;	    /*!< The starting state - we start at PRE_FLIGHT_GROUND state */

uint8_t STATE_BIT_MASK = 0;

/* GPS object */
HardwareSerial gpsSerial(2); // PIN 16 AND 17 
TinyGPSPlus gps;
char gps_buffer[20];
gps_type_t gps_packet;

/* system logger */
SystemLogger SYSTEM_LOGGER;
const char* system_log_file = "/event_log.txt";
LOG_LEVEL level = INFO;
const char* rocket_ID = "FC1";             /*!< Unique ID of the rocket. Change to the needed rocket name before uploading */

/* set initial mode as safe mode
 * flag to indicate if we are in test or flight mode - This will
 * be changed by a command from the base station
 * */
uint8_t is_safe_mode = OPERATION_MODE::SAFE_MODE;

/* Intervals for buzzer state indication - see docs */
enum BUZZ_INTERVALS {
  SETUP_INIT = 200,
  ARMING_PROCEDURE = 500
};

/* LED blink intervals */
enum BLINK_INTERVALS {
    SAFE_BLINK = 400,
    ARMED_BLINK = 100
};

unsigned long current_non_block_time = 0;
unsigned long last_non_block_time = 0;
bool buzz_state = 0;

// 🔥 GLOBAL KALMAN FILTER OUTPUTS - Accessible by all tasks
float AltitudeKalman = 0.0, VelocityVerticalKalman = 0.0;

uint8_t mqtt_connect_flag;

/* hardware init check - to pinpoint any hardware failure during setup */
#define BMP_CHECK_BIT           0
#define IMU_CHECK_BIT           1   
#define FLASH_CHECK_BIT         2
#define GPS_CHECK_BIT           3
#define SD_CHECK_BIT            4
#define SPIFFS_CHECK_BIT        5
#define TEST_HARDWARE_CHECK_BIT 6
uint8_t SUBSYSTEM_INIT_MASK = 0b00000000;

/**
 * MQTT helper instances, if using MQTT to transmit telemetry
 */

WiFiClient wifi_client;
PubSubClient client(wifi_client);
uint8_t MQTTInit(const char* broker_IP, uint16_t broker_port);

/* WIFI configuration class object */
WIFIConfig wifi_config;

uint8_t drogue_pyro = 25;
uint8_t main_pyro = 12;
uint8_t flash_cs_pin = 5;                   /*!< External flash memory chip select pin */
uint8_t remote_switch = 27;

//chute deployment variables
volatile uint8_t drogue_pin_state = 0; 
volatile uint8_t main_chute_pin_state = 0;

// Battery monitoring variable
volatile float battery_voltage = 0.0;

// WiFi RSSI monitoring variable (for MQTT mode)
volatile int32_t wifi_rssi = 0;

/* Flight data logging */
uint8_t flash_led_pin = 32;                  /*!< LED pin connected to indicate flash memory formatting  */
char filename[] = "data_01.txt";         /*!< data log filename - Filename must be less than 20 chars, including the file extension */
uint32_t FILE_SIZE_512K = 524288L;          /*!< 512KB */
uint32_t FILE_SIZE_1M  = 1048576L;          /*!< 1MB */
uint32_t FILE_SIZE_4M  = 4194304L;          /*!< 4MB */
SerialFlashFile file;                       /*!< object representing file object for flash memory */
unsigned long long previous_log_time = 0;   /*!< The last time we logged data to memory */
unsigned long long current_log_time = 0;    /*!< What is the processor time right now? */
uint16_t log_sample_interval = 5;          /*!< After how long should we sample and log data to flash memory? */
/*/For data logging to disable both 
SD-CARD and flash memory before 
selecting the one data has to be logged to */
void disableAllDevices() {
  digitalWrite(flash_cs_pin, HIGH);
  digitalWrite(SD_CS_PIN, HIGH);
}
/* create flash memory log object */
DataLogger data_logger(flash_cs_pin, RED_LED_PIN, filename, file, FILE_SIZE_512K);

/* position integration variables */
long long current_time = 0;
long long previous_time = 0;

/* To store the main telemetry packet being sent over MQTT */
char telemetry_packet_buffer[150];
ring_buffer altitude_ring_buffer;
double baseline = 0.0; // to store baseline pressure from the altimeter
float curr_val;
float oldest_val;
uint8_t apogee_flag =0; // to signal that we have detected apogee
static int apogee_val = 0; // apogee altitude aproximmation
uint8_t main_eject_flag = 0;
volatile uint8_t DROGUE_DEPLOY_FLAG = 0;
volatile uint8_t MAIN_CHUTE_EJECT_FLAG = 0;

// Global current telemetry data for ARM altitude check
telemetry_type_t g_current_telemetry = {0};
unsigned long g_last_telemetry_update = 0;
static unsigned long apogee_detected_time = 0; // Timestamp when apogee was detected
/**
* @brief create dynamic WIFI
*/
void initDynamicWIFI() {
    // 🔥 STRICT BEACON MODE - Only initialize WiFi if MQTT flag is enabled
    if (MQTT) {
        uint8_t wifi_result = wifi_config.WifiConnect(false, ROCKET_MAC); // MQTT mode: use_beacon_mode = false
        if(wifi_result) {
            debugln("Wifi config OK!");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "Wifi config OK!\r\n");
        } else {
            debugln("Wifi config failed");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "Wifi config failed\r\n");
        }
    } else {
        debugln("MQTT disabled - WiFi not initialized in initDynamicWIFI (will be handled separately for beacon mode)");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "MQTT disabled - WiFi skipped in initDynamicWIFI\r\n");
    }
}

/**
* Check the toggle pin for TESTING or RUN mode 
 */
void checkRunTestToggle() {
    if(digitalRead(SET_RUN_MODE_PIN) == 0) {
        debugln("MODE:RUN");
    } else {
        debugln("MODE:TEST");
    }
}

/*!
 * @brief process commands sent from the base station
 * @param command
 * ARM
 * DISARM
 * RESET
 */
void espnowCommandTask(void* pvParameters) {
    ESPNowBeaconTransmitter::CommandPacket cmd;
    char cmdBuffer[32]; // Increased size for communication commands

    while (1) {
        if (transmitter.getNextCommand(&cmd)) {
            // Use fixed buffer instead of String to reduce stack usage
            size_t len = (cmd.length < 31) ? cmd.length : 31;
            memcpy(cmdBuffer, cmd.command, len);
            cmdBuffer[len] = '\0';
            
            // Trim whitespace manually to avoid String operations
            while (len > 0 && (cmdBuffer[len-1] == ' ' || cmdBuffer[len-1] == '\n' || cmdBuffer[len-1] == '\r')) {
                cmdBuffer[--len] = '\0';
            }

            // Check for communication mode commands first
            if (strncmp(cmdBuffer, "CMD_", 4) == 0) {
                comm_manager.handleModeCommand(String(cmdBuffer), "ESP_NOW");
            }
            else if (strcmp(cmdBuffer, "ARM") == 0) {
                // 🛡️ ARM SAFETY: Check altitude requirement (50m minimum using Kalman filtered data)
                #if USE_KALMAN_FOR_STATE_DETECTION
                float current_altitude = g_current_telemetry.alt_data.kalman_altitude;
                #else
                float current_altitude = g_current_telemetry.alt_data.rel_altitude;
                #endif
                
                // Check if telemetry data is recent (within last 2 seconds)
                if ((millis() - g_last_telemetry_update) > 2000) {
                    debugln("❌ ARM DENIED: Telemetry data too old for altitude check");
                    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::WARNING,
                                           system_log_file, "ARM DENIED: Stale telemetry data\r\n");
                } else if (current_altitude < ARM_ALTITUDE_THRESHOLD) {
                    debug("❌ ARM DENIED: Altitude too low (");
                    debug(current_altitude);
                    debug("m < ");
                    debug(ARM_ALTITUDE_THRESHOLD);
                    debugln("m requirement)");
                    
                    char log_msg[100];
                    snprintf(log_msg, sizeof(log_msg), "ARM DENIED: Alt %.1fm < %dm threshold\r\n", 
                            current_altitude, ARM_ALTITUDE_THRESHOLD);
                    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::WARNING,
                                           system_log_file, log_msg);
                } else {
                    // ✅ ARM CONDITIONS MET
                    arm_pyros();
                    chutesInit();
                    if (use_beacon_mode) {
                        transmitter.setArmed(true);
                    }
                    is_system_armed = true;  // Set global armed state
                    operation_mode = OPERATION_MODE::ARMED_MODE;
                    blocking_buzz(BUZZ_INTERVALS::ARMING_PROCEDURE);
                    
                    debug("🚀 ARMED via ESP-NOW (Alt: ");
                    debug(current_altitude);
                    debugln("m ✓)");
                    
                    char log_msg[100];
                    snprintf(log_msg, sizeof(log_msg), "ARMED via ESP-NOW at %.1fm altitude\r\n", current_altitude);
                    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO,
                                           system_log_file, log_msg);
                }
                
                // Yield CPU after ARM operation to ensure stack integrity
                vTaskDelay(pdMS_TO_TICKS(50));
            } 
            else if (strcmp(cmdBuffer, "DISARM") == 0) {
                disarm_pyros();
                if (use_beacon_mode) {
                    transmitter.setArmed(false);
                }
                is_system_armed = false;  // Set global armed state
                operation_mode = OPERATION_MODE::SAFE_MODE;
                blocking_buzz(BUZZ_INTERVALS::ARMING_PROCEDURE);
                debugln("🛑 DISARMED via ESP-NOW");
                
                SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO,
                                       system_log_file, "DISARMED via ESP-NOW\r\n");
                
                // Yield CPU after DISARM operation to ensure stack integrity
                vTaskDelay(pdMS_TO_TICKS(50));
            } 
            else if (strcmp(cmdBuffer, "RESET") == 0) {
                debugln("🔄 RESET via ESP-NOW");
                SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO,
                                       system_log_file, "RESET via ESP-NOW\r\n");
                vTaskDelay(pdMS_TO_TICKS(100)); // Small delay before restart
                ESP.restart();
            }
            else {
                debugln("Unknown ESP-NOW cmd: " + String(cmdBuffer));
                SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::WARNING,
                                       system_log_file, ("Unknown ESP-NOW cmd: " + String(cmdBuffer) + "\r\n").c_str());
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}




void mqtt_command_processor(const char* topic, const char* payload) {
    // Check if the topic is the commands topic (n4/commands)
    if(strcmp(topic, "n4/commands") == 0) {
        
        String command = String(payload);
        
        // Check for communication mode commands first
        if(command.startsWith("CMD_")) {
            comm_manager.handleModeCommand(command, "MQTT");
            return;
        }
        
        // Check the payload content for the actual command
        if(strcmp(payload, "ARM") == 0) {
            arm_pyros();
            chutesInit();
            if (use_beacon_mode) {
                transmitter.setArmed(true);  // Only sync with ESP-NOW transmitter in beacon mode
            }
            is_system_armed = true;  // Set global armed state
            operation_mode = OPERATION_MODE::ARMED_MODE;
            blocking_buzz(BUZZ_INTERVALS::ARMING_PROCEDURE);
            debugln("🚀 ARMED via MQTT!");
            
            // Simplified logging to reduce stack usage
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, 
                                   system_log_file, "ARMED via MQTT\r\n");
            
            // Yield CPU after ARM operation to ensure stack integrity
            vTaskDelay(pdMS_TO_TICKS(50));
        } 
        else if(strcmp(payload, "DISARM") == 0) {
            disarm_pyros();
            if (use_beacon_mode) {
                transmitter.setArmed(false);  // Only sync with ESP-NOW transmitter in beacon mode
            }
            is_system_armed = false;  // Set global armed state
            operation_mode = OPERATION_MODE::SAFE_MODE;
            blocking_buzz(BUZZ_INTERVALS::ARMING_PROCEDURE);
            debugln("🛑 DISARMED via MQTT");
            
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO,
                                   system_log_file, "DISARMED via MQTT\r\n");
            
            // Yield CPU after DISARM operation to ensure stack integrity
            vTaskDelay(pdMS_TO_TICKS(50));
        } 
        else if(strcmp(payload, "RESET") == 0) {
            debugln("🔄 RESET via MQTT");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO,
                                   system_log_file, "RESET via MQTT\r\n");
            vTaskDelay(pdMS_TO_TICKS(100)); // Small delay before restart
            ESP.restart();
        }
        else {
            debugln("🔍 Unknown MQTT command: " + String(payload));
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::WARNING,
                                   system_log_file, ("Unknown MQTT command: " + String(payload) + "\r\n").c_str());
        }
    }
}








/**
 * Task creation handles
 */
 TaskHandle_t readAccelerationTaskHandle;
 TaskHandle_t readAltimeterTaskHandle;
 TaskHandle_t readGPSTaskHandle;
 TaskHandle_t clearTelemetryQueueTaskHandle;
 TaskHandle_t checkFlightStateTaskHandle;
 TaskHandle_t flightStateCallbackTaskHandle;
 TaskHandle_t MQTT_TransmitTelemetryTaskHandle;
 TaskHandle_t kalmanFilterTaskHandle;
 
 TaskHandle_t debugToTerminalTaskHandle;
 TaskHandle_t logToMemoryTaskHandle;
 TaskHandle_t opModeIndicateTaskHandle;
 TaskHandle_t espnowCommandTaskHandle;

/**
 * ///////////////////////// DATA TYPES /////////////////////////
*/
accel_type_t acc_data;
gyro_type_t gyro_data;
gps_type_t gps_data;
altimeter_type_t altimeter_data;
telemetry_type_t telemetry_packet;

/**
 * ///////////////////////// END OF DATA VARIABLES /////////////////////////
*/

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// PERIPHERALS INIT                              /////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

/**
 * create an MPU6050 object
 * 0x68 is the address of the MPU
 * set gyro to max deg to 1000 deg/sec
 * set accel fs reading to 16g
*/
MPU6050 imu(MPU_ADDRESS, MPU_ACCEL_RANGE, GYRO_RANGE); 

/* create BMP object */
SFE_BMP180 altimeter;
double altimeter_temperature = 0.0;
altimeter_type_t altimeter_packet;

/**
* @brief initialize Buzzer
*/
void buzzerInit() {
    pinMode(BUZZER_PIN, OUTPUT);
}

void LED_init() {
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
}



/**
* @brief initialize SPIFFS for event logging during flight
*/
uint8_t InitSPIFFS() {
    if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
        debugln("SPIFFS mount failed"); // TODO: Set a flag for test GUI
        return 0;
    } else {
        debugln("SPIFFS init success");
        return 1;
    }
}

/**
* @brief initilize SD card 
 */
uint8_t initSD() {
    if (!SD.begin(SD_CS_PIN)) {
        delay(100);
        debugln(F("[-]SD Card mounting failed"));
        return 0;
    } else {
        debugln(F("[+]SD card Init OK!"));

        /* check for card type */
        uint8_t cardType = SD.cardType();
        if (cardType == CARD_NONE) {
            debugln("[-]No SD card attached");
        } else {
            debugln("[+]Valid card found");
        }

        // initialize test data file
        File file = SD.open("data.txt", FILE_WRITE); // TODO: change file name to const char*
        if (!file) {
            debugln("[File does not exist. Creating file]");
            debugln("Test data file created");
        } else {
            debugln("[*]Data file already exists");
        }
        file.close();

        // initialize test state file 
        File state_file = SD.open("/state.txt", FILE_WRITE);
        if(!state_file) {
            debugln("State file does not exit. Creating file...");

            debugln("state file created."); // TODO: move to system logger
        }

        state_file.close();

        return 1;
    }
}

/*!****************************************************************************
 * @brief Initialize BMP180 barometric sensor
 * @return TODO: 1 if init OK, 0 otherwise
 * 
 *******************************************************************************/
uint8_t BMPInit() {
    if(altimeter.begin()) {
        debugln("[+]BMP init OK.");
        return 1;
    } else {
        debugln("[+]BMP init failed");
        return 0;
    }
}

/*!****************************************************************************
 * @brief Initialize the GPS connected on Serial2
 * @return 1 if init OK, 0 otherwise
 * 
 *******************************************************************************/
uint8_t GPSInit() {
    gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX, GPS_TX);
    delay(50);

    debugln("[+]GPS init OK!"); 

    /**
     * FIXME: Proper GPS init check!
     * Look into if the GPS has acquired a LOCK on satelites 
     * Only if it has a lock then can we return a 1
     * */ 

    return 1;
}

/**
* @brief - non-blocking buzz 
 */
void non_blocking_buzz(uint16_t interval) {
        /* non-blocking buzz */
    current_non_block_time = millis();
    if((current_non_block_time - last_non_block_time) > interval) {
        buzz_state = !buzz_state;
        last_non_block_time = current_non_block_time;
        digitalWrite(BUZZER_PIN, buzz_state);
    }
}

/**
 * @brief blocking buzz 
 */
void blocking_buzz(uint16_t interval) {
    digitalWrite(BUZZER_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(interval)); // Use vTaskDelay to avoid watchdog issues
    digitalWrite(BUZZER_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(interval)); // Use vTaskDelay to avoid watchdog issues
}

/**
 * ///////////////////////// END OF PERIPHERALS INIT /////////////////////////
 */
QueueHandle_t telemetry_data_queue_handle;
QueueHandle_t log_to_mem_queue_handle;
QueueHandle_t csv_log_queue_handle;  // 🔥 NEW: CSV string logging queue
QueueHandle_t check_state_queue_handle;
QueueHandle_t debug_to_term_queue_handle;
QueueHandle_t kalman_filter_queue_handle;
QueueHandle_t kalman2d_input_queue_handle;


//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// ACCELERATION AND ROCKET ATTITUDE DETERMINATION /////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

/*!****************************************************************************
 * @brief Read acceleration data from the accelerometer
 * @param pvParameters - A value that is passed as the paramater to the created task.
 * If pvParameters is set to the address of a variable then the variable must still exist when the created task executes - 
 * so it is not valid to pass the address of a stack variable.
 * @return Updates accelerometer data struct on the telemetry queue
 * 
 *******************************************************************************/
void readAccelerationTask(void* pvParameter) {
    telemetry_type_t acc_data_lcl;
    static uint32_t record_counter = 0; // Reset to 0 on each reboot

    while(1) {
        acc_data_lcl.operation_mode = operation_mode;
        acc_data_lcl.record_number = ++record_counter; // Use pre-increment for proper counting
        acc_data_lcl.state = current_state;
        acc_data_lcl.alt_data.rel_altitude = altimeter_packet.rel_altitude;
        acc_data_lcl.drogue_pin_state = drogue_pin_state;
        acc_data_lcl.main_chute_pin_state = main_chute_pin_state;
        
        // Use the global battery voltage read by monitorChutePinsTask
        acc_data_lcl.battery_voltage = battery_voltage;
        
        // Use the global wifi_rssi read by monitorChutePinsTask
        acc_data_lcl.wifi_rssi = wifi_rssi;

        // read acceleration
        acc_data_lcl.acc_data.ax = imu.readXAcceleration();
        acc_data_lcl.acc_data.ay = imu.readYAcceleration();
        acc_data_lcl.acc_data.az = imu.readZAcceleration();

        // read angular velocities
        acc_data_lcl.gyro_data.gx = imu.readXAngularVelocity();
        acc_data_lcl.gyro_data.gy = imu.readYAngularVelocity();
        acc_data_lcl.gyro_data.gz = imu.readZAngularVelocity();

        // get pitch and roll
        acc_data_lcl.acc_data.pitch = imu.getPitch();
        acc_data_lcl.acc_data.roll = imu.getRoll();
        
        // 🔥 SYNCHRONIZED KALMAN DATA - Include latest Kalman filter results in all telemetry packets
        acc_data_lcl.alt_data = altimeter_packet; // Copy entire altimeter data including Kalman results
        
        // 🛡️ UPDATE GLOBAL TELEMETRY - For ARM altitude safety checks
        g_current_telemetry = acc_data_lcl;
        g_last_telemetry_update = millis();
        
        // Send to queues for other tasks
        xQueueSend(log_to_mem_queue_handle, &acc_data_lcl, 0);
        xQueueSend(check_state_queue_handle, &acc_data_lcl, 0);
        xQueueSend(debug_to_term_queue_handle, &acc_data_lcl, 0);
        //Testing to be commented out
        //xQueueSend(telemetry_data_queue_handle, &acc_data_lcl, 0);

        //MQTT transmission (if enabled) - only send to queue when armed or in test mode
        if (MQTT) {
            // 🔥 ENHANCED DATA CONTINUITY - Always queue data for MQTT task when MQTT mode is active
            if (comm_manager.isMQTTActive()) {
                // Send to MQTT task queue for processing (arm check handled in MQTT task)
                xQueueSend(telemetry_data_queue_handle, &acc_data_lcl, 0);
            }
        }
        
        vTaskDelay(20/ portTICK_PERIOD_MS);
    }

}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// ALTITUDE AND VELOCITY DETERMINATION ////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

/*!****************************************************************************
 * @brief Read the raw pressure from the altimeter
 *******************************************************************************/
double altimeter_get_pressure()
{
    char status;
    double T, P, p0, a;
    status = altimeter.startTemperature();
    if(status != 0)
    {
        vTaskDelay(pdMS_TO_TICKS(status)); // Non-blocking delay for FreeRTOS tasks
        status = altimeter.getTemperature(T);
        altimeter_temperature = T;
        if(status != 0)
        {
            status = altimeter.startPressure(3);
            if(status != 0)
            {
                vTaskDelay(pdMS_TO_TICKS(status)); // Non-blocking delay for FreeRTOS tasks
                status = altimeter.getPressure(P, T);
                if(status != 0)
                {
                    return P;
                } else debugln("Error getting pressure");
            } else debugln("error starting pressure");
        } else debugln("error getting temperature");
    } else debugln("error starting pressure measurement");
 return P;  
}

// /*!****************************************************************************
//  * @brief Read atm pressure data from the barometric sensor onboard
//  *******************************************************************************/


float estimatedAltitude = 100;
// Real altimeter task - commented out for simulation
// void readAltimeterTask(void* pvParameters) {
//     telemetry_type_t alt_data_lcl;

//     while(1) {

//         double a, P;
//         P = altimeter_get_pressure();
//         a = altimeter.altitude(P, baseline);
        
//         //a = 100 + ((rand() % 2001 - 1000) / 100.0); // 100 ± 10m noise
        
//         //Serial.print("\nRaw Alt"); Serial.print(a);Serial.print("\nRaw Alt");
//         estimatedAltitude = a;
//         //altimeter_packet.filtered_altitude_1d = kalmanFilter(a);
//         float filtered_alt = kalmanFilter(a);
//         altimeter_packet.filtered_altitude_1d = filtered_alt;
//         xQueueSend(kalman2d_input_queue_handle, &filtered_alt, 0);  // 👈 send to 2D filter

//         //Serial.print("\n");Serial.print("Raw Altitude");Serial.print(a);Serial.print("\n");
//         /* send to altimeter global packet */
//         altimeter_packet.temperature = altimeter_temperature;
//         altimeter_packet.pressure = P;
//         altimeter_packet.rel_altitude = a;
        
//         // Update global altimeter packet with latest Kalman filter results
//         // Note: These values are updated by taskKalman2D, this ensures they're always available
//         altimeter_packet.kalman_altitude = AltitudeKalman;
//         altimeter_packet.kalman_vertical_velocity = VelocityVerticalKalman;
        
//         //vTaskDelay(CONSUME_TASK_DELAY / portTICK_PERIOD_MS);
//         vTaskDelay(20 / portTICK_PERIOD_MS);
//     }
// }
 

// 🚀 FLIGHT SIMULATION - Uncommented for state machine testing
void readAltimeterTask(void* pvParameters) {
    telemetry_type_t alt_data_lcl;
    static double a = 0; // simulated altitude
    static double P = 101325;
    static int phase = 0; // 0: ascent, 1: apogee, 2: descent
    static int count = 0;

    while(1) {
        // Ascent: 1000m over 60s (600 cycles at 100ms, +1.67m per cycle)
        if (phase == 0) {
            a += 1.67;
            debug("🚀 ASCENT: Alt="); debug(a); debug("m, Count="); debug(count); debugln("/600");
            if (++count >= 600) { 
                phase = 1; 
                count = 0; 
                debugln("🎯 REACHED APOGEE - Starting apogee hold");
            }
        } else if (phase == 1) { // Apogee hold: 20s (200 cycles)
            debug("🎯 APOGEE HOLD: Alt="); debug(a); debug("m, Count="); debug(count); debugln("/200");
            if (++count >= 200) { 
                phase = 2; 
                count = 0;
                debugln("⬇️ STARTING DESCENT");
            }
        } else if (phase == 2) { // Descent: 1000m over 60s
            a -= 1.67;
            if (a < 0) a = 0;
            debug("⬇️ DESCENT: Alt="); debug(a); debugln("m");
        }

        P = 101325 * exp(-a / 8434.5);
        
        // Apply Kalman filtering to simulated data
        estimatedAltitude = a;
        float filtered_alt = kalmanFilter(a);
        altimeter_packet.filtered_altitude_1d = filtered_alt;
        xQueueSend(kalman2d_input_queue_handle, &filtered_alt, 0);
        
        altimeter_packet.temperature = 25.0; // Simulated temperature
        altimeter_packet.pressure = P;
        altimeter_packet.rel_altitude = a;
        
        // Update global altimeter packet with latest Kalman filter results
        altimeter_packet.kalman_altitude = AltitudeKalman;
        altimeter_packet.kalman_vertical_velocity = VelocityVerticalKalman;

        vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms cycle for simulation
    }
}



// /*!****************************************************************************
//  * @brief Read the GPS location data and altitude and append to telemetry packet for transmission
//  * @param pvParameters - A value that is passed as the paramater to the created task.
//  * If pvParameters is set to the address of a variable then the variable must still exist when the created task executes - 
//  * so it is not valid to pass the address of a stack variable.
//  * 
//  *******************************************************************************/
// Real GPS task - commented out for simulation
// void readGPSTask(void* pvParameters){
//     float latitude, longitude, g_altitude;

//     gps_type_t gps_data_lcl;

//     while(1){
//         if(gpsSerial.available() > 0) {
//             gps.encode(gpsSerial.read());

//             /* get GPS coordinates */
//             if(gps.location.isValid()) {
//                 latitude = gps.location.lat();
//                 longitude = gps.location.lng();
//             } 

//             /* get GPS altitude */
//             if(gps.altitude.isValid()) {
//                 g_altitude = gps.altitude.meters();
//             }
//         } else {
//             vTaskDelay(10/portTICK_PERIOD_MS);
//         }

//         gps_packet.latitude = latitude;
//         gps_packet.longitude = longitude;
//         gps_packet.gps_altitude = g_altitude;
//     }
// }

// 🛰️ GPS SIMULATION - Uncommented for state machine testing
void readGPSTask(void* pvParameters){
    
    static float sim_gps_altitude = 0;
    static int phase = 0;
    static int count = 0;

    while(1){
        if (phase == 0) {
            sim_gps_altitude += 1.67;
            if (++count >= 600) { phase = 1; count = 0; }
        } else if (phase == 1) {
            if (++count >= 200) { phase = 2; count = 0; }
        } else if (phase == 2) {
            sim_gps_altitude -= 1.67;
            if (sim_gps_altitude < 0) sim_gps_altitude = 0;
        }

        // Simulate JKUAT coordinates with altitude changes
        gps_packet.latitude = -1.2833;   // JKUAT latitude
        gps_packet.longitude = 36.8167;  // JKUAT longitude
        gps_packet.gps_altitude = sim_gps_altitude;
        gps_packet.time = millis();      // Simulated GPS time

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Kalman filter estimated value calculation
 * 
 */
// float kalmanFilter(float z) {
//     float estimated_altitude_pred = estimated_altitude;
//     float error_covariance_pred = error_covariance_bmp + process_variance_bmp;
//     kalman_gain_bmp = error_covariance_pred / (error_covariance_pred + measurement_variance_bmp);
//     estimated_altitude = estimated_altitude_pred + kalman_gain_bmp * (z - estimated_altitude_pred);
//     error_covariance_bmp = (1 - kalman_gain_bmp) * error_covariance_pred;

//     return estimated_altitude;
// }
BLA::Matrix<2,2> F, P, Q, I;
BLA::Matrix<2,1> G, S, K;
BLA::Matrix<1,2> H;
BLA::Matrix<1,1> R, L, inv_L, Acc, M;

extern float estimatedAltitude;
float timeStep = 0.003; // 3ms time step for 2D Kalman filter (matches task delay)
const int ledPin= 25;
unsigned long timer = 0;

float errorCovariance_bmp = 1.0;
float processVariance_bmp = 0.001;
float measurementVariance_bmp = 0.1;
float kalmanGain_bmp;


  void init_kalman_matrices() {
  F = {1, 0.0034, 0, 1};
  G = {0.5 * 0.003 * 0.003, 0.003};
  H = {1, 0};
  I = {1, 0, 0, 1};
  Q = G * ~G * 4.0f * 4.0f;
  R = {0.3 * 0.3};
  P = {0, 0, 0, 0};
  S = {0, 0};
  }
// Apply Kalman filter to new altitude measurements
float kalmanFilter(float z) {
  float estimatedAltitude_pred = estimatedAltitude;
  float errorCovariance_pred = errorCovariance_bmp + processVariance_bmp;
  kalmanGain_bmp = errorCovariance_pred / (errorCovariance_pred + measurementVariance_bmp);
  estimatedAltitude = estimatedAltitude_pred + kalmanGain_bmp * (z - estimatedAltitude_pred);
  errorCovariance_bmp = (1 - kalmanGain_bmp) * errorCovariance_pred;

  return estimatedAltitude;
}
// void taskKalman2D(void *pvParameters) {
//     Serial.println("📡 Kalman2D Task Started");
//     telemetry_type_t input_data;
//     telemetry_type_t telemetry_data;
//     telemetry_type_t acc_data_lcl;
//     while (true) {
//         //Serial.println("📡 Kalman2D Task Started in loop");
     

//     timer = millis();

//     // Read raw acceleration and gyroscope data
    


//     // Calculate tilt-adjusted AccZInertial
//     // AngleRoll = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + (a.acceleration.z+1.0) * a.acceleration.z)) * 180 / PI;
//     // AnglePitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + (a.acceleration.z+1.0)* a.acceleration.z)) * 180 / PI;
//     // //Gravitational acceleration is in the downward direction and is taken as positive ,since we are moving upwards, we need to invert the sign so that g is negative
//     // AccZInertial = -((-sin(AnglePitch * (PI / 180)) * a.acceleration.x +
//     //         cos(AnglePitch * (PI / 180)) * sin(AngleRoll * (PI / 180)) * a.acceleration.y +
//     //         cos(AnglePitch * (PI / 180)) * cos(AngleRoll * (PI / 180)) * a.acceleration.z) - 8.93);//should be -9.81 , taken as 9 to account for error
//     // AccZInertial_g = AccZInertial / 9.81;
//     float AccYInertial = acc_data_lcl.acc_data.az-1.03;
//     float AccYInertial_g = AccYInertial / 9.81;
//     // Apply 2D Kalman filter
//     Acc = {AccYInertial};
//     //Acc= 0 ;
//     S = F * S + G * Acc;
//     P = F * P * ~F + Q;
//     L = H * P * ~H + R;
//     inv_L = Inverse(L);
//     K = P * ~H * inv_L;
//     float input_alt;
//     if (xQueueReceive(kalman2d_input_queue_handle, &input_alt, 0) == pdTRUE) {
//     M = {input_alt};  
//     }

    
//     S = S + K * (M - H * S);
//     AltitudeKalman = S(0, 0);
//     VelocityVerticalKalman = S(1, 0);

//     altimeter_packet.kalman_altitude = AltitudeKalman;
//     altimeter_packet.kalman_vertical_velocity = VelocityVerticalKalman;
//     P = (I - K * H) * P;
   
//     telemetry_data.alt_data.kalman_altitude = altimeter_packet.kalman_altitude;
//     telemetry_data.alt_data.kalman_vertical_velocity = altimeter_packet.kalman_vertical_velocity;

//      // After updating altimeter_packet.kalman_altitude and altimeter_packet.kalman_vertical_velocity

//    telemetry_type_t telemetry_data;
//    telemetry_data.alt_data = altimeter_packet; // Copy all altimeter data, including filtered values
// // Fill other fields as needed (accel, gyro, gps, etc.)

//    xQueueSend(debug_to_term_queue_handle, &telemetry_data, 0); // Or your actual output queue


    
//     // Serial output
//     digitalWrite(ledPin,HIGH);
//     //Serial.println("📡 Kalman2D Task Running");

//     // Serial.print(" Raw acceleration:");Serial.print(az);Serial.print("\n");
//     // Serial.print("accleration in z direction:");Serial.print(AccZInertial);Serial.print("\n");
//     // Serial.print("Raw Altitude:");Serial.print(estimatedAltitude);Serial.print("\n");
//     //Serial.print("Filtered Altitude:"); Serial.print(AltitudeKalman); Serial.print("\n");
//     // Serial.print("VerticalVelocity:"); Serial.print(VelocityVerticalKalman); Serial.print("\n");
//     //Serial.print("AccZInertial (m/s²):"); Serial.print(AccYInertial); Serial.print("\n");
//     //Serial.print("AccZInertial (g):"); Serial.print(AccYInertial_g); Serial.print("\n");
//      vTaskDelay((timeStep * 1000) / portTICK_PERIOD_MS);
//     }
   
//     }




void taskKalman2D(void *pvParameters) {
    float input_altitude;
    telemetry_type_t acc_data_lcl; // For acceleration data access
    
    while (true) {
        // Wait for filtered altitude data from the readAltimeterTask
        if (xQueueReceive(kalman2d_input_queue_handle, &input_altitude, portMAX_DELAY) == pdTRUE) {
            // We need acceleration data for the 2D Kalman filter
            // Read the latest acceleration data (non-blocking)
            if (xQueuePeek(debug_to_term_queue_handle, &acc_data_lcl, 0) == pdTRUE) {
                float offset = 9.425; // Adjust this offset based on your calibration
                float AccYInertial = (acc_data_lcl.acc_data.az*9.8)-offset;
                Acc = {AccYInertial};
            } else {
                // If no acceleration data available, use 0
                Acc = {0.0};
            }
        
            //Serial.printf("RawAcc");Serial.printf(%2f,Acc);Serial.printf("\n");
            //Serial.printf("Raw Alt: %.2f  Acc: %.2f\n", altimeter_packet.filtered_altitude_1d, AccYInertial);
            //Serial.printf("Before Prediction S: %.4f %.4f\n", S(0,0), S(1,0));

            // PREDICTION
            S = F * S + G * Acc;
            P = F * P * ~F + Q;

            //Serial.printf("After Prediction S: %.4f %.4f\n", S(0,0), S(1,0));
            //Serial.printf("P(0,0): %.4f  P(1,1): %.4f\n", P(0,0), P(1,1));

            // UPDATE
            L = H * P * ~H + R;

            if (fabs(L(0, 0)) < 1e-6 || isnan(L(0,0))) {
                Serial.println(" Skipping update: L is zero or NaN");
                vTaskDelay((timeStep * 1000) / portTICK_PERIOD_MS);
                continue;
            }

            inv_L = Inverse(L);
            K = P * ~H * inv_L;

            // Use the input altitude from the queue instead of altimeter_packet
            M = {input_altitude};

            //Serial.printf("M (meas): %.4f\n", M(0,0));
            //Serial.printf("Kalman Gain K: %.4f %.4f\n", K(0,0), K(1,0));

            S = S + K * (M - H * S);

            //  FINAL VALUES
            AltitudeKalman = S(0, 0);
            VelocityVerticalKalman = S(1, 0);
            
            // 🔥 SYNCHRONIZED KALMAN OUTPUT - Update global altimeter packet for consistent data across all tasks
            altimeter_packet.kalman_altitude = AltitudeKalman;
            altimeter_packet.kalman_vertical_velocity = VelocityVerticalKalman;

            //Serial.printf(" Filtered Alt: %.4f  Vel: %.4f\n", AltitudeKalman, VelocityVerticalKalman);
            //Serial.printf("2D Kalman: Alt=%.2f, Vel=%.2f\n", AltitudeKalman, VelocityVerticalKalman);

        }

        vTaskDelay((timeStep * 1000) / portTICK_PERIOD_MS);
    }
}


/*!***************************************************************************
 * @brief Filter data using the Kalman Filter 
 * 
 */
void kalmanFilterTask(void* pvParameters) {
    
    while (1) {
        vTaskDelay(CONSUME_TASK_DELAY/portTICK_PERIOD_MS);
    }
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  Serial.print("🔥 Stack overflow in task: ");
  Serial.println(pcTaskName);
}

// /*!****************************************************************************
//  * @brief check various condition from flight data to change the flight state
//  * - -see states.h for more info --
//  *
//  *******************************************************************************/
// void checkFlightState(void* pvParameters) {
//     // get the flight state from the telemetry task
//     telemetry_type_t flight_data; 
    
//     while (1) {
//         xQueueReceive(check_state_queue_handle, &flight_data, portMAX_DELAY);

//         if(apogee_flag != 1) {
//             // states before apogee
//             // debug("altitude value:"); debugln(flight_data.alt_data.altitude);
//             if(flight_data.alt_data.rel_altitude < LAUNCH_DETECTION_THRESHOLD) {
//                 current_state = ARMED_FLIGHT_STATE::PRE_FLIGHT_GROUND;
//                 //debugln("PREFLIGHT");
//                 delay(STATE_CHANGE_DELAY);
//             } else if(LAUNCH_DETECTION_THRESHOLD < flight_data.alt_data.rel_altitude < (LAUNCH_DETECTION_THRESHOLD+LAUNCH_DETECTION_ALTITUDE_WINDOW) ) {
//                 current_state = ARMED_FLIGHT_STATE::POWERED_FLIGHT;
//                 //debugln("POWERED");
//                 delay(STATE_CHANGE_DELAY);
//             } 

//             // COASTING

//             // APOGEE and APOGEE DETECTION
//             ring_buffer_put(&altitude_ring_buffer, flight_data.alt_data.rel_altitude);
//             if(ring_buffer_full(&altitude_ring_buffer) == 1) {
//                 oldest_val = ring_buffer_get(&altitude_ring_buffer);
//             }

//             //debug("Curr val:");debug(flight_data.alt_data.altitude); debug("    "); debugln(oldest_val);
//             if((oldest_val - flight_data.alt_data.rel_altitude) >= APOGEE_DETECTION_THRESHOLD) {
//                 if(apogee_flag == 0) {
//                     apogee_val = ( (oldest_val - flight_data.alt_data.rel_altitude) / 2 ) + oldest_val;

//                     current_state = ARMED_FLIGHT_STATE::APOGEE;
//                     delay(STATE_CHANGE_DELAY);
//                     //debugln("APOGEE");
//                     delay(STATE_CHANGE_DELAY);
//                     current_state = ARMED_FLIGHT_STATE::DROGUE_DEPLOY;
//                     //debugln("DROGUE");
//                     delay(STATE_CHANGE_DELAY);
//                     current_state =  ARMED_FLIGHT_STATE::DROGUE_DESCENT;
//                     //debugln("DROGUE_DESCENT");
//                     delay(STATE_CHANGE_DELAY);
//                     apogee_flag = 1;
//                 }
//             }

//         } else if(apogee_flag == 1) {
//             if(LAUNCH_DETECTION_THRESHOLD <= flight_data.alt_data.rel_altitude <= apogee_val) {
//                 if(main_eject_flag == 0) {
//                     current_state = ARMED_FLIGHT_STATE::MAIN_DEPLOY;
//                     //debugln("MAIN");
//                     delay(STATE_CHANGE_DELAY);
//                     main_eject_flag = 1;
//                 } else if (main_eject_flag == 1) { // todo: confirm check_done_flag
//                     current_state = ARMED_FLIGHT_STATE::MAIN_DESCENT;
//                     //debugln("MAIN_DESC");
//                     delay(STATE_CHANGE_DELAY);
//                 }
//             }

//             if(flight_data.alt_data.rel_altitude < LAUNCH_DETECTION_THRESHOLD) {
//                 current_state = ARMED_FLIGHT_STATE::POST_FLIGHT_GROUND;
//                 //debugln("POST_FLIGHT");
//             }
//         }

//         flight_data.state = current_state;

//     }
// }
void checkFlightState(void* pvParameters) {
    telemetry_type_t flight_data;
    static uint8_t last_state = 0xFF;

    while (1) {
        xQueueReceive(check_state_queue_handle, &flight_data, portMAX_DELAY);

        // 🎯 Use Kalman filtered altitude for enhanced accuracy
        #if USE_KALMAN_FOR_STATE_DETECTION
        float alt = flight_data.alt_data.kalman_altitude;
        #else
        float alt = flight_data.alt_data.rel_altitude;
        #endif

        // Update global telemetry for ARM altitude check
        g_current_telemetry = flight_data;
        g_last_telemetry_update = millis();

        // 🚀 AUTOMATIC ARMING: Auto-arm when reaching 50m altitude (if not already armed)
        if (!is_system_armed && alt >= ARM_ALTITUDE_THRESHOLD) {
            arm_pyros();
            chutesInit();
            if (use_beacon_mode) {
                transmitter.setArmed(true);
            }
            is_system_armed = true;
            operation_mode = OPERATION_MODE::ARMED_MODE;
            blocking_buzz(BUZZ_INTERVALS::ARMING_PROCEDURE);
            
            debug("🚀 AUTO-ARMED at ");
            debug(alt);
            debug("m altitude (threshold: ");
            debug(ARM_ALTITUDE_THRESHOLD);
            debugln("m) ✓");
            
            char log_msg[100];
            snprintf(log_msg, sizeof(log_msg), "AUTO-ARMED at %.1fm altitude\r\n", alt);
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO,
                                   system_log_file, log_msg);
        }

        // --- Pre-apogee states ---
        if (!apogee_flag) {
            if (alt < LAUNCH_DETECTION_THRESHOLD) {
                current_state = ARMED_FLIGHT_STATE::PRE_FLIGHT_GROUND;
            } else if (alt >= LAUNCH_DETECTION_THRESHOLD &&
                       alt < (LAUNCH_DETECTION_THRESHOLD + LAUNCH_DETECTION_ALTITUDE_WINDOW)) {
                current_state = ARMED_FLIGHT_STATE::POWERED_FLIGHT;
            } else if (alt >= (LAUNCH_DETECTION_THRESHOLD + LAUNCH_DETECTION_ALTITUDE_WINDOW)) {
                current_state = ARMED_FLIGHT_STATE::COASTING;
            }

            // 🎯 Enhanced apogee detection using Kalman filtered altitude
            ring_buffer_put(&altitude_ring_buffer, alt);
            if (ring_buffer_full(&altitude_ring_buffer)) {
                oldest_val = ring_buffer_get(&altitude_ring_buffer);
                // debug("Current alt: "); debug(alt);
                // debug(" | Oldest alt: "); debug(oldest_val);
                // debug(" | Diff: "); debugln(oldest_val - alt);
            }
            if ((oldest_val - alt) >= APOGEE_DETECTION_THRESHOLD && apogee_flag == 0) {
                apogee_val = oldest_val;
                current_state = ARMED_FLIGHT_STATE::APOGEE;
                apogee_flag = 1;
                apogee_detected_time = millis(); // Record apogee detection time
                debug("🎯 APOGEE DETECTED at ");
                debug(alt);
                debugln("m (Kalman filtered)!");
            }
        }
        // --- Post-apogee states ---
        else {
            switch (current_state) {
                case ARMED_FLIGHT_STATE::APOGEE:
                    // 📦 Deploy drogue after delay (not immediately at apogee)
                    if ((millis() - apogee_detected_time) >= DROGUE_DEPLOY_DELAY_MS) {
                        current_state = ARMED_FLIGHT_STATE::DROGUE_DEPLOY;
                        debug("⏰ Drogue deployment delay (");
                        debug(DROGUE_DEPLOY_DELAY_MS);
                        debugln("ms) completed");
                    }
                    break;
                case ARMED_FLIGHT_STATE::DROGUE_DEPLOY:
                    // Wait for deploy flag
                    if (DROGUE_DEPLOY_FLAG) {
                        current_state = ARMED_FLIGHT_STATE::DROGUE_DESCENT;
                        DROGUE_DEPLOY_FLAG = 0; // Reset for safety
                    }
                    break;
                case ARMED_FLIGHT_STATE::DROGUE_DESCENT:
                    // 📦 Wait for main deploy altitude (using Kalman filtered data)
                    if (alt <= MAIN_EJECTION_HEIGHT && main_eject_flag == 0) {
                        current_state = ARMED_FLIGHT_STATE::MAIN_DEPLOY;
                        main_eject_flag = 1;
                        debug("📦 Main chute altitude reached: ");
                        debug(alt);
                        debug("m <= ");
                        debug(MAIN_EJECTION_HEIGHT);
                        debugln("m");
                    }
                    break;
                case ARMED_FLIGHT_STATE::MAIN_DEPLOY:
                    // Wait for main deploy flag
                    if (MAIN_CHUTE_EJECT_FLAG) {
                        current_state = ARMED_FLIGHT_STATE::MAIN_DESCENT;
                        MAIN_CHUTE_EJECT_FLAG = 0; // Reset for safety
                    }
                    break;
                case ARMED_FLIGHT_STATE::MAIN_DESCENT:
                    // Wait for landing
                    if (alt < LAUNCH_DETECTION_THRESHOLD) {
                        current_state = ARMED_FLIGHT_STATE::POST_FLIGHT_GROUND;
                    }
                    break;
                default:
                    // If landed, stay in POST_FLIGHT_GROUND
                    if (alt < LAUNCH_DETECTION_THRESHOLD) {
                        current_state = ARMED_FLIGHT_STATE::POST_FLIGHT_GROUND;
                    }
                    break;
            }
        }

        // Debug state change
        if (current_state != last_state) {
            debug("🚀 State changed to: ");
            debug(current_state);
            debug(" (Alt: ");
            debug(alt);
            debugln("m)");
            last_state = current_state;
        }

        flight_data.state = current_state;
        vTaskDelay(STATE_CHANGE_DELAY / portTICK_PERIOD_MS);
    }
}

/*!****************************************************************************
 * @brief monitor the chute pins to see if they are deployed
 * @param pvParameters - A value that is passed as the paramater to the created task
 * If pvParameters is set to the address of a variable then the variable must still exist when the created task executes - 
 * so it is not valid to pass the address of a stack variable. 
 * This task reads the state of the drogue and main chute pins every 50ms
 * If the pin is HIGH, it means the chute has been deployed
 *******************************************************************************/



void monitorChutePinsTask(void* pvParameters) {
    while (1) {
        drogue_pin_state = digitalRead(DROGUE_PIN);
        main_chute_pin_state = digitalRead(MAIN_CHUTE_EJECT_PIN);
        
        // Read battery voltage from pin 35 (with voltage divider)
        battery_voltage = (analogRead(35) * 3.3 * 2.0) / 4095.0; // Adjust multiplier based on your voltage divider
        
        // 🔥 ISOLATED RSSI HANDLING - Separate WiFi and Beacon RSSI logic
        if (comm_manager.isMQTTActive() && WiFi.isConnected()) {
            // MQTT mode: Use actual WiFi RSSI from flight computer
            wifi_rssi = WiFi.RSSI();
        } else {
            // Beacon mode: Send 0 - ESP32 base station will measure and override with actual beacon RSSI
            wifi_rssi = 0; // Flight computer sends 0, base station captures real beacon signal strength
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}



/*!****************************************************************************
 * @brief performs flight actions based on the current flight state

 * If the flight state requires an action, we perform it here
 * For example if the flight state is apogee, we perform MAIN_CHUTE ejection
 * 
 *******************************************************************************/
void flightStateCallback(void* pvParameters) {
    static ARMED_FLIGHT_STATE last_state = ARMED_FLIGHT_STATE::PRE_FLIGHT_GROUND;

    while(1) {
        // Check if state changed and handle communication mode locking
        if (current_state != last_state) {
            // Lock communication modes during active flight (powered flight through drogue descent)
            if (current_state == ARMED_FLIGHT_STATE::POWERED_FLIGHT) {
                comm_manager.lockCommunicationMode(true);
                debugln("[FLIGHT STATE] Communication modes locked for flight");
            }
            // Unlock after main chute deployment when flight is essentially over
            else if (current_state == ARMED_FLIGHT_STATE::MAIN_DESCENT && 
                     last_state != ARMED_FLIGHT_STATE::MAIN_DESCENT) {
                comm_manager.lockCommunicationMode(false);
                debugln("[FLIGHT STATE] Communication modes unlocked after main deployment");
            }
            
            last_state = current_state;
        }
        
        switch (current_state) {
            // PRE_FLIGHT_GROUND
            case ARMED_FLIGHT_STATE::PRE_FLIGHT_GROUND:
                //debugln("PRE-FLIGHT STATE");
                break;

            // POWERED_FLIGHT
            case ARMED_FLIGHT_STATE::POWERED_FLIGHT:
                //debugln("POWERED FLIGHT STATE");
                break;

            // COASTING
            case ARMED_FLIGHT_STATE::COASTING:
            //    debugln("COASTING");
                break;

            // APOGEE
            case ARMED_FLIGHT_STATE::APOGEE:
                //debugln("APOGEE");
                break;

            // DROGUE_DEPLOY
            case ARMED_FLIGHT_STATE::DROGUE_DEPLOY:
                /* fire charges ony if the flight computer has been armed */
                if(operation_mode == OPERATION_MODE::ARMED_MODE) {
                    drogueChuteDeploy();
                }

                break;

            // DROGUE_DESCENT
            case ARMED_FLIGHT_STATE::DROGUE_DESCENT:
                break;

            // MAIN_DEPLOY
            case ARMED_FLIGHT_STATE::MAIN_DEPLOY:
                if(operation_mode == OPERATION_MODE::ARMED_MODE) {
                    mainChuteDeploy();
                }

                break;

            // MAIN_DESCENT
            case ARMED_FLIGHT_STATE::MAIN_DESCENT:
            //    debugln("MAIN CHUTE DESCENT");
                break;

            // POST_FLIGHT_GROUND
            case ARMED_FLIGHT_STATE::POST_FLIGHT_GROUND:
            //    debugln("POST FLIGHT GROUND");
                break;
            
            // MAINTAIN AT PRE_FLIGHT_GROUND IF NO STATE IS SPECIFIED - NOT GONNA HAPPEN BUT BETTER SAFE THAN SORRY
            default:
                debugln(current_state);
                break;

        }
        
        vTaskDelay(CONSUME_TASK_DELAY / portTICK_PERIOD_MS);
    }
}

/*!****************************************************************************
 * @brief debug flight/test data to terminal, this task is called if the DEBUG_TO_TERMINAL is set to 1 (see defs.h)
 * @param pvParameter - A value that is passed as the parameter to the created task.
 * If pvParameter is set to the address of a variable then the variable must still exist when the created task executes - 
 * so it is not valid to pass the address of a stack variable.
 * 
 *******************************************************************************/
void debugToTerminalTask(void* pvParameters){
    telemetry_type_t telemetry_received_packet; // acceleration received from acceleration_queue

    while(true){
        // get telemetry data
        xQueueReceive(debug_to_term_queue_handle, &telemetry_received_packet, 0);
        
        // Create unified 25-field CSV format with Kalman filter outputs
        sprintf(telemetry_packet_buffer,
                "%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.8f,%.8f,%.2f,%u,%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f,%d,%.2f,%.2f\n",
                telemetry_received_packet.record_number,    // 0
                telemetry_received_packet.operation_mode,   // 1  
                telemetry_received_packet.state,            // 2
                telemetry_received_packet.acc_data.ax,      // 3
                telemetry_received_packet.acc_data.ay,      // 4
                telemetry_received_packet.acc_data.az,      // 5
                telemetry_received_packet.acc_data.pitch,   // 6
                telemetry_received_packet.acc_data.roll,    // 7
                telemetry_received_packet.gyro_data.gx,     // 8
                telemetry_received_packet.gyro_data.gy,     // 9
                telemetry_received_packet.gyro_data.gz,     // 10
                gps_packet.latitude,                        // 11
                gps_packet.longitude,                       // 12
                gps_packet.gps_altitude,                    // 13
                gps_packet.time,                            // 14 - GPS time
                altimeter_packet.pressure,                  // 15
                altimeter_packet.temperature,               // 16
                altimeter_packet.rel_altitude,              // 17
                altimeter_packet.velocity,                  // 18 - velocity
                telemetry_received_packet.drogue_pin_state, // 19
                telemetry_received_packet.main_chute_pin_state, // 20
                telemetry_received_packet.battery_voltage,  // 21 - battery voltage
                telemetry_received_packet.wifi_rssi,        // 22 - RSSI from telemetry packet
                altimeter_packet.kalman_altitude,           // 23 - 2D Kalman filtered altitude
                altimeter_packet.kalman_vertical_velocity   // 24 - 2D Kalman filtered vertical velocity
            );       
        
        // 🔥 UPDATE GLOBAL TELEMETRY BUFFER for seamless mode switching
        updateGlobalTelemetryBuffer(telemetry_packet_buffer);
        
        // 🔥 QUEUE CSV DATA FOR LOGGING - Send formatted CSV to logger queue
        if (csv_log_queue_handle != NULL) {
            // Create a copy of the CSV string for the queue (max 256 chars)
            char csv_copy[256];
            strncpy(csv_copy, telemetry_packet_buffer, sizeof(csv_copy) - 1);
            csv_copy[sizeof(csv_copy) - 1] = '\0';
            
            // Send to CSV logging queue (non-blocking to avoid task delays)
            if (xQueueSend(csv_log_queue_handle, csv_copy, 0) == pdTRUE) {
                Serial.println("✅ CSV data queued for logging");
            } else {
                Serial.println("❌ CSV queue full - data dropped");
            }
        }
        
        // Only transmit via beacon if beacon mode is active and MQTT mode is NOT
        bool beacon_success = false;
        if (comm_manager.isBeaconActive() && !comm_manager.isMQTTActive()) {
            if (is_system_armed || TEST) {
                beacon_success = transmitter.sendBeacon(telemetry_packet_buffer, strlen(telemetry_packet_buffer));
                if (beacon_success) {
                    debugln("[BEACON TX] " + String(telemetry_packet_buffer));
                } else {
                    debugln("[BEACON TX] Failed to send beacon");
                }
            } else {
                debugln("[BEACON DEBUG] " + String(telemetry_packet_buffer));
                beacon_success = true; // Not a failure, just not sending due to arm state
            }
        }
        
        // Update communication manager with beacon transmission status
        comm_manager.updateTransmissionStatus(false, beacon_success);
        
        vTaskDelay(CONSUME_TASK_DELAY / portTICK_PERIOD_MS);
    }
}


/*!****************************************************************************
 * @brief log the data to the external flash memory
 * @param pvParameter - A value that is passed as the paramater to the created task.
 * If pvParameter is set to the address of a variable then the variable must still exist when the created task executes - 
 * so it is not valid to pass the address of a stack variable.
 * 
 *******************************************************************************/
// void logToMemory(void* pvParameter) {
//     telemetry_type_t received_packet;

//     while(1) {
//         xQueueReceive(log_to_mem_queue_handle, &received_packet, portMAX_DELAY);

//         // received_packet.record_number++; 

//         // is it time to record?
//         current_log_time = millis();

//         if(current_log_time - previous_log_time > log_sample_interval) {
//             previous_log_time = current_log_time;
//             data_logger.loggerWrite(received_packet);
//         }
        
//     }

// }
void logToMemory(void* pvParameter) {
    telemetry_type_t received_packet;
    char csv_data[256];  // Buffer for CSV string data

    while (1) {
        bool data_received = false;
        
        // 🔥 PRIORITY 1: Check for CSV string data (optimized logging)
        if (xQueueReceive(csv_log_queue_handle, csv_data, 0) == pdTRUE) {
            current_log_time = millis();
            
            if (current_log_time - previous_log_time > log_sample_interval) {
                previous_log_time = current_log_time;
                
                #if ENABLE_FLASH_LOGGING
                // 🛡️ MEMORY PROTECTION: Add safety checks before flash operations
                Serial.println("📝 Processing CSV data for flash logging...");
                
                // Verify CSV data integrity
                if (strlen(csv_data) > 0 && strlen(csv_data) < 256) {
                    disableAllDevices();
                    digitalWrite(flash_cs_pin, LOW);
                    
                    // Add small delay for flash chip select stabilization
                    delayMicroseconds(10);
                    
                    data_logger.loggerWriteCSV(csv_data);
                    
                    // Add small delay before releasing chip select
                    delayMicroseconds(10);
                    digitalWrite(flash_cs_pin, HIGH);
                } else {
                    Serial.println("❌ [CSV LOG] Invalid CSV data - skipping flash write");
                }
                #endif
                
                // Log to SD Card using the existing telemetry_type_t method
                // Note: We still use the traditional method for SD card for compatibility
                // The CSV logging is primarily for high-speed flash memory logging
            }
            
            data_received = true;
        }
        
        // 🔥 PRIORITY 2: Check for traditional telemetry packet data (fallback)
        if (!data_received && xQueueReceive(log_to_mem_queue_handle, &received_packet, 0) == pdTRUE) {
            current_log_time = millis();

            if (current_log_time - previous_log_time > log_sample_interval) {
                previous_log_time = current_log_time;

                #if ENABLE_FLASH_LOGGING
                //  Log to Flash Memory (controlled by flag to prevent performance issues)
                disableAllDevices();
                digitalWrite(flash_cs_pin, LOW);
                data_logger.loggerWrite(received_packet);
                digitalWrite(flash_cs_pin, HIGH);
                #endif

                //  Log to SD Card (always enabled for primary data backup)
                disableAllDevices();
                digitalWrite(SD_CS_PIN, LOW);
                sdLogger.log(received_packet, gps_packet);
                digitalWrite(SD_CS_PIN, HIGH);
            }
            
            data_received = true;
        }
        
        // If no data was received, yield to other tasks
        if (!data_received) {
            vTaskDelay(pdMS_TO_TICKS(10));
        } else {
            // Yield to other tasks to prevent blocking beacon transmission
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

/*!****************************************************************************
 * @brief send flight data to ground
 * @param pvParameter - A value that is passed as a the parameter to the created task.
 * If pvParameter is set to the address of a variable then the variable must still exist when the created task executes -
 * so it is not valid to pass the address of a stack variable.
 *
 *******************************************************************************/
void MQTT_TransmitTelemetry(void* pvParameters) {
    telemetry_type_t telemetry_received_packet;

    while(1) {
        xQueueReceive(telemetry_data_queue_handle, &telemetry_received_packet, portMAX_DELAY);

        // Create comprehensive 25-field CSV string for MQTT transmission with Kalman filter data
        sprintf(telemetry_packet_buffer,
                "%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.8f,%.8f,%.2f,%u,%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f,%d,%.2f,%.2f\n",
                telemetry_received_packet.record_number,    // 0
                telemetry_received_packet.operation_mode,   // 1  
                telemetry_received_packet.state,            // 2
                telemetry_received_packet.acc_data.ax,      // 3
                telemetry_received_packet.acc_data.ay,      // 4
                telemetry_received_packet.acc_data.az,      // 5
                telemetry_received_packet.acc_data.pitch,   // 6
                telemetry_received_packet.acc_data.roll,    // 7
                telemetry_received_packet.gyro_data.gx,     // 8
                telemetry_received_packet.gyro_data.gy,     // 9
                telemetry_received_packet.gyro_data.gz,     // 10
                gps_packet.latitude,                        // 11
                gps_packet.longitude,                       // 12
                gps_packet.gps_altitude,                    // 13
                gps_packet.time,                            // 14 - GPS time
                altimeter_packet.pressure,                  // 15
                altimeter_packet.temperature,               // 16
                altimeter_packet.rel_altitude,              // 17
                altimeter_packet.velocity,                  // 18 - velocity
                telemetry_received_packet.drogue_pin_state, // 19
                telemetry_received_packet.main_chute_pin_state, // 20
                telemetry_received_packet.battery_voltage,  // 21 - battery voltage
                telemetry_received_packet.wifi_rssi,        // 22 - RSSI from telemetry packet
                altimeter_packet.kalman_altitude,           // 23 - 2D Kalman filtered altitude
                altimeter_packet.kalman_vertical_velocity   // 24 - 2D Kalman filtered vertical velocity
                );
        // 🔥 UPDATE GLOBAL TELEMETRY BUFFER for seamless mode switching
        updateGlobalTelemetryBuffer(telemetry_packet_buffer);

        // 🔥 ISOLATED MQTT TRANSMISSION - Only transmit via MQTT when MQTT mode is active
        bool mqtt_success = false;
        if (comm_manager.isMQTTActive()) {
            #if BEACON_MODE_SAFETY_CHECKS
            // Additional safety check - ensure MQTT flag is enabled
            if (MQTT == 0) {
                debugln("[MQTT TX] MQTT disabled by flag - skipping transmission");
                mqtt_success = false;
            } else 
            #endif
            // Check WiFi connection status for MQTT mode
            if (!WiFi.isConnected()) {
                debugln("[MQTT TX] WiFi not connected - transmission failed");
                mqtt_success = false;
            }
            // Only send data if armed OR if in test mode
            else if (is_system_armed || TEST) {
                if (client.publish(MQTT_TELEMETRY_TOPIC, telemetry_packet_buffer)) {
                    debugln("[MQTT TX] " + String(telemetry_packet_buffer));
                    mqtt_success = true;
                } else {
                    debugln("[MQTT TX] Failed to publish data");
                    mqtt_success = false;
                }
            } else {
                debugln("[MQTT DEBUG] " + String(telemetry_packet_buffer));
                mqtt_success = true; // Not a failure, just not sending due to arm state
            }
        }
        // Update communication manager with MQTT transmission status
        comm_manager.updateTransmissionStatus(mqtt_success, false);

        vTaskDelay(CONSUME_TASK_DELAY/ portTICK_PERIOD_MS);
    }
}

/*!
 * @brief Try reconnecting to MQTT if connection is lost
 *
 */
void MQTT_Reconnect() {
    // while(1) {
        if(!client.connected()) {
            debugln("[..]Attempting MQTT connection..."); // TODO: SYS LOGGER
            if (client.connect("FC")) {
                debugln("[+]MQTT reconnected");
                client.subscribe("n4/commands"); // TODO: USE DEFINE here
                mqtt_connect_flag = 1;
            } else {
                mqtt_connect_flag = 0;
                debug("failed, rc=");
                debugln(client.state());
                vTaskDelay(1000/portTICK_PERIOD_MS);
            }
        }
    // }
}

// This function is called whenever an MQTT message is received
void mqtt_Callback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    const char* command = message.c_str();
    mqtt_command_processor(topic, command);
}

/*!****************************************************************************
 * @brief Initialize MQTT
 *
 *******************************************************************************/
void MQTTInit(const char* broker_IP, int broker_port) {
    // client.setBufferSize(MQTT_BUFFER_SIZE);
    debugln("[+]Initializing MQTT\n");
    client.setServer(broker_IP, broker_port);
    client.setCallback(mqtt_Callback);
    debugln("MQTT callback hooked!");
    delay(1000);
    debugln("[+]MQTT init OK");
}

/*!****************************************************************************
 * @brief blinks green LED for safe mode and red LED for armed mode
 *******************************************************************************/
void xOperationModeIndicateTask(void* pvParameters) {
    while(1)
    {
        if (operation_mode) {
            /* armed */
            digitalWrite(RED_LED_PIN, HIGH);
            vTaskDelay(BLINK_INTERVALS::ARMED_BLINK);
            digitalWrite(RED_LED_PIN, LOW);
            vTaskDelay(BLINK_INTERVALS::ARMED_BLINK);
        } else if(!operation_mode) {
            /* safe */
            digitalWrite(GREEN_LED_PIN, HIGH);
            vTaskDelay(BLINK_INTERVALS::SAFE_BLINK);
            digitalWrite(GREEN_LED_PIN, LOW);
            vTaskDelay(BLINK_INTERVALS::SAFE_BLINK);
        }
    }
}

/*!****************************************************************************
 * @brief fires the pyro-charge to deploy the drogue chute
 * Turn on the drogue chute ejection circuit by running the GPIO 
 * HIGH for a preset No. of seconds.  
 * Default no. of seconds to remain HIGH is 5 
 * 
 *******************************************************************************/
//void drogueChuteDeploy() {

    // // check for drogue chute deploy conditions 

    // //if the drogue deploy pin is HIGH, there is an error
    // if(digitalRead(DROGUE_PIN)) {
    //     // error
    // } else {
    //     // pulse the drogue pin for a number ofseceonds - determined from pop tests
    //     digitalWrite(DROGUE_PIN, HIGH);
    //     delay(PYRO_CHARGE_TIME); // TODO- Make this delay non-blocking

    //     // update the drogue deployed telemetry variable
    //     DROGUE_DEPLOY_FLAG = 1;
    //     debugln("DROGUE CHUTE DEPLOYED");
    // }

//}

void drogueChuteDeploy() {
    static bool firing = false;
    static unsigned long fireStart = 0;

    if (!firing) {
        // Start firing
        digitalWrite(DROGUE_PIN, HIGH);
        fireStart = millis();
        firing = true;
        debugln("DROGUE CHUTE EJECTION STARTED");
    }

    // Keep pin HIGH for the required time, then turn off
    if (firing && (millis() - fireStart >= PYRO_CHARGE_TIME)) {
        digitalWrite(DROGUE_PIN, LOW);
        DROGUE_DEPLOY_FLAG = 1;
        firing = false;
        debugln("DROGUE CHUTE DEPLOYED");
    }
}

/*!****************************************************************************
 * @brief fires the pyro-charge to deploy the main chute
 * Turn on the main chute ejection circuit by running the GPIO
 * HIGH for a preset No. of seconds
 * Default no. of seconds to remain HIGH is 5
 * 
 *******************************************************************************/
//void mainChuteDeploy() {
    // // check for drogue chute deploy conditions 

    // //if the drogue deploy pin is HIGH, there is an error
    // if(digitalRead(MAIN_CHUTE_EJECT_PIN)) { // NOT NECESSARY - TEST WITHOUT
    //     // error 
    // } else {
    //     // pulse the drogue pin for a number of seconds - determined from pop tests
    //     digitalWrite(MAIN_CHUTE_EJECT_PIN, HIGH);
    //     delay(MAIN_DESCENT_PYRO_CHARGE_TIME); // TODO- Make this delay non-blocking

    //     // update the drogue deployed telemetry variable
    //     MAIN_CHUTE_EJECT_FLAG = 1;
    //     debugln("MAIN CHUTE DEPLOYED");
    // }
//}

void mainChuteDeploy() {
    static bool firing = false;
    static unsigned long fireStart = 0;

    if (!firing) {
        digitalWrite(MAIN_CHUTE_EJECT_PIN, HIGH);
        fireStart = millis();
        firing = true;
        debugln("MAIN CHUTE EJECTION STARTED");
    }

    if (firing && (millis() - fireStart >= MAIN_DESCENT_PYRO_CHARGE_TIME)) {
        digitalWrite(MAIN_CHUTE_EJECT_PIN, LOW);
        MAIN_CHUTE_EJECT_FLAG = 1;
        firing = false;
        debugln("MAIN CHUTE DEPLOYED");
    }
}


void xCreateAllTasks() {
    debugln("Creating all tasks with enhanced error protection");
    
    // 🛡️ MEMORY CHECK: Verify available heap before task creation
    size_t free_heap_before = esp_get_free_heap_size();
    Serial.printf("[TASK CREATION] Available heap before tasks: %d bytes\n", free_heap_before);
    
    if (free_heap_before < 50000) { // Less than 50KB free
        Serial.println("[ERROR] Insufficient memory for task creation - halting");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::ERROR, system_log_file, "CRITICAL: Insufficient memory for task creation\r\n");
        return;
    }
    
    uint8_t tasks_created = 0;
    uint8_t tasks_failed = 0;

    /* 🛡️ READ ACCELERATION DATA - with memory protection */
    BaseType_t gr = xTaskCreatePinnedToCore(readAccelerationTask, "readAccelerometer", STACK_SIZE*4, NULL, 2, &readAccelerationTaskHandle, 1);
    if(gr == pdPASS) {
        tasks_created++;
        debugln("[+]Read acceleration task created OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]Read acceleration task created OK.\r\n");
    } else {
        tasks_failed++;
        debugln("[-]Read acceleration task creation failed - CRITICAL");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::ERROR, system_log_file, "[CRITICAL]Read acceleration task creation failed\r\n");
    }

    /* 🛡️ TASK 3: READ GPS DATA - with error protection */
    BaseType_t rg = xTaskCreatePinnedToCore(readGPSTask, "readGPS", STACK_SIZE*2, NULL, 2, &readGPSTaskHandle, 1);
    if(rg == pdPASS) {
        tasks_created++;
        debugln("[+]Read GPS task created OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]Read GPS task created OK.\r\n");
    } else {
        tasks_failed++;
        debugln("[-]Failed to create GPS task");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::WARNING, system_log_file, "[-]Failed to create GPS task\r\n");
    }

    /* 🛡️ CHECK FLIGHT STATE TASK - essential for flight safety */
    BaseType_t cf = xTaskCreatePinnedToCore(checkFlightState,"checkFlightState",STACK_SIZE*2,NULL, 2, &checkFlightStateTaskHandle, 1);
    if(cf == pdPASS) {
        tasks_created++;
        debugln("[+]checkFlightState task created OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]checkFlightState task created OK.\r\n");
    } else {
        tasks_failed++;
        debugln("[-]Failed to create checkFlightState task - CRITICAL");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::ERROR, system_log_file, "[CRITICAL]Failed to create checkFlightState task\r\n");
    }

    /* 🛡️ FLIGHT STATE CALLBACK TASK - essential for pyro control */
    BaseType_t fs = xTaskCreatePinnedToCore(flightStateCallback, "flightStateCallback", STACK_SIZE*2, NULL, 2, &flightStateCallbackTaskHandle, 1);
    if(fs == pdPASS) {
        tasks_created++;
        debugln("[+]flightStateCallback task created OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]flightStateCallback task created OK.\r\n");
    } else {
        tasks_failed++;
        debugln("[-]Failed to create flightStateCallback task - CRITICAL");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::ERROR, system_log_file, "[CRITICAL]Failed to create flightStateCallback task\r\n");
    }
    
    /* 🛡️ MONITOR CHUTE PINS TASK - essential for status monitoring */
    BaseType_t mp = xTaskCreatePinnedToCore(monitorChutePinsTask, "monitorChutePins", STACK_SIZE, NULL, 2, NULL, 1);
    if(mp == pdPASS) {
        tasks_created++;
        debugln("[+]monitorChutePinsTask created OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]monitorChutePinsTask created OK.\r\n");
    } else {
        tasks_failed++;
        debugln("[-]Failed to create monitorChutePinsTask");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::WARNING, system_log_file, "[-]Failed to create monitorChutePinsTask\r\n");
    } 

    /* 🛡️ TRANSMIT TELEMETRY DATA - communication essential (only if MQTT enabled) */
    #if BEACON_MODE_SAFETY_CHECKS
    if (MQTT == 1) {
    #endif
        BaseType_t th = xTaskCreatePinnedToCore(MQTT_TransmitTelemetry, "transmit_telemetry", STACK_SIZE*4, NULL, 2, &MQTT_TransmitTelemetryTaskHandle, 1);
        if(th == pdPASS){
            tasks_created++;
            debugln("[+]MQTT transmit task created OK");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]MQTT transmit task created OK\r\n");
        } else {
            tasks_failed++;
            debugln("[-]MQTT transmit task failed to create");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::WARNING, system_log_file, "[-]MQTT transmit task failed to create\r\n");
        }
    #if BEACON_MODE_SAFETY_CHECKS
    } else {
        debugln("[+] MQTT transmit task skipped - strict beacon mode");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "MQTT transmit task skipped - strict beacon mode\r\n");
    }
    #endif

    /* 🛡️ KALMAN FILTER 2D TASK - essential for altitude estimation */
    BaseType_t kf2d = xTaskCreatePinnedToCore(taskKalman2D, "Kalman2D", STACK_SIZE*4, NULL, 2, NULL, 1);
    if(kf2d == pdPASS) {
        tasks_created++;
        debugln("[+]Kalman2D task created OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]Kalman2D task created OK.\r\n");
    } else {
        tasks_failed++;
        debugln("[-]Kalman2D task creation failed - CRITICAL");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::ERROR, system_log_file, "[CRITICAL]Kalman2D task creation failed\r\n");
    }

    #if DEBUG_TO_TERMINAL   // set DEBUG_TO_TERMINAL to 0 to prevent serial debug data to serial monitor
        /* 🛡️ TASK 7: DISPLAY DATA ON SERIAL MONITOR - FOR DEBUGGING */
        BaseType_t dt = xTaskCreatePinnedToCore(debugToTerminalTask,"debugToTerminalTask",STACK_SIZE*4, NULL,2,&debugToTerminalTaskHandle, 1);
        if(dt == pdPASS) {
            tasks_created++;
            debugln("[+]debugToTerminal task created OK");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]debugToTerminal task created OK\r\n");
        } else {
            tasks_failed++;
            debugln("[-]debugToTerminal task not created");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::WARNING, system_log_file, "[-]debugToTerminal task not created\r\n");
        }
    #endif // DEBUG_TO_TERMINAL_TASK

    #if LOG_TO_MEMORY   // set LOG_TO_MEMORY to 1 to allow logging to memory 
        /* 🛡️ TASK 9: LOG DATA TO MEMORY */
        if(xTaskCreatePinnedToCore(logToMemory,"logToMemory",STACK_SIZE*4,NULL,2,&logToMemoryTaskHandle,1) != pdPASS){
            tasks_failed++;
            debugln("[-]logToMemory task failed to create");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::WARNING, system_log_file, "[-]logToMemory task failed to create\r\n");
        }else{
            tasks_created++;
            debugln("[+]logToMemory task created OK.");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]logToMemory task created OK.\r\n");
        }
    #endif // LOG_TO_MEMORY

    /* 🛡️ OPERATION MODE INDICATION TASK - essential for user feedback */
    if(xTaskCreatePinnedToCore(xOperationModeIndicateTask,"xOperationModeIndicateTask",STACK_SIZE*2,NULL,2,&opModeIndicateTaskHandle,1) != pdPASS){
        tasks_failed++;
        debugln("[-]xOperationModeIndicateTask task failed to create");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::WARNING, system_log_file, "[-]xOperationModeIndicateTask task failed to create\r\n");
    }else{
        tasks_created++;
        debugln("[+]xOperationModeIndicateTask task created OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]xOperationModeIndicateTask task created OK.\r\n");
    }

    /* 🛡️ READ ALTIMETER DATA - essential for flight state detection */
    BaseType_t ra = xTaskCreatePinnedToCore(readAltimeterTask,"readAltimeter",STACK_SIZE*3,NULL,2, &readAltimeterTaskHandle, 1);
    if(ra == pdPASS) {
        tasks_created++;
        debugln("[+]readAltimeterTask created OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]readAltimeterTask created OK.\r\n");
    } else {
        tasks_failed++;
        debugln("[-]Failed to create readAltimeterTask - CRITICAL");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::ERROR, system_log_file, "[CRITICAL]Failed to create readAltimeterTask\r\n");
    }

    /* 🛡️ Create ESP-NOW command task - will be managed by communication manager */
    BaseType_t ec = xTaskCreatePinnedToCore(
        espnowCommandTask,
        "ESPNowCmd",
        STACK_SIZE * 6,  // Increased from 2 to 6 to prevent stack overflow
        NULL,
        2,
        &espnowCommandTaskHandle,
        1
    );

    if (ec == pdPASS) {
        tasks_created++;
        debugln("[+]ESPNowCmd task created OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]ESPNowCmd task created OK.\r\n");
    } else {
        tasks_failed++;
        debugln("[-]Failed to create ESPNowCmd task - CRITICAL");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::ERROR, system_log_file, "[CRITICAL]Failed to create ESPNowCmd task\r\n");
    }

    // 🛡️ MEMORY CHECK AFTER TASK CREATION
    size_t free_heap_after = esp_get_free_heap_size();
    size_t heap_used = free_heap_before - free_heap_after;
    
    Serial.printf("[TASK CREATION SUMMARY] Tasks created: %d, Failed: %d\n", tasks_created, tasks_failed);
    Serial.printf("[MEMORY USAGE] Heap before: %d, After: %d, Used: %d bytes\n", 
                  free_heap_before, free_heap_after, heap_used);
    
    if (tasks_failed > 0) {
        Serial.printf("[WARNING] %d tasks failed to create - system may be unstable\n", tasks_failed);
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::WARNING, system_log_file, 
                               ("Task creation failures: " + String(tasks_failed) + "\r\n").c_str());
    }
    
    if (free_heap_after < 20000) { // Less than 20KB remaining
        Serial.println("[WARNING] Low memory after task creation - monitoring required");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::WARNING, system_log_file, "Low memory after task creation\r\n");
    }

    debugln();
    debugln(F("=============================================="));
    debugln(F("========== FINISHED CREATING TASKS ==========="));
    debugln(F("==============================================\n"));
    
    // 🛡️ WATCHDOG PROTECTION: Reset watchdog after task creation
    esp_task_wdt_reset();
}

/*!****************************************************************************
 * @brief Setup - perform initialization of all hardware subsystems, create queues, create queue handles
 * initialize system check table
 * 
 *******************************************************************************/
void setup() {
    /* initialize serial */
    Serial.begin(BAUDRATE);

    debugln("=========INITIALIZING FLIGHT COMPUTER============");
    LED_init();
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, LOW);
    buzzerInit();

    // Initialize operation mode to SAFE_MODE by default
    operation_mode = OPERATION_MODE::SAFE_MODE;
    /* buzz to indicate start of setup */
    blocking_buzz(BUZZ_INTERVALS::SETUP_INIT);

    /* core to run the tasks */
    uint8_t app_core_id = xPortGetCoreID();

    // SPIFFS Must be initialized first to allow event logging from the word go
    uint8_t spiffs_init_state = InitSPIFFS();

    // SYSTEM LOG FILE
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::WRITE, "FC1", LOG_LEVEL::INFO, system_log_file, "Flight computer Event log\r\n");

    debugln();
    debugln(F("=============================================="));
    debugln(F("========= CREATING DYNAMIC WIFI ==========="));
    debugln(F("=============================================="));
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "==CREATING DYNAMIC WIFI==\r\n");
    
    #if BEACON_MODE_SAFETY_CHECKS
    // Safety check: Only initialize WiFi/MQTT if MQTT flag is enabled
    if (MQTT == 1) {
        debugln("[+] MQTT enabled - initializing WiFi and MQTT");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "MQTT enabled - full WiFi init\r\n");
        initDynamicWIFI(); // TODO - uncomment on live testing and production
    } else {
        debugln("[+] MQTT disabled - strict beacon mode (WiFi setup for ESP-NOW only)");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "Strict beacon mode - ESP-NOW WiFi setup\r\n");
        // Initialize WiFi for ESP-NOW in beacon-only mode using existing beacon setup
        uint8_t wifi_result = wifi_config.WifiConnect(true, ROCKET_MAC); // use_beacon_mode = true
        if(wifi_result) {
            debugln("[+] WiFi initialized for ESP-NOW beacon mode");
        } else {
            debugln("[-] WiFi beacon mode initialization failed");
        }
    }
    #else
    initDynamicWIFI(); // TODO - uncomment on live testing and production
    #endif

    #if MQTT

    // create and wait for dynamic WIFI connection - only if MQTT is enabled
    #if BEACON_MODE_SAFETY_CHECKS
    if (MQTT == 1) {
    #endif
        MQTTInit(wifi_config.getBaseStationIP(), wifi_config.getMQTTPort());
        MQTT_Reconnect();  
        debugln("[+]Dynamic WIFI created OK.");
    #if BEACON_MODE_SAFETY_CHECKS
    } else {
        debugln("[+] MQTT initialization skipped - running in strict beacon mode");
    }
    #endif

    #endif // MQTT


    debugln();
    debugln(F("=============================================="));
    debugln(F("========= INITIALIZING PERIPHERALS ==========="));
    debugln(F("=============================================="));
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "==Initializing peripherals==\r\n");
    
    // Initialize transmitter for beacon capability (will be managed by communication manager)
    // ESP-NOW requires WiFi to be initialized first (done above)
    debugln("[DEBUG] Starting ESP-NOW transmitter initialization...");
    if (transmitter.begin()) {
        debugln("[+] ESP-NOW transmitter initialized for communication manager");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "ESP-NOW transmitter initialized for comm manager\r\n");
    } else {
        debugln("[-] ESP-NOW transmitter initialization failed - beacon functionality disabled");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::WARNING, system_log_file, "ESP-NOW transmitter init failed\r\n");
    }
    
    // 🔥 INITIALIZE COMMUNICATION MANAGER for isolated mode control
    debugln("[DEBUG] Starting Communication Manager initialization...");
    comm_manager.init();
    debugln("[+] Communication Manager initialized with isolated mode control");
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "Communication Manager initialized\r\n");
    
    debugln("[DEBUG] Starting BMP initialization...");
    uint8_t bmp_init_state = BMPInit();
    debugln("[DEBUG] Starting IMU initialization...");
    uint8_t imu_init_state = imu.init();
    debugln("[DEBUG] Starting GPS initialization...");
    uint8_t gps_init_state = GPSInit();
    debugln("[DEBUG] Disabling all devices...");
    disableAllDevices();
    debugln("[DEBUG] Starting SD initialization...");
    uint8_t sd_init_state = initSD();
    debugln("[DEBUG] Disabling all devices again...");
    disableAllDevices();
    debugln("[DEBUG] Starting flash logger initialization...");
    #if ENABLE_FLASH_LOGGING
    uint8_t flash_init_state = data_logger.loggerInit();
    debug("Flash memory init state:"); debugln(flash_init_state);
    #else
    debugln("[DEBUG] Flash logging disabled - skipping flash logger initialization");
    uint8_t flash_init_state = 1; // Set to success since we're not using it
    #endif
    debugln(F("=============================================="));
    Serial.print("Available heap: ");
    debugln(F("=============================================="));
    Serial.println(esp_get_free_heap_size());
    debugln(F("=============================================="));
    debugln(F("=============================================="));
    Serial.print("Task stack watermark: ");
    debugln(F("=============================================="));
    Serial.println(uxTaskGetStackHighWaterMark(NULL));
    debugln(F("=============================================="));
    //For Debugging to be deleted
    esp_task_wdt_init(10, true);  // Set timeout to 10 seconds instead of default 5

    /* initialize mqtt */
    //MQTTInit(MQTT_SERVER, MQTT_PORT);

    /* update the sub-systems init state table */
    // check if BMP init OK
    // if(bmp_init_state) { 
    //     SUBSYSTEM_INIT_MASK |= (1 << BMP_CHECK_BIT);
    // }

    // // check if MPU init OK
    // if(imu_init_state)  {
    //     SUBSYSTEM_INIT_MASK |= (1 << IMU_CHECK_BIT);
    // }

    // // check if flash memory init OK
    // if (flash_init_state) {
    //     SUBSYSTEM_INIT_MASK |= (1 << FLASH_CHECK_BIT);
    // }

    // // check if GPS init OK
    // if(gps_init_state) {
    //     SUBSYSTEM_INIT_MASK |= (1 << GPS_CHECK_BIT);
    // }

    // // check if SD CARD init OK
    // if(sd_init_state) {
    //     SUBSYSTEM_INIT_MASK |= (1 << SD_CHECK_BIT);
    // } 

    // // check if SPIFFS init OK
    // if(spiffs_init_state) {
    //     SUBSYSTEM_INIT_MASK |= (1 << SPIFFS_CHECK_BIT);
    // }

    /* register the baseline pressure at launch site - check docs to see how this works */
    debugln(F("==============================================="));
    debugln(F("==================== BASELINE ================="));
    debugln(F("=============================================="));
    baseline = altimeter_get_pressure();
    Serial.print("\n");Serial.print("Baseline ");Serial.print(baseline);Serial.print("\n");
   

    /* initialize the ring buffer - used for apogee detection */
    ring_buffer_init(&altitude_ring_buffer);

    //Initialize Kalman Matrices 
    init_kalman_matrices();

    /* check whether we are in TEST or RUN mode */
    checkRunTestToggle();

    // TODO: if toggle pin in RUN mode, set to wait for arming 

    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "RUN MODE\r\n");
    
    debugln();
    debugln(F("=============================================="));
    debugln(F("===== INITIALIZING COMMUNICATION MANAGER ===="));
    debugln(F("=============================================="));
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "==INITIALIZING COMMUNICATION MANAGER==\r\n");
    
    // Initialize the smart communication manager
    comm_manager.init();
    debugln("[+] Communication Manager initialized");
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+] Communication Manager initialized\r\n");

    /* mode 0 resets the system log file by clearing all the current contents */
    // system_logger.logToFile(SPIFFS, 0, rocket_ID, level, system_log_file, "Game Time!"); // TODO: DEBUG

    debugln();
    debugln(F("=============================================="));
    debugln(F("===== INITIALIZING DATA LOGGING SYSTEM ======="));
    debugln(F("=============================================="));
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "==INITIALIZING DATA LOGGING SYSTEM==\r\n");
    sdLogger.begin();
    debug("===== SDlogger initialization =======");
        
    debugln();
    debugln(F("=============================================="));
    debugln(F("============== CREATING QUEUES ==============="));
    debugln(F("=============================================="));
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "==CREATING QUEUES==\r\n");

    /* Every producer task sends queue to a different queue to avoid data popping issue */
    telemetry_data_queue_handle = xQueueCreate(TELEMETRY_DATA_QUEUE_LENGTH, sizeof(telemetry_type_t));
    log_to_mem_queue_handle = xQueueCreate(LOG_TO_MEM_QUEUE_LENGTH, sizeof(telemetry_type_t));  // Use large queue for flash logging
    csv_log_queue_handle = xQueueCreate(LOG_TO_MEM_QUEUE_LENGTH, 256);  // 🔥 NEW: CSV string queue (256 char strings)
    check_state_queue_handle = xQueueCreate(TELEMETRY_DATA_QUEUE_LENGTH, sizeof(telemetry_type_t));
    debug_to_term_queue_handle = xQueueCreate(TELEMETRY_DATA_QUEUE_LENGTH, sizeof(telemetry_type_t));
    kalman_filter_queue_handle = xQueueCreate(TELEMETRY_DATA_QUEUE_LENGTH, sizeof(telemetry_type_t));
    kalman2d_input_queue_handle = xQueueCreate(TELEMETRY_DATA_QUEUE_LENGTH, sizeof(float));

    if(telemetry_data_queue_handle == NULL) {
        debugln("[-]telemetry_data_queue_handle creation failed");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]telemetry_data_queue_handle creation failed\r\n");
    } else {
        debugln("[+]telemetry_data_queue_handle creation OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]telemetry_data_queue_handle creation OK.\r\n");
    }

    if(log_to_mem_queue_handle == NULL) {
        debugln("[-]telemetry_data_queue_handle creation failed");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]telemetry_data_queue_handle creation failed\r\n");
    } else {
        debugln("[+]telemetry_data_queue_handle creation OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]telemetry_data_queue_handle creation OK.\r\n");
    }

    // 🔥 CSV LOGGING QUEUE ERROR CHECK
    if(csv_log_queue_handle == NULL) {
        debugln("[-]csv_log_queue_handle creation failed");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]csv_log_queue_handle creation failed\r\n");
    } else {
        debugln("[+]csv_log_queue_handle creation OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]csv_log_queue_handle creation OK.\r\n");
    }

    if(check_state_queue_handle == NULL) {
        debugln("[-]check_state_queue_handle creation failed");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]check_state_queue_handle creation failed\r\n");
    } else {
        debugln("[+]check_state_queue_handle creation OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]check_state_queue_handle creation OK.\r\n");
    }

    if(debug_to_term_queue_handle == NULL) {
        debugln("[-]debug_to_term_queue_handle creation failed");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]debug_to_term_queue_handle creation failed\r\n");
    } else {
        debugln("[+]debug_to_term_queue_handle creation OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]debug_to_term_queue_handle creation OK.\r\n");
    }

    if(kalman_filter_queue_handle == NULL) {
        debugln("[-]kalman_filter_queue_handle creation failed");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]kalman_filter_queue_handle creation failed\r\n");
    } else {
        debugln("[+]kalman_filter_queue_handle creation OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]kalman_filter_queue_handle creation OK.\r\n");
    }

    debugln();
    debugln(F("=============================================="));
    debugln(F("============== CREATING TASKS ==============="));
    debugln(F("==============================================\n"));
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "==CREATING TASKS==\r\n");

    /* Create tasks
    * All tasks have a stack size of 1024 words - not bytes!
    * ESP32 is 32 bit, therefore 32bits x 1024 = 4096 bytes
    * So the stack size is 4096 bytes
    * 
    * TASK CREATION PARAMETERS
    * function that executes this task
    * Function name - for debugging 
    * Stack depth in words 
    * parameter to be passed to the task 
    * Task priority 
    * task handle that can be passed to other tasks to reference the task 
    *
    *
    */


    xCreateAllTasks();


    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "==FINISHED CREATING TASKS==\r\n");
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "\nEND OF INITIALIZATION\r\n");

    /* buzz to indicate start of setup */
    //blocking_buzz(BUZZ_INTERVALS::SETUP_INIT);

    //Simultaneous memory logging prerequisites
    pinMode(flash_cs_pin, OUTPUT);
    pinMode(SD_CS_PIN, OUTPUT);
    disableAllDevices();
    SPI.begin(18, 19, 23);
    
} /* End of setup */


/*!****************************************************************************
 * @brief Main loop
 *******************************************************************************/
void loop() {
    if (MQTT) {
        /* enable MQTT transmit loop */
        if (!client.connected()) {
            MQTT_Reconnect();
        }
        client.loop();
    }

    /* Update communication manager */
    comm_manager.update();
    
    /* Handle incoming commands from serial */
    handleIncomingCommands();

    /* check if the transmitter is armed */
    // Remove this line as it was overriding operation_mode set by ARM/DISARM commands
    // operation_mode = (transmitter.isArmed()) ? OPERATION_MODE::ARMED_MODE : OPERATION_MODE::SAFE_MODE;

    /* LED indication is handled by xOperationModeIndicateTask - no need to duplicate here */
    
    /* Feed the watchdog to prevent timeout */
    esp_task_wdt_reset();
    
    /* Add a small delay to prevent watchdog timeout */
    delay(10);
    
/* End of main loop*/
}