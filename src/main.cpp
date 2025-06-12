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
#include "defs.h"           // misc defines
#include "mpu.h"            // for reading MPU6050
#include "SerialFlash.h"    // Handling external SPI flash memory
#include "logger.h"         // system logging
#include "data_types.h"     // definitions of data types used
#include "states.h"         // state machine states
#include "system_logger.h"  // system logging functions
#include "system_log_levels.h"  // system logging log levels
#include "wifi-config.h"    // handle wifi connection
#include "kalman_filter.h"  // handle kalman filter functions
#include "ring_buffer.h"    // for apogee detection

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

void arm_pyros() {
    digitalWrite(REMOTE_SWITCH, HIGH);
    // todo: confirm arming
}

/**
 *
 */
void disarm_pyros() {
    digitalWrite(REMOTE_SWITCH, LOW);
}

/* state machine variables*/
uint8_t operation_mode = 0;                                         /*!< Tells whether software is in safe or flight mode - FLIGHT_MODE=1, SAFE_MODE=0 */
uint8_t current_state = ARMED_FLIGHT_STATE::PRE_FLIGHT_GROUND;	    /*!< The starting state - we start at PRE_FLIGHT_GROUND state */

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

/**
 * flight states
 * these states are to be used for flight
**/
enum OPERATION_MODE {
    SAFE_MODE = 0, /* Pyro-charges are disarmed  */
    ARMED_MODE      /* Pyro charges are armed and ready to deploy on apogee --see docs for more-- */
};

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

/* Flight data logging */
uint8_t flash_led_pin = 32;                  /*!< LED pin connected to indicate flash memory formatting  */
char filename[] = "flight_data.txt";         /*!< data log filename - Filename must be less than 20 chars, including the file extension */
uint32_t FILE_SIZE_512K = 524288L;          /*!< 512KB */
uint32_t FILE_SIZE_1M  = 1048576L;          /*!< 1MB */
uint32_t FILE_SIZE_4M  = 4194304L;          /*!< 4MB */
SerialFlashFile file;                       /*!< object representing file object for flash memory */
unsigned long long previous_log_time = 0;   /*!< The last time we logged data to memory */
unsigned long long current_log_time = 0;    /*!< What is the processor time right now? */
uint16_t log_sample_interval = 5;          /*!< After how long should we sample and log data to flash memory? */

/* create flash memory log object */
DataLogger data_logger(flash_cs_pin, RED_LED_PIN, filename, file, FILE_SIZE_4M);

/* position integration variables */
long long current_time = 0;
long long previous_time = 0;

/* To store the main telemetry packet being sent over MQTT */
char telemetry_packet_buffer[256];
ring_buffer altitude_ring_buffer;
double baseline = 0.0; // to store baseline pressure from the altimeter
float curr_val;
float oldest_val;
uint8_t apogee_flag =0; // to signal that we have detected apogee
static int apogee_val = 0; // apogee altitude aproximmation
uint8_t main_eject_flag = 0;

/**
* @brief create dynamic WIFI
*/
void initDynamicWIFI() {
    uint8_t wifi_connection_result = wifi_config.WifiConnect();
    if(wifi_connection_result) {
        debugln("Wifi config OK!");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "Wifi config OK!\r\n");
    } else {
        debugln("Wifi config failed");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "Wifi config failed\r\n");
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
void mqtt_command_processor(const char* topic, const char* command)
{
    // check topic
    if(topic == "n4/commands")
    {
      if(command == "ARM")
      {
          arm_pyros();
          operation_mode = 1;
          non_blocking_buzz(BUZZ_INTERVALS::ARMING_PROCEDURE); // IGNORE ARMING PROCEDURE
          debugln("ARM PYRO"); // TODO:log to syslogger

      }  else if(command == "DISARM") {
          disarm_pyros();
          operation_mode = 0;
          non_blocking_buzz(BUZZ_INTERVALS::ARMING_PROCEDURE);
          debugln("ARM PYRO"); // TODO:log to syslogger
      } else if(command == "RESET"){
          // reset ESP via software
          debugln("RESET"); // TODO:log to syslogger
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
    delay(interval);
    digitalWrite(BUZZER_PIN, LOW);
    delay(interval);
}

/**
 * ///////////////////////// END OF PERIPHERALS INIT /////////////////////////
 */
QueueHandle_t telemetry_data_queue_handle;
QueueHandle_t log_to_mem_queue_handle;
QueueHandle_t check_state_queue_handle;
QueueHandle_t debug_to_term_queue_handle;
QueueHandle_t kalman_filter_queue_handle;

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


    while(1) {
        acc_data_lcl.operation_mode = operation_mode; // TODO: move these to check state function
        acc_data_lcl.record_number++;
        acc_data_lcl.state = 0;

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
    
        xQueueSend(telemetry_data_queue_handle, &acc_data_lcl, 0);
        xQueueSend(log_to_mem_queue_handle, &acc_data_lcl, 0);
        xQueueSend(check_state_queue_handle, &acc_data_lcl, 0);
        xQueueSend(debug_to_term_queue_handle, &acc_data_lcl, 0);

        vTaskDelay(CONSUME_TASK_DELAY/ portTICK_PERIOD_MS);
    }

}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// ALTITUDE AND VELOCITY DETERMINATION /////////////////
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
        delay(status);
        status = altimeter.getTemperature(T);
        altimeter_temperature = T;
        if(status != 0)
        {
            status = altimeter.startPressure(3);
            if(status != 0)
            {
                delay(status);
                status = altimeter.getPressure(P, T);
                if(status != 0)
                {
                    return P;
                } else debugln("Error getting pressure");
            } else debugln("error starting pressure");
        } else debugln("error getting temperature");
    } else debugln("error starting pressure measurement");

}

/*!****************************************************************************
 * @brief Read atm pressure data from the barometric sensor onboard
 *******************************************************************************/
void readAltimeterTask(void* pvParameters) {
    telemetry_type_t alt_data_lcl;

    while(1) {

        double a, P;
        P = altimeter_get_pressure();
        a = altimeter.altitude(P, baseline);

        /* send to altimeter global packet */
        altimeter_packet.temperature = altimeter_temperature;
        altimeter_packet.pressure = P;
        altimeter_packet.rel_altitude = a;
    }
}

/*!****************************************************************************
 * @brief Read the GPS location data and altitude and append to telemetry packet for transmission
 * @param pvParameters - A value that is passed as the paramater to the created task.
 * If pvParameters is set to the address of a variable then the variable must still exist when the created task executes - 
 * so it is not valid to pass the address of a stack variable.
 * 
 *******************************************************************************/
void readGPSTask(void* pvParameters){
    float latitude, longitude, g_altitude;

    gps_type_t gps_data_lcl;

    while(1){
        if(gpsSerial.available() > 0) {
            gps.encode(gpsSerial.read());

            /* get GPS coordinates */
            if(gps.location.isValid()) {
                latitude = gps.location.lat();
                longitude = gps.location.lng();
            } 

            /* get GPS altitude */
            if(gps.altitude.isValid()) {
                g_altitude = gps.altitude.meters();
            }
        } else {
            vTaskDelay(10/portTICK_PERIOD_MS);
        }

        gps_packet.latitude = latitude;
        gps_packet.longitude = longitude;
        gps_packet.gps_altitude = g_altitude;
    }
}

/**
 * @brief Kalman filter estimated value calculation
 * 
 */
float kalmanFilter(float z) {
    float estimated_altitude_pred = estimated_altitude;
    float error_covariance_pred = error_covariance_bmp + process_variance_bmp;
    kalman_gain_bmp = error_covariance_pred / (error_covariance_pred + measurement_variance_bmp);
    estimated_altitude = estimated_altitude_pred + kalman_gain_bmp * (z - estimated_altitude_pred);
    error_covariance_bmp = (1 - kalman_gain_bmp) * error_covariance_pred;

    return estimated_altitude;
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

/*!****************************************************************************
 * @brief check various condition from flight data to change the flight state
 * - -see states.h for more info --
 *
 *******************************************************************************/
void checkFlightState(void* pvParameters) {
    // get the flight state from the telemetry task
    telemetry_type_t flight_data; 
    
    while (1) {
        xQueueReceive(check_state_queue_handle, &flight_data, portMAX_DELAY);

        if(apogee_flag != 1) {
            // states before apogee
            // debug("altitude value:"); debugln(flight_data.alt_data.altitude);
            if(flight_data.alt_data.rel_altitude < LAUNCH_DETECTION_THRESHOLD) {
                current_state = ARMED_FLIGHT_STATE::PRE_FLIGHT_GROUND;
                //debugln("PREFLIGHT");
                delay(STATE_CHANGE_DELAY);
            } else if(LAUNCH_DETECTION_THRESHOLD < flight_data.alt_data.rel_altitude < (LAUNCH_DETECTION_THRESHOLD+LAUNCH_DETECTION_ALTITUDE_WINDOW) ) {
                current_state = ARMED_FLIGHT_STATE::POWERED_FLIGHT;
                //debugln("POWERED");
                delay(STATE_CHANGE_DELAY);
            } 

            // COASTING

            // APOGEE and APOGEE DETECTION
            ring_buffer_put(&altitude_ring_buffer, flight_data.alt_data.rel_altitude);
            if(ring_buffer_full(&altitude_ring_buffer) == 1) {
                oldest_val = ring_buffer_get(&altitude_ring_buffer);
            }

            //debug("Curr val:");debug(flight_data.alt_data.altitude); debug("    "); debugln(oldest_val);
            if((oldest_val - flight_data.alt_data.rel_altitude) >= APOGEE_DETECTION_THRESHOLD) {
                if(apogee_flag == 0) {
                    apogee_val = ( (oldest_val - flight_data.alt_data.rel_altitude) / 2 ) + oldest_val;

                    current_state = ARMED_FLIGHT_STATE::APOGEE;
                    delay(STATE_CHANGE_DELAY);
                    //debugln("APOGEE");
                    delay(STATE_CHANGE_DELAY);
                    current_state = ARMED_FLIGHT_STATE::DROGUE_DEPLOY;
                    //debugln("DROGUE");
                    delay(STATE_CHANGE_DELAY);
                    current_state =  ARMED_FLIGHT_STATE::DROGUE_DESCENT;
                    //debugln("DROGUE_DESCENT");
                    delay(STATE_CHANGE_DELAY);
                    apogee_flag = 1;
                }
            }

        } else if(apogee_flag == 1) {
            if(LAUNCH_DETECTION_THRESHOLD <= flight_data.alt_data.rel_altitude <= apogee_val) {
                if(main_eject_flag == 0) {
                    current_state = ARMED_FLIGHT_STATE::MAIN_DEPLOY;
                    //debugln("MAIN");
                    delay(STATE_CHANGE_DELAY);
                    main_eject_flag = 1;
                } else if (main_eject_flag == 1) { // todo: confirm check_done_flag
                    current_state = ARMED_FLIGHT_STATE::MAIN_DESCENT;
                    //debugln("MAIN_DESC");
                    delay(STATE_CHANGE_DELAY);
                }
            }

            if(flight_data.alt_data.rel_altitude < LAUNCH_DETECTION_THRESHOLD) {
                current_state = ARMED_FLIGHT_STATE::POST_FLIGHT_GROUND;
                //debugln("POST_FLIGHT");
            }
        }

        flight_data.state = current_state;

    }
}

/*!****************************************************************************
 * @brief performs flight actions based on the current flight state

 * If the flight state requires an action, we perform it here
 * For example if the flight state is apogee, we perform MAIN_CHUTE ejection
 * 
 *******************************************************************************/
void flightStateCallback(void* pvParameters) {

    while(1) {
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
        
        /**
         * record number
         * operation_mode
         * state
         * ax
         * ay
         * az
         * pitch
         * roll
         * gx
         * gy
         * gz
         * latitude
         * longitude
         * gps_altitude
         * pressure
         * temperature
         * altitude_agl
         * velocity
         *
         */
        sprintf(telemetry_packet_buffer,
                "%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f\n",

                telemetry_received_packet.record_number,
                telemetry_received_packet.operation_mode,
                telemetry_received_packet.state,
                telemetry_received_packet.acc_data.ax,
                telemetry_received_packet.acc_data.ay,
                telemetry_received_packet.acc_data.az,
                telemetry_received_packet.acc_data.pitch,
                telemetry_received_packet.acc_data.roll,
                telemetry_received_packet.gyro_data.gx,
                telemetry_received_packet.gyro_data.gy,
                telemetry_received_packet.gyro_data.gz,
                gps_packet.latitude,
                gps_packet.longitude,
                gps_packet.gps_altitude,
                altimeter_packet.pressure,
                altimeter_packet.temperature,
                altimeter_packet.rel_altitude
              );
        
        debugln(telemetry_packet_buffer);
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
void logToMemory(void* pvParameter) {
    telemetry_type_t received_packet;

    while(1) {
        xQueueReceive(log_to_mem_queue_handle, &received_packet, portMAX_DELAY);

        // received_packet.record_number++; 

        // is it time to record?
        current_log_time = millis();

        if(current_log_time - previous_log_time > log_sample_interval) {
            previous_log_time = current_log_time;
            data_logger.loggerWrite(received_packet);
        }
        
    }

}

/*!****************************************************************************
 * @brief send flight data to ground
 * @param pvParameter - A value that is passed as the parameter to the created task.
 * If pvParameter is set to the address of a variable then the variable must still exist when the created task executes -
 * so it is not valid to pass the address of a stack variable.
 *
 *******************************************************************************/
void MQTT_TransmitTelemetry(void* pvParameters) {
    // variable to store the received packet to transmit
    telemetry_type_t telemetry_received_packet;

    while(1) {

        // receive from telemetry queue
        xQueueReceive(telemetry_data_queue_handle, &telemetry_received_packet, portMAX_DELAY);

        /**
         * PACKAGE TELEMETRY PACKET
         */

        /**
         * record number
         * operation_mode
         * state
         * ax
         * ay
         * az
         * pitch
         * roll
         * gx
         * gy
         * gz
         * latitude
         * longitude
         * gps_altitude
         * pressure
         * temperature
         * relative_altitude
         */
        sprintf(telemetry_packet_buffer,
                "%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f\n",

                telemetry_received_packet.record_number,
                telemetry_received_packet.operation_mode,
                telemetry_received_packet.state,
                telemetry_received_packet.acc_data.ax,
                telemetry_received_packet.acc_data.ay,
                telemetry_received_packet.acc_data.az,
                telemetry_received_packet.acc_data.pitch,
                telemetry_received_packet.acc_data.roll,
                telemetry_received_packet.gyro_data.gx,
                telemetry_received_packet.gyro_data.gy,
                telemetry_received_packet.gyro_data.gz,
                gps_packet.latitude,
                gps_packet.longitude,
                gps_packet.gps_altitude,
                altimeter_packet.pressure,
                altimeter_packet.temperature,
                altimeter_packet.rel_altitude
        );

        /* Send to MQTT topic  */
         if(client.publish(MQTT_TELEMETRY_TOPIC, telemetry_packet_buffer) ) {
             debugln("[+]Data sent");
         } else {
             debugln("[-]Data not sent");
         }

        client.publish(MQTT_TELEMETRY_TOPIC, telemetry_packet_buffer);
    }

    vTaskDelay(CONSUME_TASK_DELAY/ portTICK_PERIOD_MS);
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
void drogueChuteDeploy() {

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

}

/*!****************************************************************************
 * @brief fires the pyro-charge to deploy the main chute
 * Turn on the main chute ejection circuit by running the GPIO
 * HIGH for a preset No. of seconds
 * Default no. of seconds to remain HIGH is 5
 * 
 *******************************************************************************/
void mainChuteDeploy() {
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
}

void xCreateAllTasks() {
        debugln("Creating all tasks");
        //vTaskDelay(200/portTICK_PERIOD_MS);

        /* READ ACCELERATION DATA */
        BaseType_t gr = xTaskCreatePinnedToCore(readAccelerationTask, "readAccelerometer", STACK_SIZE*2, NULL, 2, &readAccelerationTaskHandle, 1);
        if(gr == pdPASS) {
            debugln("[+]Read acceleration task created OK.");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]Read acceleration task created OK.\r\n");
        } else {
            debugln("[-]Read acceleration task creation failed");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]Read acceleration task creation failed\r\n");
        }

        /* TASK 3: READ GPS DATA */
        BaseType_t rg = xTaskCreatePinnedToCore(readGPSTask, "readGPS", STACK_SIZE*2, NULL, 2, &readGPSTaskHandle, 1);
        if(rg == pdPASS) {
            debugln("[+]Read GPS task created OK.");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]Read GPS task created OK.\r\n");
        } else {
            debugln("[-]Failed to create GPS task");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]Failed to create GPS task\r\n");
        }

        /* CHECK FLIGHT STATE TASK */
         BaseType_t cf = xTaskCreatePinnedToCore(checkFlightState,"checkFlightState",STACK_SIZE*2,NULL, 2, &checkFlightStateTaskHandle, 1);

         if(cf == pdPASS) {
             debugln("[+]checkFlightState task created OK.");
             SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]checkFlightState task created OK.\r\n");
         } else {
             debugln("[-]Failed to create checkFlightState task");
             SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]Failed to create checkFlightState task\r\n");
         }

        /* FLIGHT STATE CALLBACK TASK */
        BaseType_t fs = xTaskCreatePinnedToCore(flightStateCallback, "flightStateCallback", STACK_SIZE*2, NULL, 2, &flightStateCallbackTaskHandle, 1);
        if(fs == pdPASS) {
            debugln("[+]flightStateCallback task created OK.");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]flightStateCallback task created OK.\r\n");
        } else {
            debugln("[-]Failed to create flightStateCallback task");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]Failed to create flightStateCallback task\r\n");
        }

        #if MQTT
            /* TRANSMIT TELEMETRY DATA */
            BaseType_t th = xTaskCreatePinnedToCore(MQTT_TransmitTelemetry, "transmit_telemetry", STACK_SIZE*2, NULL, 2, &MQTT_TransmitTelemetryTaskHandle, 1);

            if(th == pdPASS){
                debugln("[+]MQTT transmit task created OK");
                SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]MQTT transmit task created OK\r\n");
                
            } else {
                debugln("[-]MQTT transmit task failed to create");
                SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]MQTT transmit task failed to create\r\n");
            }

        #endif

        BaseType_t kf = xTaskCreatePinnedToCore(kalmanFilterTask, "kalman filter", STACK_SIZE*2, NULL, 2, &kalmanFilterTaskHandle, 1);

        if(kf == pdPASS) {
            debugln("[+]kalmanFilter task created OK.");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]kalman_filter_queue_handle creation OK.\r\n");
        } else {
            debugln("[-]kalmanFilter task failed to create");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]kalmanFilter task failed to create\r\n");
        }

        #if DEBUG_TO_TERMINAL   // set DEBUG_TO_TERMINAL to 0 to prevent serial debug data to serial monitor

            /* TASK 7: DISPLAY DATA ON SERIAL MONITOR - FOR DEBUGGING */
            BaseType_t dt = xTaskCreatePinnedToCore(debugToTerminalTask,"debugToTerminalTask",STACK_SIZE*4, NULL,2,&debugToTerminalTaskHandle, 1);
        
            if(dt == pdPASS) {
                debugln("[+]debugToTerminal task created OK");
                SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]debugToTerminal task created OK\r\n");
            } else {
                debugln("[-]debugToTerminal task not created");
                SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]debugToTerminal task not created\r\n");
            }
        
        #endif // DEBUG_TO_TERMINAL_TASK

        #if LOG_TO_MEMORY   // set LOG_TO_MEMORY to 1 to allow logging to memory 
            /* TASK 9: LOG DATA TO MEMORY */
            if(xTaskCreatePinnedToCore(logToMemory,"logToMemory",STACK_SIZE*4,NULL,2,&logToMemoryTaskHandle,1) != pdPASS){
                debugln("[-]logToMemory task failed to create");
                SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]logToMemory task failed to create\r\n");

            }else{
                debugln("[+]logToMemory task created OK.");
                SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]logToMemory task created OK.\r\n");
            }
        #endif // LOG_TO_MEMORY

        if(xTaskCreatePinnedToCore(xOperationModeIndicateTask,"xOperationModeIndicateTask",STACK_SIZE*2,NULL,2,&opModeIndicateTaskHandle,1) != pdPASS){

            debugln("[-]xOperationModeIndicateTask task failed to create");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]xOperationModeIndicateTask task failed to create\r\n");
        }else{
            debugln("[+]xOperationModeIndicateTask task created OK.");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]xOperationModeIndicateTask task created OK.\r\n");
        }

        /* READ ALTIMETER DATA */
        BaseType_t ra = xTaskCreatePinnedToCore(readAltimeterTask,"readAltimeter",STACK_SIZE*2,NULL,2, &readAltimeterTaskHandle, 1);
        if(ra == pdPASS) {
            debugln("[+]readAltimeterTask created OK.");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]readAltimeterTask created OK.\r\n");
        } else {
            debugln("[-]Failed to create readAltimeterTask");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]Failed to create readAltimeterTask\r\n");
        }

        /* RECONNECT MQTT */
        // BaseType_t rp = xTaskCreatePinnedToCore(MQTT_Reconnect,"reconnectMQTT",STACK_SIZE*2,NULL,2, NULL, 1);
        // if(rp == pdPASS) {
        //     debugln("[+]reconnectMQTT created OK.");
        //     SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]reconnectMQTT created OK.\r\n");
        // } else {
        //     debugln("[-]Failed to create reconnectMQTT");
        //     SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]Failed to create reconnectMQTT\r\n");
        // }


        debugln();
        debugln(F("=============================================="));
        debugln(F("========== FINISHED CREATING TASKS ==========="));
        debugln(F("==============================================\n"));

        // resume all tasks after creation

        // delete this task
        vTaskDelete(NULL);
    
    
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

    // create and wait for dynamic WIFI connection
    initDynamicWIFI(); // TODO - uncomment on live testing and production
    MQTTInit(MQTT_SERVER, MQTT_PORT);
    MQTT_Reconnect();  
    debugln("[+]Dynamic WIFI created OK.");

    debugln();
    debugln(F("=============================================="));
    debugln(F("========= INITIALIZING PERIPHERALS ==========="));
    debugln(F("=============================================="));
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "==Initializing peripherals==\r\n");

    uint8_t bmp_init_state = BMPInit();
    uint8_t imu_init_state = imu.init();
    uint8_t gps_init_state = GPSInit();
    // uint8_t sd_init_state = initSD();
    uint8_t flash_init_state = data_logger.loggerInit();
    debug("Flash memory init state:"); debugln(flash_init_state);

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
    baseline = altimeter_get_pressure();

    /* initialize the ring buffer - used for apogee detection */
    ring_buffer_init(&altitude_ring_buffer);

    /* check whether we are in TEST or RUN mode */
    checkRunTestToggle();

    // TODO: if toggle pin in RUN mode, set to wait for arming 

    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "RUN MODE\r\n");

    /* mode 0 resets the system log file by clearing all the current contents */
    // system_logger.logToFile(SPIFFS, 0, rocket_ID, level, system_log_file, "Game Time!"); // TODO: DEBUG

    debugln();
    debugln(F("=============================================="));
    debugln(F("===== INITIALIZING DATA LOGGING SYSTEM ======="));
    debugln(F("=============================================="));
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "==INITIALIZING DATA LOGGING SYSTEM==\r\n");
        
    debugln();
    debugln(F("=============================================="));
    debugln(F("============== CREATING QUEUES ==============="));
    debugln(F("=============================================="));
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "==CREATING QUEUES==\r\n");

    /* Every producer task sends queue to a different queue to avoid data popping issue */
    telemetry_data_queue_handle = xQueueCreate(TELEMETRY_DATA_QUEUE_LENGTH, sizeof(telemetry_type_t));
    log_to_mem_queue_handle = xQueueCreate(TELEMETRY_DATA_QUEUE_LENGTH, sizeof(telemetry_type_t));
    check_state_queue_handle = xQueueCreate(TELEMETRY_DATA_QUEUE_LENGTH, sizeof(telemetry_type_t));
    debug_to_term_queue_handle = xQueueCreate(TELEMETRY_DATA_QUEUE_LENGTH, sizeof(telemetry_type_t));
    kalman_filter_queue_handle = xQueueCreate(TELEMETRY_DATA_QUEUE_LENGTH, sizeof(telemetry_type_t));

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
    blocking_buzz(BUZZ_INTERVALS::SETUP_INIT);
    
} /* End of setup */


/*!****************************************************************************
 * @brief Main loop
 *******************************************************************************/
void loop() {
    /* enable MQTT transmit loop */
   if (!client.connected()) {
       MQTT_Reconnect();
   }
    client.loop();

} /* End of main loop*/
