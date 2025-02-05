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
void buzz(uint16_t interval);

/* state machine variables*/
uint8_t operation_mode = 0;                                     /*!< Tells whether software is in safe or flight mode - FLIGHT_MODE=1, SAFE_MODE=0 */
uint8_t current_state = ARMED_FLIGHT_STATE::PRE_FLIGHT_GROUND;	    /*!< The starting state - we start at PRE_FLIGHT_GROUND state */
uint8_t STATE_BIT_MASK = 0;

/* GPS object */
TinyGPSPlus gps;

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

/* set initial mode as safe mode */

uint8_t is_flight_mode = 0; /* flag to indicate if we are in test or flight mode */

/* Intervals for buzzer state indication - see docs */
enum BUZZ_INTERVALS {
  SETUP_INIT = 200,
  ARMING_PROCEDURE = 500
};
unsigned long current_non_block_time = 0;
unsigned long last_non_block_time = 0;
bool buzz_state = 0;

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
uint8_t flash_cs_pin = 5;           /*!< External flash memory chip select pin */
uint8_t remote_switch = 27;

/* Flight data logging */
uint8_t flash_led_pin = 32;                  /*!< LED pin connected to indicate flash memory formatting  */
char filename[] = "flight.txt";             /*!< data log filename - Filename must be less than 20 chars, including the file extension */
uint32_t FILE_SIZE_512K = 524288L;          /*!< 512KB */
uint32_t FILE_SIZE_1M  = 1048576L;          /*!< 1MB */
uint32_t FILE_SIZE_4M  = 4194304L;          /*!< 4MB */
SerialFlashFile file;                       /*!< object representing file object for flash memory */
unsigned long long previous_log_time = 0;   /*!< The last time we logged data to memory */
unsigned long long current_log_time = 0;    /*!< What is the processor time right now? */
uint16_t log_sample_interval = 10;          /*!< After how long should we sample and log data to flash memory? */

DataLogger data_logger(flash_cs_pin, flash_led_pin, filename, file,  FILE_SIZE_4M);

/* position integration variables */
long long current_time = 0;
long long previous_time = 0;

/* To store the main telemetry packet being sent over MQTT */
char telemetry_packet_buffer[256];
ring_buffer altitude_ring_buffer;
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
    debugln("Check");
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
MPU6050 imu(MPU_ADDRESS, MPU_ACCEL_RANGE, GYRO_RANGE); // TODO: remove magic numbers 

/* create BMP object */
SFE_BMP180 altimeter;
char status;
double T, PRESSURE, p0, a;

/**
* @brief initialize Buzzer
*/
void buzzerInit() {
    pinMode(BUZZER_PIN, OUTPUT);
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
        // TODO: update system table
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
    Serial2.begin(GPS_BAUD_RATE);
    delay(100); // wait for GPS to init

    debugln("[+]GPS init OK!"); 

    /**
     * FIXME: Proper GPS init check!
     * Look into if the GPS has acquired a LOCK on satelites 
     * Only if it has a lock then can we return a 1
     * 
     * GPS is low priority at the time of writing this but make it work! 
     * */ 

    return 1;
}

/**
* @brief - non-blocking buzz 
 */
void buzz(uint16_t interval) {
        /* non-blocking buzz */
    current_non_block_time = millis();
    if((current_non_block_time - last_non_block_time) > interval) {
        buzz_state = !buzz_state;
        last_non_block_time = current_non_block_time;
        digitalWrite(BUZZER_PIN, buzz_state);
    }

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

        acc_data_lcl.acc_data.ax = imu.readXAcceleration();
        acc_data_lcl.acc_data.ay = imu.readYAcceleration();
        acc_data_lcl.acc_data.az = 0;

        // get pitch and roll
        acc_data_lcl.acc_data.pitch = imu.getPitch();
        acc_data_lcl.acc_data.roll = imu.getRoll();
        

        xQueueSend(telemetry_data_queue_handle, &acc_data_lcl, 0);
        xQueueSend(log_to_mem_queue_handle, &acc_data_lcl, 0);
        xQueueSend(check_state_queue_handle, &acc_data_lcl, 0);
        xQueueSend(debug_to_term_queue_handle, &acc_data_lcl, 0);
    }

}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// ALTITUDE AND VELOCITY DETERMINATION /////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

/*!****************************************************************************
 * @brief Read atm pressure data from the barometric sensor onboard
 *******************************************************************************/
void readAltimeterTask(void* pvParameters) {
    telemetry_type_t alt_data_lcl;

    while(1) {
        // If you want to measure altitude, and not pressure, you will instead need
        // to provide a known baseline pressure. This is shown at the end of the sketch.

        // You must first get a temperature measurement to perform a pressure reading.
        
        // Start a temperature measurement:
        // If request is successful, the number of ms to wait is returned.
        // If request is unsuccessful, 0 is returned.
        status = altimeter.startTemperature();
        if(status !=0 ) {
            // wait for measurement to complete
            delay(status);

            // retrieve the completed temperature measurement 
            // temperature is stored in variable T

            status = altimeter.getTemperature(T);
            if(status != 0) {
                // print out the measurement 
                // debug("temperature: ");
                // debug(T, 2);
                // debug(" \xB0 C, ");

                // start pressure measurement 
                // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
                // If request is successful, the number of ms to wait is returned.
                // If request is unsuccessful, 0 is returned.
                status = altimeter.startPressure(3);
                if(status != 0) {
                    // wait for the measurement to complete
                    delay(status);

                    // Retrieve the completed pressure measurement:
                    // Note that the measurement is stored in the variable P.
                    // Note also that the function requires the previous temperature measurement (T).
                    // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
                    // Function returns 1 if successful, 0 if failure.

                    status = altimeter.getPressure(PRESSURE, T);
                    if(status != 0) {
                        // print out the measurement
                        // debug("absolute pressure: ");
                        // debug(P, 2);
                        // debug(" mb, "); // in millibars

                        p0 = altimeter.sealevel(PRESSURE,ALTITUDE);
                        // If you want to determine your altitude from the pressure reading,
                        // use the altitude function along with a baseline pressure (sea-level or other).
                        // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
                        // Result: a = altitude in m.

                        a = altimeter.altitude(PRESSURE, p0);
                        //debug(a);

                        // feed the altitude into the kalman filter
                        estimated_altitude = kalmanFilter(a);
                        //debug(",");
                        //debugln(estimated_altitude);

                    } else {
                        debugln("error retrieving pressure measurement\n");
                    } 
                
                } else {
                    debugln("error starting pressure measurement\n");
                }

            } else {
                debugln("error retrieving temperature measurement\n");
            }

        } else {
            debugln("error starting temperature measurement\n");
        }

        // delay(2000);

        // TODO: compute the velocity from the altimeter data

        // assign data to queue
        alt_data_lcl.alt_data.pressure = PRESSURE;
        alt_data_lcl.alt_data.altitude = a;
        alt_data_lcl.alt_data.velocity = 0;
        alt_data_lcl.alt_data.temperature = T;

        alt_data_lcl.alt_data.pressure = 0;
        alt_data_lcl.alt_data.altitude = 0;
        alt_data_lcl.alt_data.velocity = 0;
        alt_data_lcl.alt_data.temperature = 0;

        // send this pressure data to queue
        // do not wait for the queue if it is full because the data rate is so high, 
        // we might lose some data as we wait for the queue to get space

        xQueueSend(telemetry_data_queue_handle, &alt_data_lcl, 0);
        xQueueSend(log_to_mem_queue_handle, &alt_data_lcl, 0);
        xQueueSend(check_state_queue_handle, &alt_data_lcl, 0);
        xQueueSend(debug_to_term_queue_handle, &alt_data_lcl, 0);

        vTaskDelay(CONSUME_TASK_DELAY / portTICK_PERIOD_MS);
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

    telemetry_type_t gps_data_lcl;

    while(true){
        // if(Serial2.available()) {
        //     char c = Serial2.read();

        //     if(gps.encode(c)) {
        //         // GPS lock hs been acquired 
        //         // set the new data lock to 1
        //         debugln(c); // dump gps data
        //     } else {
        //        // set new data lock to 0
        //     } 
        // }

        if (Serial2.available()) {
            char c = Serial2.read();
            if(gps.encode(c)){
                // get location, latitude and longitude 
                if(gps.location.isValid()) {
                    gps_data_lcl.gps_data.latitude = gps.location.lat();
                    gps_data_lcl.gps_data.longitude = gps.location.lng();
                } else {
                    // debugln("Invalid GPS location");
                    gps_data_lcl.gps_data.latitude = 0;
                    gps_data_lcl.gps_data.longitude = 0;
                }

                // if(gps.time.isValid()) {
                //     gps_data_lcl.gps_data.time = gps.time.value(); // decode this time value post flight - write a script for that
                // } else {
                //     debugln("Invalid GPS time");
                // }

                if(gps.altitude.isValid()) {
                    gps_data_lcl.gps_data.gps_altitude = gps.altitude.meters();
                } else {
                    // debugln("Invalid altitude data"); // TODO: LOG to system logger
                    gps_data_lcl.gps_data.gps_altitude = 0;
                }
            }
        }

        xQueueSend(telemetry_data_queue_handle, &gps_data_lcl, portMAX_DELAY);
        xQueueSend(log_to_mem_queue_handle, &gps_data_lcl, portMAX_DELAY);
        xQueueSend(check_state_queue_handle, &gps_data_lcl, portMAX_DELAY);
        xQueueSend(debug_to_term_queue_handle, &gps_data_lcl, portMAX_DELAY);

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
            //debug("altitude value:"); debugln(flight_data.alt_data.altitude);
            if(flight_data.alt_data.altitude < LAUNCH_DETECTION_THRESHOLD) {
                current_state = ARMED_FLIGHT_STATE::PRE_FLIGHT_GROUND;
                debugln("PREFLIGHT");
                delay(STATE_CHANGE_DELAY);
            } else if(LAUNCH_DETECTION_THRESHOLD < flight_data.alt_data.altitude < (LAUNCH_DETECTION_THRESHOLD+LAUNCH_DETECTION_ALTITUDE_WINDOW) ) {
                current_state = ARMED_FLIGHT_STATE::POWERED_FLIGHT;
                debugln("POWERED");
                delay(STATE_CHANGE_DELAY);
            } 

            // COASTING

            // APOGEE and APOGEE DETECTION
            ring_buffer_put(&altitude_ring_buffer, flight_data.alt_data.altitude);
            if(ring_buffer_full(&altitude_ring_buffer) == 1) {
                oldest_val = ring_buffer_get(&altitude_ring_buffer);
            }

            //debug("Curr val:");debug(flight_data.alt_data.altitude); debug("    "); debugln(oldest_val);
            if((oldest_val - flight_data.alt_data.altitude) >= APOGEE_DETECTION_THRESHOLD) {

                if(apogee_flag == 0) {
                    apogee_val = ( (oldest_val - flight_data.alt_data.altitude) / 2 ) + oldest_val;

                    current_state = ARMED_FLIGHT_STATE::APOGEE;
                    delay(STATE_CHANGE_DELAY);
                    debugln("APOGEE");
                    delay(STATE_CHANGE_DELAY);
                    current_state = ARMED_FLIGHT_STATE::DROGUE_DEPLOY;
                    debugln("DROGUE");
                    delay(STATE_CHANGE_DELAY);
                    current_state =  ARMED_FLIGHT_STATE::DROGUE_DESCENT;
                    debugln("DROGUE_DESCENT");
                    delay(STATE_CHANGE_DELAY);
                    apogee_flag = 1;
                }

            }

        } else if(apogee_flag == 1) {

            if(LAUNCH_DETECTION_THRESHOLD <= flight_data.alt_data.altitude <= apogee_val) {
                if(main_eject_flag == 0) {
                    current_state = ARMED_FLIGHT_STATE::MAIN_DEPLOY;
                    debugln("MAIN");
                    delay(STATE_CHANGE_DELAY);
                    main_eject_flag = 1;
                } else if (main_eject_flag == 1) { // todo: confirm check_done_flag
                    current_state = ARMED_FLIGHT_STATE::MAIN_DESCENT;
                    debugln("MAIN_DESC");
                    delay(STATE_CHANGE_DELAY);
                }
            }

            if(flight_data.alt_data.altitude < LAUNCH_DETECTION_THRESHOLD) {
                current_state = ARMED_FLIGHT_STATE::POST_FLIGHT_GROUND;
                debugln("POST_FLIGHT");
            }

        }
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
                //debugln("DROGUE DEPLOY");
                drogueChuteDeploy();
                break;

            // DROGUE_DESCENT
            case ARMED_FLIGHT_STATE::DROGUE_DESCENT:
            //    debugln("DROGUE DESCENT");
                break;

            // MAIN_DEPLOY
            case ARMED_FLIGHT_STATE::MAIN_DEPLOY:
            //    debugln("MAIN CHUTE DEPLOY");
                mainChuteDeploy();
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
        xQueueReceive(log_to_mem_queue_handle, &telemetry_received_packet, portMAX_DELAY);
        
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
                "%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",

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
                telemetry_received_packet.gps_data.latitude,
                telemetry_received_packet.gps_data.longitude,
                telemetry_received_packet.gps_data.gps_altitude,
                telemetry_received_packet.alt_data.pressure,
                telemetry_received_packet.alt_data.temperature,
                telemetry_received_packet.alt_data.AGL,
                telemetry_received_packet.alt_data.velocity
                );
        
        debugln(telemetry_packet_buffer);
        vTaskDelay(CONSUME_TASK_DELAY/portTICK_PERIOD_MS);
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
 * @param pvParameter - A value that is passed as the paramater to the created task.
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
         * gps_time
         * pressure
         * temperature
         * altitude_agl
         * velocity
         * pyro1_state // not used
         * pyro2_state // not used
         * battery_voltage // not used
         *
         */
        sprintf(telemetry_packet_buffer,
            "%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",

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
            telemetry_received_packet.gps_data.latitude,
            telemetry_received_packet.gps_data.longitude,
            telemetry_received_packet.gps_data.gps_altitude,
            telemetry_received_packet.alt_data.pressure,
            telemetry_received_packet.alt_data.temperature,
            telemetry_received_packet.alt_data.AGL,
            telemetry_received_packet.alt_data.velocity
            );

        /* Send to MQTT topic  */
        // if(client.publish(MQTT_TOPIC, telemetry_packet_buffer) ) {
        //     debugln("[+]Data sent");
        // } else {
        //     debugln("[-]Data not sent");
        // }

        client.publish(MQTT_TOPIC, telemetry_packet_buffer);
    }

    vTaskDelay(CONSUME_TASK_DELAY/ portTICK_PERIOD_MS);
}

/*!
 * @brief Try reconnecting to MQTT if connection is lost
 *
 */
void MQTT_Reconnect() {
     if(!client.connected()){
         debug("[..]Attempting MQTT connection..."); // TODO: SYS LOGGER
         String client_id = "[+]Flight-computer-1 client: ";
         client_id += String(random(0XFFFF), HEX);

         if(client.connect(client_id.c_str())){
             debugln("[+]MQTT reconnected");
         }
    }
}


/*!****************************************************************************
 * @brief Initialize MQTT
 *
 *******************************************************************************/
void MQTTInit(const char* broker_IP, int broker_port) {
    // client.setBufferSize(MQTT_BUFFER_SIZE);
    debugln("[+]Initializing MQTT\n");
    client.setServer(broker_IP, broker_port);
    delay(2000);
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

/*!****************************************************************************
 * @brief Setup - perform initialization of all hardware subsystems, create queues, create queue handles
 * initialize system check table
 * 
 *******************************************************************************/
void setup() {
    buzzerInit();

    /* buzz to indicate start of setup */
    buzz(BUZZ_INTERVALS::SETUP_INIT);

    /* core to run the tasks */
    uint8_t app_id = xPortGetCoreID();

    /* initialize serial */
    Serial.begin(BAUDRATE);
    delay(100);

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
    //initDynamicWIFI(); // TODO - uncomment on live testing and production

    debugln();
    debugln(F("=============================================="));
    debugln(F("========= INITIALIZING PERIPHERALS ==========="));
    debugln(F("=============================================="));
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "==Initializing peripherals==\r\n");

    uint8_t bmp_init_state = BMPInit();
    uint8_t imu_init_state = imu.init();
    uint8_t gps_init_state = GPSInit();
    uint8_t sd_init_state = initSD();
    uint8_t flash_init_state = data_logger.loggerInit();
    

    /* initialize mqtt */
    MQTTInit(MQTT_SERVER, MQTT_PORT);

    /* update the sub-systems init state table */
    // check if BMP init OK
    if(bmp_init_state) { 
        SUBSYSTEM_INIT_MASK |= (1 << BMP_CHECK_BIT);
    }

    // check if MPU init OK
    if(imu_init_state)  {
        SUBSYSTEM_INIT_MASK |= (1 << IMU_CHECK_BIT);
    }

    // check if flash memory init OK
    if (flash_init_state) {
        SUBSYSTEM_INIT_MASK |= (1 << FLASH_CHECK_BIT);
    }

    // check if GPS init OK
    if(gps_init_state) {
        SUBSYSTEM_INIT_MASK |= (1 << GPS_CHECK_BIT);
    }

    // check if SD CARD init OK
    if(sd_init_state) {
        SUBSYSTEM_INIT_MASK |= (1 << SD_CHECK_BIT);
    } 

    // check if SPIFFS init OK
    if(spiffs_init_state) {
        SUBSYSTEM_INIT_MASK |= (1 << SPIFFS_CHECK_BIT);
    }


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
    * /

    /* TASK 1: READ ACCELERATION DATA */
    BaseType_t gr = xTaskCreate(readAccelerationTask, "readGyroscope", STACK_SIZE*2, NULL, 2, &readAccelerationTaskHandle);
    if(gr == pdPASS) {
        debugln("[+]Read acceleration task created OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]Read acceleration task created OK.\r\n");
    } else {
        debugln("[-]Read acceleration task creation failed");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]Read acceleration task creation failed\r\n");
    }

    /* TASK 2: READ ALTIMETER DATA */
    BaseType_t ra = xTaskCreate(readAltimeterTask,"readAltimeter",STACK_SIZE*3,NULL,2, &readAltimeterTaskHandle);
    if(ra == pdPASS) {
        debugln("[+]readAltimeterTask created OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]readAltimeterTask created OK.\r\n");
    } else {
        debugln("[-]Failed to create readAltimeterTask");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]Failed to create readAltimeterTask\r\n");
    }

    /* TASK 3: READ GPS DATA */
    BaseType_t rg = xTaskCreate(readGPSTask, "readGPS", STACK_SIZE*2, NULL,2, &readGPSTaskHandle);
    if(rg == pdPASS) {
        debugln("[+]Read GPS task created OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]Read GPS task created OK.\r\n");
    } else {
        debugln("[-]Failed to create GPS task");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]Failed to create GPS task\r\n");
    }

    /* TASK 5: CHECK FLIGHT STATE TASK */
    BaseType_t cf = xTaskCreate(checkFlightState,"checkFlightState",STACK_SIZE*2,NULL,2, &checkFlightStateTaskHandle);

    if(cf == pdPASS) {
        debugln("[+]checkFlightState task created OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]checkFlightState task created OK.\r\n");
    } else {
        debugln("[-]Failed to create checkFlightState task");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]Failed to create checkFlightState task\r\n");
    }

    /* TASK 6: FLIGHT STATE CALLBACK TASK */
    BaseType_t fs = xTaskCreate(flightStateCallback, "flightStateCallback", STACK_SIZE*2, NULL, 2, &flightStateCallbackTaskHandle);

    if(fs == pdPASS) {
        debugln("[+]flightStateCallback task created OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]flightStateCallback task created OK.\r\n");
    } else {
        debugln("[-]Failed to create flightStateCallback task");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]Failed to create flightStateCallback task\r\n");
    }

    /* TASK 8: TRANSMIT TELEMETRY DATA */
    BaseType_t th = xTaskCreate(MQTT_TransmitTelemetry, "transmit_telemetry", STACK_SIZE*4, NULL, 2, &MQTT_TransmitTelemetryTaskHandle);

    if(th == pdPASS){
        debugln("[+]MQTT transmit task created OK");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]kalman_filter_queue_handle creation OK.\r\n");
        
    } else {
        debugln("[-]MQTT transmit task failed to create");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]MQTT transmit task failed to create\r\n");
    }

    BaseType_t kf = xTaskCreate(kalmanFilterTask, "kalman filter", STACK_SIZE*2, NULL, 2, &kalmanFilterTaskHandle);

    if(kf == pdPASS) {
        debugln("[+]kalmanFilter task created OK.");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]kalman_filter_queue_handle creation OK.\r\n");
    } else {
        debugln("[-]kalmanFilter task failed to create");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]kalmanFilter task failed to create\r\n");
    }

    #if DEBUG_TO_TERMINAL   // set DEBUG_TO_TERMINAL to 0 to prevent serial debug data to serial monitor

        /* TASK 7: DISPLAY DATA ON SERIAL MONITOR - FOR DEBUGGING */
        BaseType_t dt = xTaskCreate(debugToTerminalTask,"debugToTerminalTask",STACK_SIZE*4, NULL,2,&debugToTerminalTaskHandle);
    
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
        if(xTaskCreate(
                logToMemory,
                "logToMemory",
                STACK_SIZE,
                NULL,
                2,
                &logToMemoryTaskHandle
        ) != pdPASS){
            debugln("[-]logToMemory task failed to create");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[-]logToMemory task failed to create\r\n");
            vTaskSuspend(logToMemoryTaskHandle);

        }else{
            debugln("[+]logToMemory task created OK.");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "[+]logToMemory task created OK.\r\n");

        }
    #endif // LOG_TO_MEMORY

    debugln();
    debugln(F("=============================================="));
    debugln(F("========== FINISHED CREATING TASKS ==========="));
    debugln(F("==============================================\n"));
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "==FINISHED CREATING TASKS==\r\n");

    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "\nEND OF INITIALIZATION\r\n");


    /* buzz to indicate start of setup */
    buzz(BUZZ_INTERVALS::SETUP_INIT);
    
} /* End of setup */


/*!****************************************************************************
 * @brief Main loop
 *******************************************************************************/
void loop() {
    /* enable MQTT transmit loop */
    MQTT_Reconnect();
    client.loop();
} /* Enf of main loop*/
