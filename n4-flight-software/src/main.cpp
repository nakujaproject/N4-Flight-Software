/**
 * @file main.cpp
 * @author Edwin Mwiti
 * @version 0.1
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
#include "custom_time.h"    // custom time conversions
#include "states.h"         // state machine states
#include "system_logger.h"  // system logging functions
#include "system_log_levels.h"  // system logging log levels
#include "wifi-config.h"    // handle wifi connection

/* function prototypes definition */
void drogueChuteDeploy();
void mainChuteDeploy();

/* state machine variables*/
uint8_t operation_mode = 0;                                     /*!< Tells whether software is in safe or flight mode - FLIGHT_MODE=1, SAFE_MODE=0 */
uint8_t current_state = FLIGHT_STATE::PRE_FLIGHT_GROUND;	    /*!< The starting state - we start at PRE_FLIGHT_GROUND state */

/* GPS object */
TinyGPSPlus gps;

/* system logger */
SystemLogger system_logger;
const char* system_log_file = "/sys_log.log";
LOG_LEVEL level = INFO;
const char* rocket_ID = "rocket-1";             /*!< Unique ID of the rocket. Change to the needed rocket name before uploading */


//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// FLIGHT COMPUTER TESTING SYSTEM  /////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
char subsystems_state_buffer[10]; /*!< Holds the status of the subsystems. 1 if Init OK, 0 if init failed */

#define BMP_CHECK_BIT           0
#define IMU_CHECK_BIT           1   
#define FLASH_CHECK_BIT         2
#define GPS_CHECK_BIT           3
#define SD_CHECK_BIT            4
#define SPIFFS_CHECK_BIT        5
#define TEST_HARDWARE_CHECK_BIT 6

uint8_t SUBSYSTEM_INIT_MASK = 0b00000000;  /*!< Holds the status of the subsystems. 1 if Init OK, 0 if init failed */

uint8_t DAQ_MODE = 0;
uint8_t TEST_MODE = 0;
uint8_t current_test_state;

#define BAUDRATE        115200
#define NAK_INTERVAL    4000        /*!< Interval in which to send the NAK command to the transmitter */

//  Flags
uint8_t SOH_recvd_flag = 0; /*!< Transmitter acknowledged?  */

unsigned long last_NAK_time = 0;
unsigned long current_NAK_time = 0;
char SOH_CHR[6] = "SOH";

/* define XMODEM commands in HEX */
#define SOH     0x01    /*!< start of header */
#define EOT     0x04    /*!< end of transmission */
#define ACK     0x06    /*!< positive acknowledgement */
#define NAK     0x15    /*!< negative acknowledgement */
#define CAN     0x18    /*!< cancel */

#define MAX_CMD_LENGTH 10 /*!< Maximum length of the XMODEM command string that can be received */
#define MAX_CSV_LENGTH 256 /*!< Maximum length of the csv string that can be received */

// buffer to store the XMODEM commands
char serial_buffer[MAX_CMD_LENGTH];
int16_t serial_index = 0;

// buffer to store the CSV test data
char test_data_buffer[MAX_CSV_LENGTH];
char data_buffer_formatted[MAX_CSV_LENGTH];
int16_t test_data_serial_index = 0;

// pin definitions
uint8_t recv_data_led = 2;          /*!< External flash memory chip select pin */
uint8_t red_led = 15;               /*!< Red LED pin */
uint8_t green_led = 4;              /*!< Green LED pin */
uint8_t buzzer = 33;
uint8_t SET_DAQ_MODE_PIN = 14;     /*!< Pin to set the flight computer to DAQ mode */
uint8_t SET_TEST_MODE_PIN = 13;      /*!< Pin to set the flight computer to TEST mode */
uint8_t SD_CS_PIN = 26;             /*!< Chip select pin for SD card */


/*!*****************************************************************************
 * @brief This enum holds the states during flight computer test mode
 *******************************************************************************/
enum DAQ_STATES {
    HANDSHAKE = 0,      /*!< state to establish initial communication with transmitter */
    RECEIVE_TEST_DATA,  /*!< sets the flight computer to receive test data over serial */
    CONFIRM_TEST_DATA,
    FINISH_DATA_RECEIVE
};

/**
 * 
 * Holds the states used when consuming the test data in testing mode
 * 
 */
enum TEST_STATES {
    DATA_CONSUME = 0,
    DONE_TESTING
};

uint8_t current_DAQ_state = DAQ_STATES::HANDSHAKE; /*!< Define current state the flight computer is in */

/**
 * XMODEM serial function prototypes
 */
void listDir(fs::FS &fs, const char *dirname, uint8_t levels);

uint8_t initTestGPIO();

uint8_t InitSPIFFS();

uint8_t initSD(); // TODO: return a bool
void SwitchLEDs(uint8_t, uint8_t);

void InitXMODEM();

void SerialEvent();

void ParseSerial(char *);

void checkRunTestToggle();

//////////////////// SPIFFS FILE OPERATIONS ///////////////////////////

#define FORMAT_SPIFFS_IF_FAILED 1
const char *test_data_file = "/data.csv";
const char* run_state_file = "/state.txt";

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
    Serial.printf("Listing directory: %s\r\n", dirname);
    File root = fs.open(dirname);
    if (!root) {
        debugln("- failed to open directory");
        return;
    }
    if (!root.isDirectory()) {
        debugln(" - not a directory");
        return;
    }
    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            debug("  DIR : ");
            debugln(file.name());
            if (levels) {
                listDir(fs, file.name(), levels - 1);
            }
        } else {
            debug("  FILE: ");
            debug(file.name());
            debug("\tSIZE: ");
            debugln(file.size());
        }
        file = root.openNextFile();
    }
}

void readFile(fs::FS &fs, const char *path) {
    Serial.printf("Reading file: %s\r\n", path);
    File file = fs.open(path);
    if (!file || file.isDirectory()) {
        debugln("- failed to open file for reading");
        return;
    }
    debugln("- read from file:");
    while (file.available()) {
        Serial.write(file.read());
    }
    file.close();
}

char current_test_state_buffer[50] = ""; // TODO: use appropriate size, to hold the test state read from the SD card state.txt file 
void readStateFile(fs::FS &fs, const char *path) {
    Serial.printf("Reading file: %s\r\n", path);
    File file = fs.open(path);
    if (!file || file.isDirectory()) {
        debugln("- failed to open file for reading");
        return;
    }

    debugln("- read from file:");
    int index = 0;
    while (file.available()) {
        // Serial.write(file.read());
        current_test_state_buffer[index++] = file.read();
        // sprintf(current_test_state_buf,"%s\n", file.read());
    }

    current_test_state_buffer[index] = '\0';

    file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
    Serial.printf("Writing file: %s\r\n", path);
    File file = fs.open(path, FILE_WRITE);
    if (!file) {
        debugln("- failed to open file for writing");
        return;
    }
    if (file.print(message)) {
        debugln("- file written");
    } else {
        debugln("- write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
    Serial.printf("Appending to file: %s\r\n", path);
    File file = fs.open(path, FILE_APPEND);
    if (!file) {
        debugln("- failed to open file for appending");
        return;
    }
    if (file.print(message)) {
        debugln("OK");
    } else {
        debugln("FAILED");
    }
    file.close();
}

void deleteFile(fs::FS &fs, const char *path) {
    Serial.printf("Deleting file: %s\r\n", path);
    if (fs.remove(path)) {
        debugln("- file deleted");
    } else {
        debugln("- delete failed");
    }
}

void readTestDataFile() {
    // File logFile = SPIFFS.open(test_data_file, "r");
    // if (logFile) {
    //     debugln("Log file contents:");
    //     while (logFile.available()) {
    //         Serial.write(logFile.read());
    //     }
    //     logFile.close();
    // } else {
    //     debugln("Failed to open log file for reading.");
    // }

    // read back the received data to confirm
    readFile(SD, "/data.txt");
    
}

uint8_t InitSPIFFS() {
    if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
        debugln("SPIFFS mount failed"); // TODO: Set a flag for test GUI
        return 0;
    } else {
        debugln("SPIFFS init success");
        return 1;
    }
}

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
        File file = SD.open("/data.txt", FILE_WRITE); // TODO: change file name to const char*
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

void initDataFiles() {

    writeFile(SD, "/data.txt", "Test data\r\n");
    writeFile(SD, "/state.txt", "DATA_CONSUME\r\n"); 

}

//////////////////// END OF SPIFFS FILE OPERATIONS ///////////////////////////

/*!****************************************************************************
 * @brief Inititialize the GPIOs
 *******************************************************************************/
uint8_t initTestGPIO() {
    pinMode(red_led, OUTPUT);
    pinMode(green_led, OUTPUT);
    pinMode(SET_DAQ_MODE_PIN, INPUT);
    pinMode(SET_TEST_MODE_PIN, INPUT);
    pinMode(buzzer, OUTPUT);

    // set LEDs to a known starting state
    digitalWrite(red_led, LOW);
    digitalWrite(green_led, LOW);

    return 1; /* FIXME: Do a proper check here! */
}

/*!****************************************************************************
 * @brief Switch the LEDS states
 *******************************************************************************/
void SwitchLEDs(uint8_t red_state, uint8_t green_state) {
    digitalWrite(red_led, red_state);
    digitalWrite(green_led, green_state);
}

// non blocking timings
unsigned long last_buzz = 0;
unsigned long current_buzz = 0;
unsigned long buzz_interval = 200;
uint8_t buzzer_state = LOW;

unsigned long last_blink = 0;
unsigned long current_blink = 0;
unsigned long blink_interval = 200;
uint8_t led_state = LOW;

/*!****************************************************************************
 * @brief Buzz the buzzer for a given buzz_interval
 * This function is non-blocking
 *******************************************************************************/
void buzz() {
    current_buzz = millis();
    if ((current_buzz - last_buzz) > buzz_interval) {
        if (buzzer_state == LOW) {
            buzzer_state = HIGH;
        } else {
            buzzer_state = LOW;
        }

        digitalWrite(buzzer, buzzer_state);

        last_buzz = current_buzz;
    }

}

/*!****************************************************************************
 * @brief implements non-blocking blink
 *******************************************************************************/
void blink_200ms(uint8_t led_pin) {
    current_blink = millis();
    if ((current_blink - last_blink) > blink_interval) {
        if (led_state == LOW) {
            led_state = HIGH;
        } else if (led_state == HIGH) {
            led_state = LOW;
        }

        digitalWrite(led_pin, led_state);
        last_blink = current_blink;
    }
}

/*!****************************************************************************
 * @brief Sample the RUN/TEST toggle pins to check whether the flight computer is in test mode
 * or run mode.
 * If in TEST mode, define the TEST flag
 * If in RUN mode, define the RUN flag
 * TEST_MODE Pin and RUN_MODE pin are both pulled HIGH. When you set the jumper, you pull that pin to LOW.
 *******************************************************************************/
void checkRunTestToggle() {

    if ((digitalRead(SET_TEST_MODE_PIN) == 0) && (digitalRead(SET_DAQ_MODE_PIN) == 1)) {
        // run mode
        TEST_MODE = 1;
        DAQ_MODE = 0;
        SwitchLEDs(DAQ_MODE, TEST_MODE);
    }

    if ((digitalRead(SET_TEST_MODE_PIN) == 1) && (digitalRead(SET_DAQ_MODE_PIN) == 0)) {
        // test mode
        TEST_MODE = 0;
        DAQ_MODE = 1;

        SwitchLEDs(DAQ_MODE, TEST_MODE);
    }

    // here the jumper has been removed. we are neither in the TEST or RUN mode
    // INVALID STATE
    if ((digitalRead(SET_TEST_MODE_PIN) == 1) && (digitalRead(SET_DAQ_MODE_PIN) == 1)) {
        DAQ_MODE = 0;
        TEST_MODE = 0;
        SwitchLEDs(!DAQ_MODE, !TEST_MODE);
    }

}


/**
 * XMODEM serial function definition
 */

/*!****************************************************************************
 * @brief Initiate XMODEM protocol by sending a NAK command every 4 seconds until the transmitter returns an ACK signal
 * @param none
 *******************************************************************************/
void InitXMODEM() {

    // call the trasmitter
    Serial.begin(BAUDRATE);
    debug(NAK);
    debug("\n");
    Serial.flush();

}

/*!****************************************************************************
 * @brief Parse the received serial command if it is a string
 *******************************************************************************/
int value = 0;

void ParseSerialBuffer(char *buffer) {

    if (strcmp(buffer, SOH_CHR) == 0) {
        // if(buffer == SOH){

        debugln(F("<Start of transmission>"));
        SOH_recvd_flag = 1;
        digitalWrite(red_led, HIGH);
        debugln(F("<SOH rcvd from receiver> Waiting for data..."));

        // put the MCU in data receive state
        current_DAQ_state = DAQ_STATES::RECEIVE_TEST_DATA;
        SwitchLEDs(0, 1);


    } else {
        debugln("Unknown");
    }

}

/*!****************************************************************************
 * @brief Parse the received serial command if it is a digit
 We are only interested in numeric values being sent by the transmitter to us, the receiver
 *******************************************************************************/
void ParseSerialNumeric(int value) {
    debug("Receive val: ");
    debugln(value);

    if (value == 1) // SOH: numeric 1 -> ready to receive data
    {
        debugln("<Start of transmission>");
        SOH_recvd_flag = 1;
        debugln("<SOH rcvd> Waiting for data");

        // put the MCU in data receive state
        // any serial data after this will be the actual test data being received
        SwitchLEDs(0, 1); // red off, green on
        current_DAQ_state = DAQ_STATES::RECEIVE_TEST_DATA;

    } else if (value == 4) {
        // EOT: numeric 4
        debugln("Unknown");
    }
}

/*!****************************************************************************
 * @brief Receive serial message during handshake
 *******************************************************************************/
void handshakeSerialEvent() {
    SwitchLEDs(1, 0);
    if (Serial.available()) {
        char ch = Serial.read();

        if (isDigit(ch)) { // character between 0 an 9
            // accumulate value
            value = value * ch + (ch - '0');
        } else if (ch == '\n') { 
            debug("SerEvent: ");
            debugln(value);
            ParseSerialNumeric(value);
            value = 0; // reset value for the next transmission burst
        }


        // if(serial_index < MAX_CMD_LENGTH && (ch != '\n') ) { // use newline to signal end of command
        //     serial_buffer[serial_index++] = ch;
        // } else {
        //     // here when buffer is full or a newline is received
        //     debugln(serial_buffer);
        //     ParseSerial(serial_buffer);
        //     serial_buffer[serial_index] = 0; // terminate the string with a 0
        //     serial_index = 0;

        // }

    }
}

/*!****************************************************************************
 * @brief Receive serial message during RECEIVE_TEST_DATA state
 * Data received in this state is the actual test data. It is saved into the test flash memory
 *
 *******************************************************************************/
void receiveTestDataSerialEvent() {
    // initialize test data file
    
    // File file = SD.open("/data.txt", FILE_APPEND); // TODO: change file name to const char*

    if (Serial.available()) {
        char ch = Serial.read();
        // Serial.write(ch);

        // each CSV string ends with a newline
        if (ch != '\n') { // TODO: check for string length
            test_data_buffer[test_data_serial_index++] = ch;

        } else {
            // newline is received            
            
            // debugln(test_data_buffer);
            test_data_buffer[test_data_serial_index] = 0; // NUL terminator

            sprintf(data_buffer_formatted, "%s\n", test_data_buffer);
            appendFile(SD, "/data.txt", (const char*) data_buffer_formatted);

            test_data_serial_index = 0;   

        } 
    } else {
        // end of transmission
        debugln("EOT");

        // current_test_state = TEST_STATE::CONFIRM_TEST_DATA;
        current_DAQ_state = DAQ_STATES::FINISH_DATA_RECEIVE;
        
    }   

    // file.close(); // close the file after receiving the test data is completed
}

/**
 * @brief this function prepares the flight computer to receive test data 
 * depending on the state, it tries to establich link between flight computer and sending PC
 * receives the actual data
 * confirm the actual data
 */
void prepareForDataReceive() {

    // switch (current_test_state) {
    //     case TEST_STATE::HANDSHAKE:
    //         debugln("HANDSHAKE");
    //         break;

    //     case TEST_STATE::RECEIVE_TEST_DATA:
    //         debugln("RECEIVE_TEST_DATA");
    //         break;
    // }
    
    if(DAQ_MODE) {
        // we are in test mode 
        switch (current_DAQ_state)
        {
            // this state tries to establish communication with the sending PC
            case DAQ_STATES::HANDSHAKE:
                handshakeSerialEvent();
                if(!SOH_recvd_flag) {
                    current_NAK_time = millis();
                    if((current_NAK_time - last_NAK_time) > NAK_INTERVAL) {
                        InitXMODEM(); // send NAK every NAK_INTERVAL (4 seconds typically)
                        last_NAK_time = current_NAK_time;
                    }
                }

                break;

            // this state receives data sent from the transmitting PC
            case DAQ_STATES::RECEIVE_TEST_DATA:
                // debugln("RECEIVE_TEST_DATA");
                receiveTestDataSerialEvent();
                break;

            // this state perfoms post transmission checks to see if we received the right data
            // packets
            case DAQ_STATES::CONFIRM_TEST_DATA:
                readTestDataFile();
                // debugln("CONFIRM_TEST_DATA");
                break;

            // this state stops the data transmission state 
            case DAQ_STATES::FINISH_DATA_RECEIVE:
                break;
            }

    } // end of test mode
    
}


//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// END OF FLIGHT COMPUTER TESTING SYSTEM  //////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////


//=================== MAIN SYSTEM =================

/**
 * MQTT helper instances, if using MQTT to transmit telemetry
 */
#if MQTT
    WiFiClient wifi_client;
    PubSubClient mqtt_client(wifi_client);
    void MQTTInit(const char* broker_IP, uint16_t broker_port);
#endif

/* WIFI configuration class object */
WIFIConfig wifi_config;

uint8_t drogue_pyro = 25;
uint8_t main_pyro = 12;
uint8_t flash_cs_pin = 5;           /*!< External flash memory chip select pin */
uint8_t remote_switch = 27;


/* Flight data logging */
uint8_t flash_led_pin = 32;                  /*!< LED pin connected to indicate flash memory formatting  */
char filename[] = "flight.bin";             /*!< data log filename - Filename must be less than 20 chars, including the file extension */
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

/**
 * Task synchronization variables
 */
// event group bits 
#define TRANSMIT_TELEMETRY_BIT  ((EventBits_t) 0x01 << 0)   // for bit 0
#define CHECK_FLIGHT_STATE_BIT  ((EventBits_t) 0x01 << 1)   // for bit 1
#define LOG_TO_MEMORY_BIT       ((EventBits_t) 0x01 << 2)   // for bit 2
#define TRANSMIT_XBEE_BIT       ((EventBits_t) 0x01 << 3)   // for bit 3
#define DEBUG_TO_TERM_BIT       ((EventBits_t) 0x01 << 4)   // for bit 4

// event group type for task synchronization
EventGroupHandle_t tasksDataReceiveEventGroup;

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
MPU6050 imu(0x68, 16, 1000);

// create BMP object 
SFE_BMP180 altimeter;
char status;
double T, P, p0, a;
#define ALTITUDE 1525.0 // altitude of iPIC building, JKUAT, Juja. TODO: Change to launch site altitude

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
     * FIXME: Proper GPS init check
     * Look into if the GPS has acquired a LOCK on satelites 
     * Only if it has a lock then can we return a 1
     * 
     * GPS is low priority at the time of writing this but make it work! 
     * */ 

    return 1;
}

/**
 * ///////////////////////// END OF PERIPHERALS INIT /////////////////////////
 */

/* create queue to store altimeter data
 * store pressure and altitude
 * */
QueueHandle_t telemetry_data_qHandle;
QueueHandle_t accel_data_qHandle;
QueueHandle_t altimeter_data_qHandle;
QueueHandle_t gps_data_qHandle;
// QueueHandle_t filtered_data_queue;
// QueueHandle_t flight_states_queue;


// void connectToWifi(){
//     digitalWrite(LED_BUILTIN, HIGH);
//     /* Connect to a Wi-Fi network */
//     debugln("[..]Scanning for network...");

//     WiFi.begin(SSID, PASSWORD);

//     while (WiFi.status() != WL_CONNECTED)
//     {
//         delay(500);
//         debugln("[..]Scanning for network...");
//     }

//     debugln("[+]Network found");debug("[+]My IP address: "); debugln();
//     debugln(WiFi.localIP());
//     digitalWrite(LED_BUILTIN, LOW);
// }


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
        
        xQueueSend(telemetry_data_qHandle, &acc_data_lcl, portMAX_DELAY);

    }
}


//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// ALTITUDE AND VELOCITY DETERMINATION /////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

/*!****************************************************************************
 * @brief Read ar pressure data from the barometric sensor onboard
 * @param pvParameters - A value that is passed as the parameter to the created task.
 * If pvParameters is set to the address of a variable then the variable must still exist when the created task executes - 
 * so it is not valid to pass the address of a stack variable.
 * @return Sends altimeter data to altimeter queue
 *******************************************************************************/
void readAltimeterTask(void* pvParameters){
    telemetry_type_t alt_data_lcl;

    while(true){    
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

                    status = altimeter.getPressure(P, T);
                    if(status != 0) {
                        // print out the measurement
                        // debug("absolute pressure: ");
                        // debug(P, 2);
                        // debug(" mb, "); // in millibars

                        p0 = altimeter.sealevel(P,ALTITUDE);
                        // If you want to determine your altitude from the pressure reading,
                        // use the altitude function along with a baseline pressure (sea-level or other).
                        // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
                        // Result: a = altitude in m.

                        a = altimeter.altitude(P, p0);
                        // debug("computed altitude: ");
                        // debug(a, 0);
                        // debug(" meters, ");

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
        alt_data_lcl.alt_data.pressure = P;
        alt_data_lcl.alt_data.altitude = a;
        alt_data_lcl.alt_data.velocity = 0;
        alt_data_lcl.alt_data.temperature = T;

        // send this pressure data to queue
        // do not wait for the queue if it is full because the data rate is so high, 
        // we might lose some data as we wait for the queue to get space
        xQueueSend(telemetry_data_qHandle, &alt_data_lcl, 0); // TODO: CHECK SUCCESS SENDING TO QUEUE

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
                    debugln("Invalid GPS location");
                }

                if(gps.time.isValid()) {
                    gps_data_lcl.gps_data.time = gps.time.value(); // decode this time value post flight - write a script for that
                } else {
                    debugln("Invalid GPS time");
                }

                if(gps.altitude.isValid()) {
                    gps_data_lcl.gps_data.gps_altitude = gps.altitude.meters();
                } else {
                    debugln("Invalid altitude data"); // TODO: LOG to system logger
                }
            }
        }

        // send to telemetry queue
        if(xQueueSend(telemetry_data_qHandle, &gps_data_lcl, portMAX_DELAY) != pdPASS){
            debugln("[-]GPS queue full"); // TODO: LOG TO SYSTEM LOGGER
        } else {
            // debugln("Sent to GPS Queue"); // TODO: LOG TO SYSTEM LOGGER
        }
    }

}

/*!****************************************************************************
 * @brief dequeue data from telemetry queue after all the tasks have consumed the data
 * @param pvParameters - A value that is passed as the parameter to the created task.
 * If pvParameters is set to the address of a variable then the variable must still exist when the created task executes - 
 * so it is not valid to pass the address of a stack variable.
 * @return none
 * 
 *******************************************************************************/
void clearTelemetryQueueTask(void* pvParameters) {
    telemetry_type_t data_item; // data item to dequeue

    const EventBits_t xBitsToWaitFor = (TRANSMIT_TELEMETRY_BIT | CHECK_FLIGHT_STATE_BIT | LOG_TO_MEMORY_BIT); // todo: ADD  TRANSMIT_TO_XBEE AND DEBUG TO_TERMINAL
    EventBits_t xEventGroupValue;
    
    while (1) {
        xEventGroupValue = xEventGroupWaitBits(
                tasksDataReceiveEventGroup, 		// event group to use
                xBitsToWaitFor, 	                // bit combo to wait for 
                pdTRUE,				                // clear all bits on exit
                pdTRUE, 			                // Wait for all bits (AND)
                portMAX_DELAY		                // wait indefinitely
        );

        // Clear the data item from telemetry queue
        // receiving pops the item from queue, hence removing it entirely
        // which is what we want, after the data item has been distributed to other tasks
        xQueueReceive(telemetry_data_qHandle, &data_item, portMAX_DELAY);

        // check if all data consuming tasks have received the data
        // check TRANSMIT_TELEMETRY TASK
        if(xEventGroupValue & TRANSMIT_TELEMETRY_BIT != 0) {
            // TODO: MUST LOG TO SYSTEM LOGGER
            debugln("[transmit telemetry task receive data OK!]");
        }

        // check CHECK_FLIGHT_STATE_TASK
        if(xEventGroupValue & CHECK_FLIGHT_STATE_BIT != 0) {
            // TODO: MUST LOG TO SYSTEM LOGGER
            debugln("[check state task receive data OK!]");
        }

        // check LOG_TO_MEMORY task 
        if(xEventGroupValue & LOG_TO_MEMORY_BIT != 0) {
            // TODO: MUST LOG TO SYSTEM LOGGER
            debugln("[log to memory task receive data OK!]");
        }

        //TODO: XBEE, DEBUG TO TERM
    }
	
}

/*!****************************************************************************
 * @brief Check and update the current state of flight - refer to states.h
 * @param pvParameters - A value that is passed as the parameter to the created task.
 * If pvParameters is set to the address of a variable then the variable must still exist when the created task executes - 
 * so it is not valid to pass the address of a stack variable.
 * @return Updates the telemetry data flight state value
 * 
 *******************************************************************************/
void checkFlightState(void* pvParameters) {
    // get the flight state from the telemetry tasi
    telemetry_type_t sensor_data; 
    
    while (1) {
        uint8_t s = xQueuePeek(telemetry_data_qHandle, &sensor_data, portMAX_DELAY);
        xEventGroupSetBits(tasksDataReceiveEventGroup, CHECK_FLIGHT_STATE_BIT); // signal that we have received flight data
        

        // check the flight state based on the conditions 
        // this is a dummy condition
        if(sensor_data.acc_data.ax < 5 && sensor_data.acc_data.ay > 2){
            // change flight state to POWERED FLIGHT
            current_state = FLIGHT_STATE::POWERED_FLIGHT;
        } 
        // else if() {}
    }

}

/*!****************************************************************************
 * @brief performs flight actions based on the current flight state
 * If the flight state neccessisates an operation, we perform it here
 * For example if the flight state is apogee, we perform MAIN_CHUTE ejection
 * 
 * @param pvParameter - A value that is passed as the parameter to the created task.
 * If pvParameter is set to the address of a variable then the variable must still exist when the created task executes - 
 * so it is not valid to pass the address of a stack variable.
 * 
 *******************************************************************************/
void flightStateCallback(void* pvParameters) {
    while(1) {
        switch (current_state) {
            // PRE_FLIGHT_GROUND
            case FLIGHT_STATE::PRE_FLIGHT_GROUND:
                debugln("PRE-FLIGHT STATE");
                break;

            // POWERED_FLIGHT
            case FLIGHT_STATE::POWERED_FLIGHT:
                debugln("POWERED FLIGHT STATE");
                break;

            // COASTING
            case FLIGHT_STATE::COASTING:
                debugln("COASTING");
                break;

            // APOGEE
            case FLIGHT_STATE::APOGEE:
                debugln("APOGEE");
                break;

            // DROGUE_DEPLOY
            case FLIGHT_STATE::DROGUE_DEPLOY:
                debugln("DROGUE DEPLOY");
                drogueChuteDeploy();
                break;

            // DROGUE_DESCENT
            case FLIGHT_STATE::DROGUE_DESCENT: 
                debugln("DROGUE DESCENT");
                break;

            // MAIN_DEPLOY
            case FLIGHT_STATE::MAIN_DEPLOY:
                debugln("MAIN CHUTE DEPLOY");
                mainChuteDeploy();
                break;

            // MAIN_DESCENT
            case FLIGHT_STATE::MAIN_DESCENT:
                debugln("MAIN CHUTE DESCENT");
                break;

            // POST_FLIGHT_GROUND
            case FLIGHT_STATE::POST_FLIGHT_GROUND:
                debugln("POST FLIGHT GROUND");
                break;
            
            // MAINTAIN AT PRE_FLIGHT_GROUND IF NO STATE IS SPECIFIED - NOT GONNA HAPPEN BUT BETTER SAFE THAN SORRY
            default:
                debugln(current_state);
                break;

        }
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
    telemetry_type_t rcvd_data; // accelleration received from acceleration_queue

    while(true){
        if(xQueuePeek(telemetry_data_qHandle, &rcvd_data, portMAX_DELAY) == pdPASS){
            // debug CSV to terminal 
            // debug(rcvd_data.acc_data.ax); debug(","); 
            // debug(rcvd_data.acc_data.ay); debug(","); 
            // debug(rcvd_data.acc_data.az); debug(","); 
            // debug(rcvd_data.acc_data.pitch); debug(","); 
            // debug(rcvd_data.acc_data.roll); debug(","); 
            // debug(rcvd_data.alt_data.pressure); debug(","); 
            // debug(rcvd_data.alt_data.velocity); debug(","); 
            // debug(rcvd_data.alt_data.altitude); debug(","); 
            // debug(rcvd_data.alt_data.temperature); debug(","); 

            debug(rcvd_data.gps_data.latitude); debug(","); 
            debug(rcvd_data.gps_data.longitude); debug(",");
            debug(rcvd_data.gps_data.gps_altitude); debug(",");
            debug(rcvd_data.gps_data.time); debug(","); 

            debugln();

        }else{
            /* no queue */
        }

        // if(xQueueReceive(altimeter_data_queue, &altimeter_buffer, portMAX_DELAY) == pdPASS){
        //     debug("Pressure: "); debug(altimeter_buffer.pressure); debugln();
        //     debug("Altitude: "); debug(altimeter_buffer.altitude); debugln();
        //     debug("Velocity: "); debug(altimeter_buffer.velocity); debugln();
        //     debug("AGL: "); debug(altimeter_buffer.AGL); debugln();
            
        // }else{
        //     /* no queue */
        // }

        // if(xQueueReceive(gps_data_queue, &gps_buffer, portMAX_DELAY) == pdPASS){
        //     debug("Latitude: "); debug(gps_buffer.latitude); debugln();
        //     debug("Longitude: "); debug(gps_buffer.longitude); debugln();
        //     debug("Time: "); debug(gps_buffer.time); debugln();
            
        // }else{
        //     /* no queue */
        // }

        xEventGroupSetBits(tasksDataReceiveEventGroup, DEBUG_TO_TERM_BIT); // signal that we have received flight data

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
        xQueuePeek(telemetry_data_qHandle, &received_packet, portMAX_DELAY);
        xEventGroupSetBits(tasksDataReceiveEventGroup, LOG_TO_MEMORY_BIT); // signal that we have received flight data

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

    while(1) {
        if(mqtt_client.publish(MQTT_TOPIC, "Hello from FC1")) {
             debugln("[+]Data sent");
         } else {
             debugln("[-]Data not sent");
         }
    }

}

/*!
 * @brief Try reconnecting to MQTT if connection is lost
 *
 */
void MQTT_Reconnect() {
     while(!mqtt_client.connected()){
         debug("[..]Attempting MQTT connection..."); // TODO: SYS L
         String client_id = "[+]FC Client - ";
         client_id += String(random(0XFFFF), HEX);

         if(mqtt_client.connect(client_id.c_str())){
             debugln("[+]MQTT connected");
         }
    }
}

// void transmitTelemetry(void* pvParameters){
//     /* This function sends data to the ground station */

//      /*  create two pointers to the data structures to be transmitted */
    
//     char telemetry_data[180];
//     struct Acceleration_Data gyroscope_data_receive;
//     struct Altimeter_Data altimeter_data_receive;
//     struct GPS_Data gps_data_receive;
//     int32_t flight_state_receive;
//     int id = 0;

//     while(true){
//         file = SPIFFS.open("/log.csv", FILE_APPEND);
//         if(!file) debugln("[-] Failed to open file for appending");
//         else debugln("[+] File opened for appending");
        
//         /* receive data into respective queues */
//         if(xQueueReceive(gyroscope_data_queue, &gyroscope_data_receive, portMAX_DELAY) == pdPASS){
//             debugln("[+]Gyro data ready for sending ");
//         }else{
//             debugln("[-]Failed to receive gyro data");
//         }

//         if(xQueueReceive(altimeter_data_queue, &altimeter_data_receive, portMAX_DELAY) == pdPASS){
//             debugln("[+]Altimeter data ready for sending ");
//         }else{
//             debugln("[-]Failed to receive altimeter data");
//         }

//         if(xQueueReceive(gps_data_queue, &gps_data_receive, portMAX_DELAY) == pdPASS){
//             debugln("[+]GPS data ready for sending ");
//         }else{
//             debugln("[-]Failed to receive GPS data");
//         }

//         if(xQueueReceive(flight_states_queue, &flight_state_receive, portMAX_DELAY) == pdPASS){
//             debugln("[+]Flight state ready for sending ");
//         }else{
//             debugln("[-]Failed to receive Flight state");
//         }

//         sprintf(telemetry_data,
//             "%i,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%i,%.16f,%.16f,%i,%i\n",
//             id,//0
//             gyroscope_data_receive.ax,//1
//             gyroscope_data_receive.ay,//2
//             gyroscope_data_receive.az,//3
//             gyroscope_data_receive.gx,//4
//             gyroscope_data_receive.gy,//5
//             gyroscope_data_receive.gz,//6
//             altimeter_data_receive.AGL,//7
//             altimeter_data_receive.altitude,//8
//             altimeter_data_receive.velocity,//9
//             altimeter_data_receive.pressure,//10
//             gps_data_receive.latitude,//11
//             gps_data_receive.longitude,//12
//             gps_data_receive.time,//13
//             flight_state_receive//14
//         );
//         if(file.print(telemetry_data)){
//             debugln("[+] Message appended");
//         } else {
//             debugln("[-] Append failed");
//         }
//         file.close();
//         id+=1;

//         if(mqtt_client.publish("n3/telemetry", telemetry_data)) {
//             debugln("[+]Data sent");
//         } else{
//             debugln("[-]Data not sent");
//         }
//     }
// }

/*!****************************************************************************
 * @brief Initialize MQTT
 *
 *
 *******************************************************************************/
void MQTTInit(const char* broker_IP, int broker_port) {
    // mqtt_client.setBufferSize(MQTT_BUFFER_SIZE);
    mqtt_client.setServer(broker_IP, broker_port);
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
 * HIGH for a preset No. of seconds.  
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
void setup(){
    /* initialize serial */
    Serial.begin(BAUDRATE);
    delay(100);

    debugln();
    debugln(F("=============================================="));
    debugln(F("========= INITIALIZING PERIPHERALS ==========="));
    debugln(F("=============================================="));

    uint8_t bmp_init_state = BMPInit();
    uint8_t imu_init_state = imu.init();
    uint8_t gps_init_state = GPSInit();
    uint8_t sd_init_state = initSD();
    initDataFiles();
    uint8_t spiffs_init_state = InitSPIFFS();
    uint8_t test_gpio_init_state = initTestGPIO();
    // uint8_t mqtt_init_state = MQTTInit(MQTT_SERVER, MQTT_PORT);    

    /* update the susbsystems init state table */   
    // check if BMP init OK
    if(bmp_init_state) { 
        SUBSYSTEM_INIT_MASK |= (1 << BMP_CHECK_BIT);
    }

    // check if MPU init OK
    if(imu_init_state)  {
        SUBSYSTEM_INIT_MASK |= (1 << IMU_CHECK_BIT);
    }

    // check if flash memory init OK
    // if (flash_init_state) {
    //     SUBSYSTEM_INIT_MASK |= (1 << FLASH_CHECK_BIT);
    // }

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

    // check if test hardware init OK
    if(test_gpio_init_state) {
        SUBSYSTEM_INIT_MASK |= (1 << TEST_HARDWARE_CHECK_BIT);
    }

    debug("SUBSYSTEM_INIT_MASK: "); debugln(SUBSYSTEM_INIT_MASK);

    // delay(2000);

    /* check whether we are in TEST or RUN mode */
    checkRunTestToggle();

    //////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////// FLIGHT COMPUTER TESTING SYSTEM  /////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////
    if(DAQ_MODE) {
        // in test mode we only transfer test data from the testing PC to the SD card
        debugln();
        debugln(F("=============================================="));
        debugln(F("=========FLIGHT COMPUTER DATA ACQUISITION MODE========="));
        debugln(F("=============================================="));

        debugln("Ready to receive data...");

        //////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////// END OF FLIGHT COMPUTER TESTING SYSTEM  //////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////
            
    }  else if (TEST_MODE) {
        debugln();
        debugln(F("=============================================="));
        debugln(F("========= TEST MODE ========="));
        debugln(F("=============================================="));

        /**
         * We need to read the TEST state from a file in the SD card 
         * This file stores the state we are in permanently, so that next time we reset while in run mode,
         * we have a reference state to use
         * 
         * This file will be updated in the loop once we are done consuming the test data 
         */
        readStateFile(SD, "/state.txt");
        debugln(current_test_state_buffer);

        /**
         * check what state we are in while in the test state. 
         * If we are in the data consume state, set the current state to DATA_CONSUME
         * The state is got from the SD card state.txt file
         */
        if (strcmp(current_test_state_buffer, "DATA_CONSUME\r\n") == 0) {
            current_test_state = TEST_STATES::DATA_CONSUME;
            debugln("STATE set to DATA CONSUME ");
        } else {
            debugln("current state undefined... ");
        }

        // create dynamic WIFI
        uint8_t wifi_connection_result = wifi_config.WifiConnect();
        if(wifi_connection_result) {
            debugln("Wifi config OK!");
        } else {
            debugln("Wifi config failed");
        }

        /* mode 0 resets the system log file by clearing all the current contents */
        // system_logger.logToFile(SPIFFS, 0, rocket_ID, level, system_log_file, "Game Time!"); // TODO: DEBUG

        debugln();
        debugln(F("=============================================="));
        debugln(F("===== INITIALIZING DATA LOGGING SYSTEM ======="));
        debugln(F("=============================================="));
        
        data_logger.loggerInit();

        uint8_t app_id = xPortGetCoreID();
        BaseType_t th; // task creation handle
            
        debugln();
        debugln(F("=============================================="));
        debugln(F("============== CREATING QUEUES ==============="));
        debugln(F("=============================================="));

        // this queue holds the data from MPU 6050 - this data is filtered already
        accel_data_qHandle = xQueueCreate(GYROSCOPE_QUEUE_LENGTH, sizeof(accel_type_t)); 

        // this queue hold the data read from the BMP180
        altimeter_data_qHandle = xQueueCreate(ALTIMETER_QUEUE_LENGTH, sizeof(altimeter_type_t)); 

        /* create gps_data_queue */   
        gps_data_qHandle = xQueueCreate(GPS_QUEUE_LENGTH, sizeof(gps_type_t));

        // this queue holds the telemetry data packet
        telemetry_data_qHandle = xQueueCreate(TELEMETRY_DATA_QUEUE_LENGTH, sizeof(telemetry_packet));

        // /* this queue will hold the flight states */
        // flight_states_queue = xQueueCreate(FLIGHT_STATES_QUEUE_LENGTH, sizeof(int32_t));

        /* check if the queues were created successfully */
        if(accel_data_qHandle == NULL){
            debugln("[-]accel data queue creation failed");
        } else{
            debugln("[+]Acceleration data queue creation OK.");
        }
        
        if(altimeter_data_qHandle == NULL){
            debugln("[-]Altimeter data queue creation failed");
        } else{
            debugln("[+]Altimeter data queue creation OK.");
        }

        if(gps_data_qHandle == NULL){
            debugln("[-]GPS data queue creation failed");
        } else{
            debugln("[+]GPS data queue creation OK.");
        }

        if(telemetry_data_qHandle == NULL) {
            debugln("[-]Telemetry data queue creation failed");
        } else {
            debugln("[+]Telemetry data queue creation OK.");
        }

        // if(filtered_data_queue == NULL){
        //     debugln("[-]Filtered data queue creation failed!");
        // } else{
        //     debugln("[+]Filtered data queue creation OK.");
        // }

        // if(flight_states_queue == NULL){
        //     debugln("[-]Flight states queue creation failed!");
        // } else{
        //     debugln("[+]Flight states queue creation OK.");
        // }


        // create  event group to sync flight data consumption
        // see N4 flight software docs for more info
        debugln();
        debugln(F("=============================================="));
        debugln(F("===== CREATING DATA CONSUMER EVENT GROUP ===="));
        debugln(F("=============================================="));

        tasksDataReceiveEventGroup = xEventGroupCreate(); // pss! whatever u do, this line must come before creating the tasks!
        if(tasksDataReceiveEventGroup == NULL) {
            debugln("[-] data consumer event group failed to create");
        } else {
            debugln("[+] data consumer event group created OK.");
        }

        debugln();
        debugln(F("=============================================="));
        debugln(F("============== CREATING TASKS ==============="));
        debugln(F("=============================================="));

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
        * Task priority - in this case 1 
        * task handle that can be passed to other tasks to reference the task 
        *
        * /
        debugln("==============Creating tasks==============");

        /* TASK 1: READ ACCELERATION DATA */
        th = xTaskCreatePinnedToCore(readAccelerationTask, "readGyroscope", STACK_SIZE*2, NULL, 1, NULL,app_id);
        if(th == pdPASS) {
            debugln("[+]Read acceleration task created");
        } else {
            debugln("[-]Read acceleration task creation failed");
        }

        /* TASK 2: READ ALTIMETER DATA */
        th = xTaskCreatePinnedToCore(readAltimeterTask,"readAltimeter",STACK_SIZE*2,NULL,2,NULL,app_id);
        if(th == pdPASS) {
            debugln("[+]Read altimeter task created OK.");
        } else {
            debugln("[-]Failed to create read altimeter task");
        }

        /* TASK 3: READ GPS DATA */
        th = xTaskCreatePinnedToCore(readGPSTask, "readGPS", STACK_SIZE*2, NULL,1,NULL, app_id);

        if(th == pdPASS) {
            debugln("[+]GPS task created OK.");
        } else {
            debugln("[-]Failed to create GPS task");
        }

        /* TASK 4: CLEAR TELEMETRY QUEUE ITEM */
        th = xTaskCreatePinnedToCore(clearTelemetryQueueTask,"clearTelemetryQueueTask",STACK_SIZE*2,NULL,1, NULL,app_id);

        if(th == pdPASS) {
            debugln("[+]clearTelemetryQueueTask task created OK.");
        } else {
            debugln("[-]Failed to create clearTelemetryQueueTask task");
        }

        /* TASK 5: CHECK FLIGHT STATE TASK */
        th = xTaskCreatePinnedToCore(checkFlightState,"checkFlightState",STACK_SIZE*2,NULL,1, NULL,app_id);
        if(th == pdPASS) {
            debugln("[+]checkFlightState task created OK.");
        } else {
            debugln("[-}Failed to create checkFlightState task");
        }

        /* TASK 6: FLIGHT STATE CALLBACK TASK */    
        th = xTaskCreatePinnedToCore(flightStateCallback,"flightStateCallback",STACK_SIZE*2,NULL,1, NULL,app_id);
        if(th == pdPASS) {
            debugln("[+]flightStateCallback task created OK.");
        } else {
            debugln("[-}Failed to create flightStateCallback task");
        }

        #if DEBUG_TO_TERMINAL   // set DEBUG_TO_TERMINAL to 0 to prevent serial debug data to serial monitor

        /* TASK 7: DISPLAY DATA ON SERIAL MONITOR - FOR DEBUGGING */
        th = xTaskCreatePinnedToCore(debugToTerminalTask,"debugToTerminalTask",STACK_SIZE,NULL,1,NULL,app_id);
        
        if(th == pdPASS) {
            debugln("[+}debugToTerminalTaskTask created");
        } else {
            debugln("[-}Task not created");
        }

        #endif // DEBUG_TO_TERMINAL_TASK

        /* TASK 8: TRANSMIT TELEMETRY DATA */
        th = xTaskCreatePinnedToCore(MQTT_TransmitTelemetry, "transmit_telemetry", STACK_SIZE*2, NULL, 2, NULL, app_id);
        if(th == pdPASS){
            debugln("[+]Transmit task created OK!");
        } else {
            debugln("[-]Transmit task failed to create");
        }

        #if LOG_TO_MEMORY   // set LOG_TO_MEMORY to 1 to allow logging to memory 
            /* TASK 9: LOG DATA TO MEMORY */
            if(xTaskCreate(
                    logToMemory,
                    "logToMemory",
                    STACK_SIZE,
                    NULL,
                    1,
                    NULL
            ) != pdPASS){
                debugln("[-]logToMemory task failed to create");
    
            }else{
                debugln("[+]logToMemory task created OK.");
            }
        #endif // LOG_TO_MEMORY

        // if(xTaskCreate(
        //         flight_state_check,
        //         "testFSM",
        //         STACK_SIZE,
        //         NULL,
        //         2,
        //         NULL
        // ) != pdPASS){
        //     debugln("[-]FSM task failed to create");
        // }else{
        //     debugln("[+]FSM task created OK.");
        // }

    } // end of setup in running mode 
    
}


/*!****************************************************************************
 * @brief Main loop
 *******************************************************************************/
void loop() {
    //     if(WiFi.status() != WL_CONNECTED){
    //         WiFi.begin(SSID, PASSWORD);
    //         delay(500);
    //         debug(".");
    //     }


    // check if the testing system is done receiving the test data 
    // 
    // if(current_test_state == TEST_STATE::FINISH_DATA_RECEIVE) {
    //     // consume the received test data 
    //     readFile(SD, "/data.txt");
    // }

    if(DAQ_MODE) {

        //////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////// FLIGHT COMPUTER TESTING SYSTEM  /////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////
        prepareForDataReceive();

        //////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////// END OF FLIGHT COMPUTER TESTING SYSTEM  ////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////

    } else if(TEST_MODE) {

        // check the current mode
        // if(current_run_state)



        // if( !mqtt_client.connected() ) {
        //     /* try to reconnect if connection is lost */
        //     MQTT_Reconnect();
        // }

        // mqtt_client.loop();

    }

}
