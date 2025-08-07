/**
 * @file logger.cpp
 * @brief Implement onboard logger class functions
 *
*/

#include "logger.h"
#include "data_types.h"
#include "defs.h"

telemetry_type_t t;
char pckt_buff[150];


/**
 * @brief class constructor 
 * pass the chip select pin as a parameter for that class instance
 * 
 * @param cs_pin chip select pin
 * @param flash_led LED to show formatting status
 * @param filename the filename of the file being created
 * @param file_size the size of the file being created
*/
DataLogger::DataLogger(uint8_t cs_pin, uint8_t led_pin, char* filename, SerialFlashFile file, uint32_t filesize) {
    this->_cs_pin = cs_pin;
    this->_led_pin = led_pin;
    this->_file_size = filesize;

    strcpy(this->_filename, filename);
    this->_file = file;

}

/**
 * @brief format the flash memory
 * @param none
*/
void DataLogger::loggerFormat() {
    this->loggerEquals();
    Serial.println(F("Formatting flash memory..."));
 
    SerialFlash.eraseAll();

    // create a dummy file so that next time we start up, we know there
    // is an actual file system
    if(SerialFlash.create("dummy.txt", 100)) {
        Serial.println(F("Created dummy file "));
        SerialFlashFile file;
        file = SerialFlash.open("dummy.txt");
        file.write("Recovery team", 14);
        file.close();
    } else {
        Serial.println(F("Failed to create dummy file "));
    }   
    
    Serial.println(F("Done"));

    // while the flash is formatting, blink the LED at a frequency of 10Hz
    //while(!SerialFlash.ready()) {
        //digitalWrite(this->_led_pin, HIGH);
       // delay(_flash_delay);
        //digitalWrite(this->_led_pin, LOW);
        //delay(_flash_delay);
    //}

    // remain OFF once formatting is done
    //digitalWrite(this->_led_pin, LOW);

    this->loggerEquals();
}

/**
 * @brief Initialize the flash memory 
 * @return true on success and false on fail
 * 
*/
bool DataLogger::loggerInit() {
    char filename[20];

    if (!SerialFlash.begin(this->_cs_pin)) {
        return false;

    } else {
        // this->loggerEquals();
        // this->loggerInfo();

        // // return a list of files currently in the memory
        // if(!SerialFlash.exists("dummy.txt")) {
        //     Serial.println(F("Flash doesn't appear to hold a file system - may need erasing first.")); // TODO: Log to system logger

        //     // format the memory
        //     this->loggerFormat();

        // } else {
        //     Serial.println(F("File system found"));
        //     Serial.println(F("Files currently in flash:")); // TODO: LOG TO SYSTEM LOGGER
        //     SerialFlash.opendir();

        //     // list all files in memory
        //     while (1) {
        //         uint32_t filesize;
        //         if (SerialFlash.readdir(filename, sizeof(filename), filesize)) {
        //             Serial.print(filename);
        //             Serial.print(F("  "));
        //             Serial.print(filesize);
        //             Serial.print(F(" bytes"));
        //             Serial.println();
        //         }
        //         else {
        //             break; 
        //         }
        //     }

            if(SerialFlash.exists(this->_filename)) {
                // erase file contents 
                Serial.println("flight_data.txt file found. Erasing file contents");
                SerialFlashFile flight_file;
                flight_file = SerialFlash.open(this->_filename);
                flight_file.erase();
                Serial.println("Done erasing file contents");
                
                // 🔥 FIX: Re-open the file after erasing for CSV logging
                this->_file = SerialFlash.open(this->_filename);
                if (this->_file) {
                    Serial.println("✅ Flash file re-opened successfully for CSV logging");
                } else {
                    Serial.println("❌ Failed to re-open flash file after erase");
                }
            } else {
                Serial.println("flightk_data.txt file does not exist. Creating file...");
                uint8_t file_create_status = SerialFlash.createErasable(this->_filename, this->_file_size);
                if (!file_create_status) {
                    Serial.println(F("Failed to create file"));
                } else {
                    Serial.println(F("Created flight_data.txt file. Ready for data logging!"));
                    this->_file = SerialFlash.open(this->_filename);
                    if (this->_file) {
                        Serial.println("✅ New flash file opened successfully for CSV logging");
                    } else {
                        Serial.println("❌ Failed to open newly created flash file");
                    }
                }

            }

        // }
        
        // this->loggerEquals(); 

        return true;
    }
}

/**
 * @brief test the flash memory write and read function by reading and 
 * writing a variable to it
 * @param none
 * @return true if R/W OK, false otherwise
 * 
*/
void DataLogger::loggerTest() {
    // create a string variable 
    char tst_var[15] = "FlashTesting";
}

/**
 * @brief write the provided data to the file created
 * @param data this is a struct pointer to the struct that contains the data that needs to 
 * be written to the memory
 * 
 * 
*/
// void DataLogger::loggerWrite(telemetry_type_t packet){
//     // write the record to the flash chip
    
//     // sprintf(pckt_buff, 
//     //         "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
//     //         packet.acc_data.ax,
//     //         packet.acc_data.ay,
//     //         packet.acc_data.az,
//     //         packet.acc_data.pitch,
//     //         packet.acc_data.roll,
//     //         packet.alt_data.pressure);

//     // write the packet to memory
//     this->_file.write((uint8_t*)&packet, sizeof(packet));

//     // Serial.print( packet.record_number );
//     // Serial.print( "," );
//     // Serial.print( packet.operation_mode );
//     // Serial.print( "," );
//     // Serial.print( packet.state );
//     // Serial.print( "," );
//     // Serial.print( packet.acc_data.ax );
//     // Serial.print( "," );
//     // Serial.print( packet.acc_data.ay );
//     // Serial.print( "," );
//     // Serial.print( packet.acc_data. az );
//     // Serial.print( "," );
//     // Serial.print( packet.acc_data.pitch );
//     // Serial.print( "," );
//     // Serial.print( packet.acc_data.roll );
//     // Serial.print( "," );
//     // Serial.print( packet.gyro_data.gx );
//     // Serial.print( "," );
//     // Serial.print( packet.gyro_data.gy );
//     // Serial.print( "," );
//     // Serial.print( packet.gyro_data.gz );
//     // Serial.print( "," );
//     // Serial.print( packet.alt_data.altitude );
//     // Serial.print( "," );
//     // Serial.print( packet.alt_data.velocity );
//     // Serial.print( "," );
//     // Serial.print( packet.alt_data.pressure );
//     // Serial.print( "," );
//     // Serial.println( packet.alt_data.temperature );

//     // Serial.println(F("logged"));
    
//     // at this point, the flash memory is ready for writing and reading 
//     // check that the passed struct is not null
//     // if(data == NULL) {
//     //     // do sth here 
//     //     // maybe log error
//     // } else {
//     //     // data valid, ready to proceed
//     // }

//     // TODO: maybe return the size of memory written 

// }

void DataLogger::loggerWrite(telemetry_type_t packet) {
    extern gps_type_t gps_packet;
    extern altimeter_type_t altimeter_packet;
    telemetry_type_t telemetry_received_packet;
    

    // sprintf(pckt_buff,
    //     "%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%d,%d,%.2f,%d\n",
    //     packet.record_number,
    //     packet.operation_mode,
    //     packet.state,
    //     packet.acc_data.ax,
    //     packet.acc_data.ay,
    //     packet.acc_data.az,
    //     packet.acc_data.pitch,
    //     packet.acc_data.roll,
    //     packet.gyro_data.gx,
    //     packet.gyro_data.gy,
    //     packet.gyro_data.gz,
    //     gps_packet.latitude,
    //     gps_packet.longitude,
    //     gps_packet.gps_altitude,
    //     altimeter_packet.pressure,
    //     altimeter_packet.temperature,
    //     altimeter_packet.rel_altitude,
    //     altimeter_packet.kalman_altitude,           // 19replace altitude after drone test
    //     altimeter_packet.kalman_vertical_velocity,  // 20replace vertical velocity after drone test
    //     telemetry_received_packet.drogue_pin_state, // 21
    //     telemetry_received_packet.main_chute_pin_state, // 22
    //     telemetry_received_packet.battery_voltage,  // 23 - battery voltage
    //     telemetry_received_packet.wifi_rssi
        
    // );
    
    sprintf(pckt_buff,
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

    // Write it to flash as a string
    this->_file.write((uint8_t*)pckt_buff, strlen(pckt_buff));

    // Optional: echo to serial for debugging
    Serial.print("[LOGGED]: ");
    Serial.print(pckt_buff);
}

/*!****************************************************************************
 * @brief Write CSV string directly to flash memory - OPTIMIZED for queue-based logging
 * @param csv_string - Pre-formatted CSV string ready for logging
 *******************************************************************************/
void DataLogger::loggerWriteCSV(const char* csv_string) {
    static uint32_t csv_record_count = 0;  // Track number of CSV records logged
    
    // 🛡️ SAFETY CHECK: Verify flash file is open and valid
    if (!this->_file) {
        Serial.println("❌ [CSV LOG ERROR] Flash file not open!");
        return;
    }
    
    // 🛡️ SAFETY CHECK: Verify CSV string is valid
    if (!csv_string || strlen(csv_string) == 0) {
        Serial.println("❌ [CSV LOG ERROR] Invalid CSV string!");
        return;
    }
    
    size_t csv_length = strlen(csv_string);
    if (csv_length > 512) {  // Reasonable limit
        Serial.println("❌ [CSV LOG ERROR] CSV string too long!");
        return;
    }
    
    // 🛡️ MEMORY SAFE: Direct write of CSV string to flash memory with proper error checking
    size_t bytes_written = this->_file.write(csv_string, csv_length);
    csv_record_count++;
    
    // Enhanced debug output with record count and status
    Serial.print("🔥 [CSV LOGGED #");
    Serial.print(csv_record_count);
    Serial.print("] (");
    Serial.print(bytes_written);
    Serial.print("/");
    Serial.print(csv_length);
    Serial.print(" bytes): ");
    
    // Only print first 50 chars to avoid overflow
    String preview = String(csv_string);
    if (preview.length() > 50) {
        preview = preview.substring(0, 50) + "...";
    }
    Serial.println(preview);
    
    // Check write success
    if (bytes_written != csv_length) {
        Serial.println("⚠️ [CSV LOG WARNING] Partial write detected!");
    } else {
        Serial.println(" ✅ Write successful!");
    }
}


/**
 * @brief Read data from the start of the file to the end of the file 
 * 
 * @param _file_pointer pointer to where we want to start reading the file. By default, this value os 0
 * @param buffer char array to read the data into
*/
void DataLogger::loggerRead(uint8_t file_pointer, char buffer) {
    // confirm the file exists
    // seek the file to the start 


}

/**
 * @brief print the data about the flash memory
 *  
*/
void DataLogger::loggerInfo() {
    uint8_t id[5];
    Serial.println(F("Data logging system check"));
    SerialFlash.readID(id);
    Serial.println(F("Data logging system OK!"));
    Serial.print(F("Capacity: "));
    Serial.print(SerialFlash.capacity( id ) / MB_SIZE_DIVISOR);
    Serial.println(" MB");

}

/**
 * @brief helper function to print spaces for data formatting
*/
void DataLogger::loggerSpaces() {
    for(int i = 0; i < 25; i++) {
        Serial.println(F(" "));
    }
}

/**
 * @brief helper function to print = for data formatting
*/
void DataLogger::loggerEquals() {
    Serial.println();
    for(int i = 0; i < 25; i++) {
        Serial.print("=");
    }
    Serial.println();
}


