/**
 * @file system_logger.cpp
 * 
 * Implement flight logging to SPIFFS functions 
 */

#include "system_logger.h"

/**
 * @brief write event logs to file 
 * @param 
 */
void SystemLogger::logToFile (fs::FS &fs, uint8_t mode, const char* client, uint8_t log_level, const char* file,  const char* msg) {
    char log_buffer[256];
    // get the timestamp
    unsigned long raw_timestamp = millis();


    // construct the log message
    // timestamp clientID log_level msg
    sprintf(log_buffer,
            "%d:%s:%s:%s\n",
            raw_timestamp,
            client,
            this->getLogLevelString(log_level),
            msg
    );

    // open the file in the specified mode
    if(mode == 0) {
        // clear the file contents
        // this is used just before flight to make sure we do not have previous data
        // on the log file
        File f = fs.open(file, FILE_WRITE);
        if(!f) {
            Serial.println("Failed to open file ");
            return;
        }

        // log to file
        if(f.print(log_buffer)) {
            Serial.println("Log OK");
        } else {
            Serial.println("- write failed");
        }

        // close file
        f.close();

    } else if(mode == 1) {
        File f = fs.open(file, FILE_APPEND);
        if(!f) {
            Serial.println("Failed to open file ");
            return;
        }

        // log to file
        if(f.print(log_buffer)) {
            Serial.println("Log OK");
        } else {
            Serial.println("- write failed");
        }

        // close file
        f.close();

    }
}

/**
 * @brief read log file to console
 */
 void SystemLogger::readLogFile(fs::FS &fs, const char* file) {
        Serial.printf("Reading file: %s\r\n", file);
        File f = fs.open(file);

        if(!f || f.isDirectory()){
            Serial.println("- failed to open file for reading");
            return;
        }

        Serial.println("- read from file:");
        while(f.available()){
            Serial.write(f.read());
        }
        Serial.println();
        f.close();
 }

/**
 * 
 * @brief convert the log level to string
 * 
 */
const char* SystemLogger::getLogLevelString(uint8_t log_level) {
    static const char* debug = "DEBUG";
    static const char* info = "INFO";
    static const char* warning = "WARNING";
    static const char* critical = "CRITICAL";
    static const char* error = "ERROR";
    static const char* unknown = "UNKNOWN";

    switch (log_level) {
    case 0:
        return debug;
        break;

    case 1:
        return info;
        break;

    case 2:
        return warning;
        break;

    case 3:
        return critical;
        break;
    
    case 4:
        return error;
        break;
    
    default:
        return unknown;
        break;
    }

}



