/**
 * @file system_logger.h
 * 
 * Declarations for flight events logging 
 */

#ifndef SYSTEMLOGGER_H
#define SYSTEMLOGGER_H

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include "system_log_levels.h"

class SystemLogger {
	public:
        const char* getLogLevelString(uint8_t log_level);
		void logToConsole (const uint32_t timestamp, const char* client, uint8_t log_level, const char* msg);
		void logToFile (fs::FS &fs, uint8_t mode, const char* client, uint8_t log_level, const char* file,  const char* msg);
		void readLogFile(fs::FS &fs, const char* file);
};
	
#endif
