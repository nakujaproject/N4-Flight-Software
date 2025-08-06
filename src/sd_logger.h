#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <Arduino.h>
#include <SD.h>
#include "defs.h"
#include "data_types.h"
// extern QueueHandle_t pckt_buff;
// extern TickType_t t;

extern telemetry_type_t t;
extern char pckt_buff[256];


class SDLogger {
public:
    SDLogger(uint8_t csPin);
    bool begin();
    void setFilename(const char* fname);
    bool log(const telemetry_type_t& packet, const gps_type_t& gps);
    bool initialized() const;

private:
    uint8_t cs;
    const char* filename = "/flight_data.txt";
    bool isInitialized = false;

    void disableOtherDevices();
    void writeHeaderIfNew();
};

#endif 