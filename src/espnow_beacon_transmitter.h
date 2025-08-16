#ifndef ESPNOW_BEACON_TRANSMITTER_H
#define ESPNOW_BEACON_TRANSMITTER_H

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

#define MAX_COMMAND_LENGTH 20
#define MAX_BEACON_SIZE 256



class ESPNowBeaconTransmitter {
public:
    struct CommandPacket {
        uint32_t timestamp;
        uint8_t length;
        uint8_t command[MAX_COMMAND_LENGTH];
    };
    bool isArmed() const { return armed; }
    void setArmed(bool state) { armed = state; } 


    ESPNowBeaconTransmitter(const uint8_t* rocket_mac, const uint8_t* base_mac);
    ~ESPNowBeaconTransmitter();
    
    bool begin();
    bool sendBeacon(const void* telemetry_data, size_t telemetry_size);
    bool getNextCommand(CommandPacket* packet);
    
    void setTelemetryCallback(void (*callback)(void* data, size_t size)) {
        telemetryCallback = callback;
    }

private:
    static void OnDataRecv(const uint8_t* mac, const uint8_t* data, int len);
    void buildBeaconFrame(uint8_t* frame, size_t* frame_size, 
                         const void* telemetry_data, size_t telemetry_size);
    
    uint8_t rocketMAC[6];
    uint8_t baseMAC[6];
    bool armed = false;
    uint32_t beaconCounter = 0;
    QueueHandle_t commandQueue;
    
    void (*telemetryCallback)(void* data, size_t size) = nullptr;
    static ESPNowBeaconTransmitter* instance;
};

#endif