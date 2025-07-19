#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>git 

class WIFIConfig {
public:
    /**
     * @brief Connects to WiFi in either MQTT or Beacon mode.
     * 
     * @param enable_ap_mode If true, sets WiFi to AP+STA and sets rocket MAC.
     * @param rocket_mac Pointer to 6-byte MAC to set as AP MAC (only if enable_ap_mode is true).
     * @return uint8_t 1 = success, 0 = failure
     */
    uint8_t WifiConnect(bool enable_ap_mode, const uint8_t* rocket_mac);
};

#endif