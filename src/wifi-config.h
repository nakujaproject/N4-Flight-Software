#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h>

class WIFIConfig {
private:
    Preferences preferences;
    char basestation_ip[16] = "192.168.100.248";  // Default IP
    char mqtt_port[6] = "1883";                    // Default port
    
public:
    /**
     * @brief Connects to WiFi in either MQTT or Beacon mode.
     * 
     * @param enable_ap_mode If true, sets WiFi to AP+STA and sets rocket MAC.
     * @param rocket_mac Pointer to 6-byte MAC to set as AP MAC (only if enable_ap_mode is true).
     * @return uint8_t 1 = success, 0 = failure
     */
    uint8_t WifiConnect(bool enable_ap_mode, const uint8_t* rocket_mac);
    
    /**
     * @brief Get the configured base station IP address
     * @return const char* IP address string
     */
    const char* getBaseStationIP();
    
    /**
     * @brief Get the configured MQTT port
     * @return int MQTT port number
     */
    int getMQTTPort();
    
    /**
     * @brief Save IP configuration to persistent storage
     */
    void saveConfig();
    
    /**
     * @brief Load IP configuration from persistent storage
     */
    void loadConfig();
};

#endif