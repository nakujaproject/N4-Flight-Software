#include "wifi-config.h"

uint8_t WIFIConfig::WifiConnect(bool enable_ap_mode, const uint8_t* rocket_mac) {
    if (enable_ap_mode) {
        // Configure AP + STA mode for beacon transmission
        Serial.println("[WiFiConfig] Set mode: WIFI_AP_STA");
        WiFi.mode(WIFI_AP_STA);                                // 1. Set mode
        esp_wifi_set_mac(WIFI_IF_AP, rocket_mac);
        esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);        // 3. Set channel

        // No WiFiManager — skip real WiFi connection
        return 1;
    } else {
        // Use WiFiManager for infrastructure connection (MQTT)
        Serial.println("[WiFiConfig] Set mode: WIFI_STA");
        WiFi.mode(WIFI_STA);
        WiFiManager wm;

        wm.setConfigPortalTimeout(180);
        wm.setBreakAfterConfig(true);

        Serial.println("[WiFiConfig] Starting WiFiManager...");
        bool connected = wm.autoConnect("flight-computer-1");

        if (!connected) {
            Serial.println("[WiFiConfig] WiFi connection FAILED or timed out.");
            return 0;
        } else {
            Serial.print("[WiFiConfig] Connected successfully. IP: ");
            Serial.println(WiFi.localIP());
            return 1;
        }
    }
}
