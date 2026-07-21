#include "wifi-config.h"

const char* WIFIConfig::getBaseStationIP() {
    return basestation_ip;
}

int WIFIConfig::getMQTTPort() {
    return atoi(mqtt_port);
}

void WIFIConfig::saveConfig() {
    preferences.begin("wifi-config", false);
    preferences.putString("basestation_ip", basestation_ip);
    preferences.putString("mqtt_port", mqtt_port);
    preferences.end();
    Serial.println("[WiFiConfig] Configuration saved to preferences");
}

void WIFIConfig::loadConfig() {
    preferences.begin("wifi-config", true);
    
    String saved_ip = preferences.getString("basestation_ip", "192.168.100.248");
    String saved_port = preferences.getString("mqtt_port", "1883");
    
    strcpy(basestation_ip, saved_ip.c_str());
    strcpy(mqtt_port, saved_port.c_str());
    
    preferences.end();
    
    Serial.printf("[WiFiConfig] Loaded config - IP: %s, Port: %s\n", basestation_ip, mqtt_port);
}

uint8_t WIFIConfig::WifiConnect(bool enable_ap_mode, const uint8_t* rocket_mac) {
    // Load saved configuration first
    loadConfig();

    // Always ensure defaults are set before WiFiManager runs
    if (strlen(basestation_ip) == 0 || strcmp(basestation_ip, "0.0.0.0") == 0) {
        strcpy(basestation_ip, "192.168.100.45");
    }
    if (strlen(mqtt_port) == 0) {
        strcpy(mqtt_port, "1883");
    }

    if (enable_ap_mode) {
        // Configure AP + STA mode for beacon transmission
        Serial.println("[WiFiConfig] Set mode: WIFI_AP_STA");
        WiFi.mode(WIFI_AP_STA);                                // 1. Set mode
        esp_wifi_set_mac(WIFI_IF_AP, rocket_mac);              // 2. Set custom AP MAC (optional)
        esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);        // 3. Fixed channel for beacon / ESP-NOW

        // Bring up a minimal AP so WIFI_IF_AP is active for esp_wifi_80211_tx
        // Open network (no password) for lowest overhead; change if security needed
    // Start open AP (no password) for beacon/ESP-NOW layer
    bool ap_ok = WiFi.softAP("N4-FC");
        if(!ap_ok){
            Serial.println("[WiFiConfig] softAP start FAILED");
            return 0;
        } else {
            Serial.print("[WiFiConfig] AP started. IP: ");
            Serial.println(WiFi.softAPIP());
        }

        // Short delay to ensure AP is fully up before ESP-NOW init
        delay(100);

        // No WiFiManager — skip infrastructure connection
        return 1;
    } else {
        // Use WiFiManager for infrastructure connection (MQTT)
        Serial.println("[WiFiConfig] Set mode: WIFI_STA");
        WiFi.mode(WIFI_STA);

        WiFiManager wm;

        // 🔄 FACTORY RESET: Uncomment the line below to completely forget WiFi credentials
        //wm.resetSettings(); // ⚠️ UNCOMMENT THIS LINE TO FORGET WIFI & FORCE RECONFIGURATION

        // Create custom parameters for base station configuration (with valid defaults)
        WiFiManagerParameter custom_basestation_ip("basestation_ip", "Base Station IP Address", basestation_ip, 16);
        WiFiManagerParameter custom_mqtt_port("mqtt_port", "MQTT Port", mqtt_port, 6);

        // Add parameters to WiFiManager
        wm.addParameter(&custom_basestation_ip);
        wm.addParameter(&custom_mqtt_port);

        // Set save config callback
        wm.setSaveConfigCallback([this]() {
            Serial.println("[WiFiConfig] Should save config flag set");
        });

        wm.setConfigPortalTimeout(180);
        wm.setBreakAfterConfig(true);

        // Add custom HTML for better UI
        const char* custom_head = "<style>body{background:#2c3e50;color:#ecf0f1;font-family:Arial,sans-serif;}.c{text-align:center;}.wrap{text-align:left;display:inline-block;min-width:260px;max-width:500px;}</style>";
        wm.setCustomHeadElement(custom_head);

        Serial.println("[WiFiConfig] Starting WiFiManager with base station configuration...");
        Serial.printf("[WiFiConfig] Current base station IP: %s\n", basestation_ip);
        Serial.printf("[WiFiConfig] Current MQTT port: %s\n", mqtt_port);

        bool connected = wm.autoConnect("N4-Flight-Computer-Setup");

        // Always update config from portal, even if not connected
        strcpy(basestation_ip, custom_basestation_ip.getValue());
        strcpy(mqtt_port, custom_mqtt_port.getValue());

        // Validate IP address format
        IPAddress test_ip;
        if (!test_ip.fromString(basestation_ip)) {
            Serial.println("[WiFiConfig] Invalid IP address format, using default");
            strcpy(basestation_ip, "192.168.100.45");
        }

        // Validate port number
        int port_num = atoi(mqtt_port);
        if (port_num < 1 || port_num > 65535) {
            Serial.println("[WiFiConfig] Invalid port number, using default 1883");
            strcpy(mqtt_port, "1883");
        }

        // Save the configuration
        saveConfig();

        if (!connected) {
            Serial.println("[WiFiConfig] WiFi connection FAILED or timed out.");
            return 0;
        } else {
            Serial.print("[WiFiConfig] Connected successfully. IP: ");
            Serial.println(WiFi.localIP());
            Serial.printf("[WiFiConfig] Base station configured - IP: %s, Port: %s\n", basestation_ip, mqtt_port);
            return 1;
        }
    }
}
