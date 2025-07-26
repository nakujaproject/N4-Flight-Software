/**
 * @file communication_manager.cpp
 * @brief Smart communication manager implementation
 * @author AI Assistant
 * @date July 2025
 */

#include "communication_manager.h"
#include "system_logger.h"
#include "wifi-config.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <FS.h>
#include <SPIFFS.h>
#include "esp_wifi.h"
#include "esp_now.h"
#include "espnow_beacon_transmitter.h"
#include "defs.h"
#include <WiFi.h>
#include <esp_now.h>

// External references from main.cpp
extern SystemLogger SYSTEM_LOGGER;
extern const char* system_log_file;
extern ESPNowBeaconTransmitter transmitter;
#include "defs.h"



void reconfigureForMQTTMode() {
    // 1. Disconnect WiFi if connected (optional, but safe)
    if (WiFi.isConnected()) {
        WiFi.disconnect(true);
        delay(100);
    }
    // 2. Set WiFi to STA mode for MQTT
    WiFi.mode(WIFI_STA);
    delay(100);
    // 3. Deinit ESP-NOW if active
    esp_now_deinit();
    delay(50);
    Serial.println("[RECONFIG] WiFi set to STA, ESP-NOW deinitialized for MQTT mode");
    // 4. Reconnect to WiFi
    wifi_config.WifiConnect(false, ROCKET_MAC);
    delay(100);
    // 5. Re-initialize MQTT
    MQTTInit(wifi_config.getBaseStationIP(), wifi_config.getMQTTPort());
    MQTT_Reconnect();
}



void reconfigureForBeaconMode() {
    // 1. Disconnect WiFi if connected
    if (WiFi.isConnected()) {
        WiFi.disconnect(true);
        delay(100);
    }
    // 2. Set WiFi to AP+STA mode (required for esp_wifi_80211_tx)
    WiFi.mode(WIFI_AP_STA);
    delay(100);
    // 3. Set AP MAC address and channel (must match beacon requirements)
    esp_wifi_set_mac(WIFI_IF_AP, ROCKET_MAC);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    delay(100);
    // 4. Deinit and re-init ESP-NOW (clean state)
    esp_now_deinit();
    delay(50);
    if (esp_now_init() == ESP_OK) {
        Serial.println("[RECONFIG] ESP-NOW re-initialized for beacon mode");
    } else {
        Serial.println("[RECONFIG] ESP-NOW re-init failed!");
    }
    // 5. Re-initialize beacon transmitter (sets up peer, callback, etc.)
    transmitter.begin();
}



// Global communication manager instance
CommunicationManager comm_manager;

void CommunicationManager::init() {
    // Initialize communication status
    comm_status.last_mqtt_success = 0;
    comm_status.last_beacon_success = 0;
    comm_status.mqtt_failure_count = 0;
    comm_status.beacon_failure_count = 0;
    comm_status.mqtt_connection_stable = false;
    comm_status.beacon_connection_stable = false;
    comm_status.current_mode = "STARTING";
    comm_status.last_command_source = "SYSTEM";
    
    // Set default communication modes
    use_mqtt_mode = true;
    use_beacon_mode = false;
    auto_fallback_enabled = true;
    communication_mode_locked = false;
    
    last_mode_check = millis();
    last_status_report = millis();
    
    Serial.println("[COMM MANAGER] Initialized - MQTT mode enabled, auto-fallback active");
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "Communication Manager initialized\r\n");
}

void CommunicationManager::handleModeCommand(String command, String source) {
    if (communication_mode_locked && command != "CMD_RESET" && command != "CMD_ARM" && command != "CMD_DISARM") {
        Serial.printf("[COMM MANAGER] Mode locked - command %s from %s ignored\n", command.c_str(), source.c_str());
        return;
    }
    
    comm_status.last_command_source = source.c_str();
    
    if (command == "CMD_MQTT_MODE") {
        setMQTTMode(source);
    } else if (command == "CMD_BEACON_MODE") {
        setBeaconMode(source);
    } else if (command == "CMD_DUAL_MODE") {
        setDualMode(source);
    } else if (command == "CMD_AUTO_FALLBACK_ON") {
        auto_fallback_enabled = true;
        Serial.printf("[COMM MANAGER] Auto-fallback ENABLED by %s\n", source.c_str());
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "Auto-fallback enabled\r\n");
    } else if (command == "CMD_AUTO_FALLBACK_OFF") {
        auto_fallback_enabled = false;
        Serial.printf("[COMM MANAGER] Auto-fallback DISABLED by %s\n", source.c_str());
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "Auto-fallback disabled\r\n");
    } else if (command == "CMD_GET_MODE") {
        reportCurrentMode();
    } else if (command == "CMD_ARM") {
        // Handle arming
        is_system_armed = true;
        Serial.printf("[COMM MANAGER] ARMED by %s\n", source.c_str());
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "System armed\r\n");
    } else if (command == "CMD_DISARM") {
        // Handle disarming
        is_system_armed = false;
        communication_mode_locked = false; // Unlock modes when disarmed
        Serial.printf("[COMM MANAGER] DISARMED by %s\n", source.c_str());
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "System disarmed\r\n");
    } else if (command == "CMD_RESET") {
        // Reset communication system
        init();
        Serial.printf("[COMM MANAGER] RESET by %s\n", source.c_str());
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "Communication system reset\r\n");
    } else {
        Serial.printf("[COMM MANAGER] Unknown command: %s from %s\n", command.c_str(), source.c_str());
    }
}

void CommunicationManager::setMQTTMode(String source) {
    bool was_mqtt = use_mqtt_mode && !use_beacon_mode;
    use_mqtt_mode = true;
    use_beacon_mode = false;
    comm_status.current_mode = "MQTT_ONLY";
    Serial.printf("[COMM MANAGER] Switched to MQTT-only mode by %s\n", source.c_str());
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "Switched to MQTT-only mode\r\n");
    // Only reconfigure if we weren't already in MQTT-only mode
    if (!was_mqtt) {
        reconfigureForMQTTMode();
    }
}

void CommunicationManager::setBeaconMode(String source) {
    bool was_beacon = use_beacon_mode && !use_mqtt_mode;
    use_mqtt_mode = false;
    use_beacon_mode = true;
    comm_status.current_mode = "BEACON_ONLY";
    Serial.printf("[COMM MANAGER] Switched to Beacon-only mode by %s\n", source.c_str());
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "Switched to Beacon-only mode\r\n");
    // Only reconfigure if we weren't already in beacon-only mode
    if (!was_beacon) {
        reconfigureForBeaconMode();
    }
}

void CommunicationManager::setDualMode(String source) {
    use_mqtt_mode = true;
    use_beacon_mode = true;
    comm_status.current_mode = "DUAL_MODE";
    
    Serial.printf("[COMM MANAGER] Switched to Dual mode (MQTT + Beacon) by %s\n", source.c_str());
    SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "Switched to dual communication mode\r\n");
}

void CommunicationManager::reportCurrentMode() {
    String mode = getCurrentMode();
    String status = "";
    
    if (communication_mode_locked) {
        status += " [LOCKED]";
    }
    if (auto_fallback_enabled) {
        status += " [AUTO-FALLBACK]";
    }
    if (is_system_armed) {
        status += " [ARMED]";
    }
    
    uint32_t now = millis();
    uint32_t mqtt_age = (comm_status.last_mqtt_success > 0) ? 
                        (now - comm_status.last_mqtt_success) : 0;
    uint32_t beacon_age = (comm_status.last_beacon_success > 0) ? 
                          (now - comm_status.last_beacon_success) : 0;
    
    Serial.printf("[COMM STATUS] Mode: %s%s | MQTT: %s (%lums ago, %d fails) | Beacon: %s (%lums ago, %d fails)\n",
                  mode.c_str(), status.c_str(),
                  use_mqtt_mode ? "ON" : "OFF", mqtt_age, comm_status.mqtt_failure_count,
                  use_beacon_mode ? "ON" : "OFF", beacon_age, comm_status.beacon_failure_count);
}

void CommunicationManager::updateTransmissionStatus(bool mqtt_success, bool beacon_success) {
    uint32_t now = millis();
    
    if (mqtt_success && use_mqtt_mode) {
        comm_status.last_mqtt_success = now;
        comm_status.mqtt_failure_count = 0;
    } else if (use_mqtt_mode) {
        comm_status.mqtt_failure_count++;
    }
    
    if (beacon_success && use_beacon_mode) {
        comm_status.last_beacon_success = now;
        comm_status.beacon_failure_count = 0;
    } else if (use_beacon_mode) {
        comm_status.beacon_failure_count++;
    }
    
    // Update connection stability
    bool mqtt_stable = !use_mqtt_mode || 
                       (comm_status.mqtt_failure_count < MQTT_RETRY_ATTEMPTS);
    bool beacon_stable = !use_beacon_mode || 
                         (comm_status.beacon_failure_count < MQTT_RETRY_ATTEMPTS);
    
    comm_status.mqtt_connection_stable = mqtt_stable;
    comm_status.beacon_connection_stable = beacon_stable;
}

void CommunicationManager::checkAutoFallback() {
    if (!auto_fallback_enabled || communication_mode_locked) {
        return;
    }
    
    uint32_t now = millis();
    
    // Check if MQTT has been failing for too long
    if (use_mqtt_mode && !use_beacon_mode) {
        bool mqtt_failed = (comm_status.mqtt_failure_count >= MQTT_RETRY_ATTEMPTS) ||
                          (comm_status.last_mqtt_success > 0 && 
                           (now - comm_status.last_mqtt_success) > MQTT_FAILURE_TIMEOUT);
        
        if (mqtt_failed) {
            Serial.println("[COMM MANAGER] MQTT failure detected - auto-switching to Beacon mode");
            setBeaconMode("AUTO_FALLBACK");
            SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::WARNING, system_log_file, "Auto-fallback: MQTT failed, switched to Beacon\r\n");
        }
    }
    
    // Check if we should fall back to MQTT when it's recovered
    else if (!use_mqtt_mode && use_beacon_mode) {
        // Only try to switch back if we've been in beacon mode for a while
        if (comm_status.current_mode == "BEACON_ONLY" &&
            comm_status.last_mqtt_success > 0 &&
            (now - comm_status.last_mqtt_success) > AUTO_FALLBACK_HYSTERESIS) {
            
            // Try to reconnect WiFi if it's disconnected
            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("[COMM MANAGER] Attempting WiFi reconnection for fallback");
                // WiFi reconnection logic would go here
            } else {
                Serial.println("[COMM MANAGER] WiFi recovered - auto-switching back to MQTT mode");
                setMQTTMode("AUTO_RECOVERY");
                SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "Auto-recovery: Switched back to MQTT mode\r\n");
            }
        }
    }
}

void CommunicationManager::lockCommunicationMode(bool lock) {
    communication_mode_locked = lock;
    if (lock) {
        Serial.println("[COMM MANAGER] Communication mode LOCKED (flight active)");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "Communication mode locked for flight\r\n");
    } else {
        Serial.println("[COMM MANAGER] Communication mode UNLOCKED");
        SYSTEM_LOGGER.logToFile(SPIFFS, LOG_MODE::APPEND, "FC1", LOG_LEVEL::INFO, system_log_file, "Communication mode unlocked\r\n");
    }
}

void CommunicationManager::periodicStatusReport() {
    uint32_t now = millis();
    
    // Report status every 30 seconds
    if (now - last_status_report > 30000) {
        reportCurrentMode();
        last_status_report = now;
    }
}

void CommunicationManager::update() {
    uint32_t now = millis();
    
    // Check auto-fallback every 5 seconds
    if (now - last_mode_check > 5000) {
        checkAutoFallback();
        last_mode_check = now;
    }
    
    // Periodic status reporting
    periodicStatusReport();
}

String CommunicationManager::getCurrentMode() {
    if (use_mqtt_mode && use_beacon_mode) {
        return "DUAL_MODE";
    } else if (use_mqtt_mode) {
        return "MQTT_ONLY";
    } else if (use_beacon_mode) {
        return "BEACON_ONLY";
    } else {
        return "NO_COMMS";
    }
}

bool CommunicationManager::isMQTTActive() {
    return use_mqtt_mode;
}

bool CommunicationManager::isBeaconActive() {
    return use_beacon_mode;
}

bool CommunicationManager::isModeLocked() {
    return communication_mode_locked;
}

// Global command processing functions
void processCommand(String command, String source) {
    command.trim();
    comm_manager.handleModeCommand(command, source);
}

void handleIncomingCommands() {
    // Check for serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.length() > 0) {
            processCommand(command, "SERIAL");
        }
    }
}
