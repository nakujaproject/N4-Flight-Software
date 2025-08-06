
/**
 * @file communication_manager.h
 * @brief Smart communication manager for dynamic mode switching between MQTT and Beacon modes
 * @author AI Assistant
 * @date July 2025
 */

#ifndef COMMUNICATION_MANAGER_H
#define COMMUNICATION_MANAGER_H

#include <Arduino.h>
#include "defs.h"


// C++ linkage only (no extern "C")
void reconfigureForBeaconMode();
void reconfigureForMQTTMode();

class CommunicationManager {
private:
    uint32_t last_mode_check = 0;
    uint32_t last_status_report = 0;
    
public:
    /**
     * @brief Initialize the communication manager
     */
    void init();
    
    /**
     * @brief Handle incoming mode commands
     * @param command The command string to process
     * @param source The source of the command (MQTT, ESP_NOW, SERIAL, etc.)
     */
    void handleModeCommand(String command, String source = "UNKNOWN");
    
    /**
     * @brief Set communication to MQTT mode only
     * @param source Source of the command
     */
    void setMQTTMode(String source);
    
    /**
     * @brief Set communication to Beacon mode only
     * @param source Source of the command
     */
    void setBeaconMode(String source);
    
    /**
     * @brief Report current communication mode and status
     */
    void reportCurrentMode();
    
    /**
     * @brief Update transmission status for both modes
     * @param mqtt_success True if MQTT transmission was successful
     * @param beacon_success True if beacon transmission was successful
     */
    void updateTransmissionStatus(bool mqtt_success, bool beacon_success);
    
    /**
     * @brief Check if automatic fallback should be triggered
     */
    void checkAutoFallback();
    
    /**
     * @brief Lock or unlock communication mode changes
     * @param lock True to lock mode changes, false to unlock
     */
    void lockCommunicationMode(bool lock);
    
    /**
     * @brief Send periodic status reports
     */
    void periodicStatusReport();
    
    /**
     * @brief Main update function - call this regularly in main loop
     */
    void update();
    
    /**
     * @brief Get current communication mode as string
     * @return Current mode string
     */
    String getCurrentMode();
    
    /**
     * @brief Check if MQTT mode is currently active
     * @return True if MQTT mode is active
     */
    bool isMQTTActive();
    
    /**
     * @brief Check if Beacon mode is currently active
     * @return True if Beacon mode is active
     */
    bool isBeaconActive();
    
    /**
     * @brief Check if communication mode is locked
     * @return True if locked
     */
    bool isModeLocked();
};

// Global communication manager instance
extern CommunicationManager comm_manager;

// Function prototypes for command processing
void processCommand(String command, String source);
void handleIncomingCommands();

#endif // COMMUNICATION_MANAGER_H
