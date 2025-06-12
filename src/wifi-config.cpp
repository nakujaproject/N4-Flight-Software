
/*!****************************************************************************
 * @file configureWifi.cpp
 * @brief This file creates methods to allow dynamic configuration of the WIFI credentials
 * This allows the launch crew to avoid hardcoding the WIFI SSID and password inside the
 * flight software
 *
 * The crew can just create the Wifi with any SSID and/or password, and this interface will allow them
 * to connect to that wifi without touching the flight software
 *
 * This effectively adds an improved layer of abstraction
 *******************************************************************************/

#include "wifi-config.h"

/*!****************************************************************************
 * @brief allow connection to WiFi
 *
 */
// uint8_t WIFIConfig::WifiConnect() {
//     WiFi.mode(WIFI_STA); // start in station mode
//     WiFiManager wm;

//     // wipe stored credentials
//     // wm.resetSettings();

//     bool connection_result = 0;

//     // generate open access point for devices to connect to
//     connection_result = wm.autoConnect("flight-computer-1"); // change with unique ID of the respective flight computer

//     if(!connection_result) {
//         return 0;
//         // TODO: log to system logger
//     } else {
//         // Wifi is connected here
//         // we will check this in main.cpp to see if we have been connected successfully
//         return 1;

//     }

// }




uint8_t WIFIConfig::WifiConnect() {
    WiFi.mode(WIFI_STA);  // Start in station mode
    WiFiManager wm;

    // Uncomment only for debugging or clearing corrupted WiFi
    // wm.resetSettings(); 

    Serial.println("[WiFiConfig] Starting WiFiManager...");

    wm.setConfigPortalTimeout(180);        // 3-minute timeout for AP
    wm.setBreakAfterConfig(true);          // Exit after config
    // wm.setAPCallback([](WiFiManager* w) { Serial.println("[WiFiConfig] AP Mode Started"); }); // Optional

    bool connected = wm.autoConnect("flight-computer-1");  // Start config AP

    if (!connected) {
        Serial.println("[WiFiConfig] WiFi connection FAILED or timed out.");
        return 0;
    } else {
        Serial.print("[WiFiConfig] Connected successfully. IP: ");
        Serial.println(WiFi.localIP());
        return 1;
    }
}