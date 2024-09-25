
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
WIFIConfig::WifiConnect() {
    WiFi.mode(WIFI_STA); // start in station mode
    WiFiManager wm;
    bool connection_result = 0;

    // generate password procteted acess point
    connection_result = wm.autoConnect("flight-computer");

    if(!connection_result) {
        return 0;
        // TODO: log to system logger
    } else {
        // Wifi is connected here
        // we will check this in main.cpp to see if we have been connected successfully
        return 1;
    }
}
