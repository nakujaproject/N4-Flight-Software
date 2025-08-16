/**
 * @file main.cpp
 * @author Flight Computer Team
 * @version N4
 * @date July 2025
 * 
 * @brief ESP32 Beacon Receiver for N4 Rocket Flight Computer
 * 
 * This receiver listens for ESP-NOW beacons from the flight computer,
 * parses telemetry data, captures RSSI, and forwards everything to
 * the base station via HTTP POST requests.
 * 
 * Data Format Expected (CSV):
 * record_number,operation_mode,state,ax,ay,az,pitch,roll,gx,gy,gz,
 * latitude,longitude,gps_altitude,gps_time,pressure,temperature,
 * rel_altitude,velocity,drogue_pin_state,main_chute_pin_state,battery_voltage
 */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Configuration
const char* WIFI_SSID = "your-wifi-ssid";
const char* WIFI_PASSWORD = "your-wifi-password";
const char* BASE_STATION_URL = "http://192.168.100.248:3001/api/telemetry";

// Beacon data structure
struct BeaconData {
    uint32_t timestamp;
    int8_t rssi;
    uint8_t mac_addr[6];
    String csv_data;
    bool is_valid;
};

// Global variables
BeaconData latest_beacon;
unsigned long last_beacon_time = 0;
unsigned long last_http_post = 0;
const unsigned long HTTP_POST_INTERVAL = 1000; // Send data every 1 second
bool wifi_connected = false;

// Function prototypes
void setupWiFi();
void setupESPNow();
void onDataReceive(const uint8_t *mac_addr, const uint8_t *data, int data_len);
void sendToBaseStation();
void parseAndValidateCSV(const String& csv_data);
bool connectToWiFi();

/**
 * @brief Initialize WiFi connection
 */
void setupWiFi() {
    WiFi.mode(WIFI_STA);
    wifi_connected = connectToWiFi();
    
    if (wifi_connected) {
        Serial.println("[+] WiFi connected successfully");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("[-] WiFi connection failed");
    }
}

/**
 * @brief Attempt to connect to WiFi
 */
bool connectToWiFi() {
    Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println();
    
    return WiFi.status() == WL_CONNECTED;
}

/**
 * @brief Initialize ESP-NOW for beacon reception
 */
void setupESPNow() {
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[-] Error initializing ESP-NOW");
        return;
    }
    
    Serial.println("[+] ESP-NOW initialized successfully");
    
    // Register callback for received data
    esp_now_register_recv_cb(onDataReceive);
    
    Serial.println("[+] ESP-NOW receiver callback registered");
}

/**
 * @brief Callback function when ESP-NOW data is received
 */
void onDataReceive(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    // Get current time and RSSI
    latest_beacon.timestamp = millis();
    last_beacon_time = latest_beacon.timestamp;
    
    // Get RSSI (signal strength)
    wifi_pkt_rx_ctrl_t *rx_ctrl = (wifi_pkt_rx_ctrl_t *)data;
    latest_beacon.rssi = rx_ctrl->rssi;
    
    // Copy MAC address
    memcpy(latest_beacon.mac_addr, mac_addr, 6);
    
    // Convert received data to string
    char received_data[data_len + 1];
    memcpy(received_data, data, data_len);
    received_data[data_len] = '\0';
    latest_beacon.csv_data = String(received_data);
    
    // Validate and parse the CSV data
    parseAndValidateCSV(latest_beacon.csv_data);
    
    // Print received beacon info
    Serial.printf("[BEACON] RSSI: %d dBm, MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  latest_beacon.rssi,
                  mac_addr[0], mac_addr[1], mac_addr[2], 
                  mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.printf("[DATA] %s\n", latest_beacon.csv_data.c_str());
}

/**
 * @brief Parse and validate CSV telemetry data
 */
void parseAndValidateCSV(const String& csv_data) {
    // Count commas to validate CSV format
    int comma_count = 0;
    for (int i = 0; i < csv_data.length(); i++) {
        if (csv_data.charAt(i) == ',') {
            comma_count++;
        }
    }
    
    // Expected format has 21 commas (22 fields)
    if (comma_count >= 20 && comma_count <= 22) {
        latest_beacon.is_valid = true;
        Serial.println("[+] Beacon data validated successfully");
    } else {
        latest_beacon.is_valid = false;
        Serial.printf("[-] Invalid beacon data - expected 21-22 fields, got %d\n", comma_count + 1);
    }
}

/**
 * @brief Send telemetry data to base station via HTTP POST
 */
void sendToBaseStation() {
    if (!wifi_connected || !latest_beacon.is_valid) {
        return;
    }
    
    HTTPClient http;
    http.begin(BASE_STATION_URL);
    http.addHeader("Content-Type", "application/json");
    
    // Create JSON payload with RSSI and parsed telemetry data
    DynamicJsonDocument doc(1024);
    
    // Parse CSV data into JSON
    String csv = latest_beacon.csv_data;
    csv.trim();
    
    // Split CSV by commas
    int field_index = 0;
    int start_pos = 0;
    String fields[25]; // Extra space for safety
    
    for (int i = 0; i <= csv.length(); i++) {
        if (i == csv.length() || csv.charAt(i) == ',') {
            if (field_index < 25) {
                fields[field_index] = csv.substring(start_pos, i);
                fields[field_index].trim();
                field_index++;
            }
            start_pos = i + 1;
        }
    }
    
    // Build JSON object with proper field names
    if (field_index >= 21) {
        doc["timestamp"] = latest_beacon.timestamp;
        doc["rssi"] = latest_beacon.rssi;
        doc["mac_address"] = String(latest_beacon.mac_addr[0], HEX) + ":" + 
                             String(latest_beacon.mac_addr[1], HEX) + ":" +
                             String(latest_beacon.mac_addr[2], HEX) + ":" +
                             String(latest_beacon.mac_addr[3], HEX) + ":" +
                             String(latest_beacon.mac_addr[4], HEX) + ":" +
                             String(latest_beacon.mac_addr[5], HEX);
        
        // Telemetry fields
        doc["record_number"] = fields[0].toInt();
        doc["operation_mode"] = fields[1].toInt();
        doc["state"] = fields[2].toInt();
        doc["acceleration"]["ax"] = fields[3].toFloat();
        doc["acceleration"]["ay"] = fields[4].toFloat(); 
        doc["acceleration"]["az"] = fields[5].toFloat();
        doc["acceleration"]["pitch"] = fields[6].toFloat();
        doc["acceleration"]["roll"] = fields[7].toFloat();
        doc["gyroscope"]["gx"] = fields[8].toFloat();
        doc["gyroscope"]["gy"] = fields[9].toFloat();
        doc["gyroscope"]["gz"] = fields[10].toFloat();
        doc["gps"]["latitude"] = fields[11].toDouble();
        doc["gps"]["longitude"] = fields[12].toDouble(); 
        doc["gps"]["altitude"] = fields[13].toFloat();
        doc["gps"]["time"] = fields[14].toInt();
        doc["altimeter"]["pressure"] = fields[15].toFloat();
        doc["altimeter"]["temperature"] = fields[16].toFloat();
        doc["altimeter"]["altitude"] = fields[17].toFloat();
        doc["altimeter"]["velocity"] = fields[18].toFloat();
        doc["pyro"]["drogue_state"] = fields[19].toInt();
        doc["pyro"]["main_state"] = fields[20].toInt();
        
        if (field_index >= 22) {
            doc["battery_voltage"] = fields[21].toFloat();
        }
        
        // Convert to string
        String json_string;
        serializeJson(doc, json_string);
        
        // Send HTTP POST
        int http_response_code = http.POST(json_string);
        
        if (http_response_code > 0) {
            Serial.printf("[HTTP] POST successful, response: %d\n", http_response_code);
            if (http_response_code == 200) {
                String response = http.getString();
                Serial.printf("[HTTP] Response: %s\n", response.c_str());
            }
        } else {
            Serial.printf("[HTTP] POST failed, error: %s\n", http.errorToString(http_response_code).c_str());
        }
    }
    
    http.end();
}

/**
 * @brief Arduino setup function
 */
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=================================");
    Serial.println("N4 Rocket Beacon Receiver v2.0");
    Serial.println("=================================");
    
    // Initialize WiFi
    setupWiFi();
    
    // Initialize ESP-NOW
    setupESPNow();
    
    Serial.println("[+] Beacon receiver ready");
    Serial.println("Listening for flight computer beacons...");
}

/**
 * @brief Arduino main loop
 */
void loop() {
    unsigned long current_time = millis();
    
    // Check WiFi connection status
    if (WiFi.status() != WL_CONNECTED) {
        if (wifi_connected) {
            Serial.println("[-] WiFi connection lost, attempting reconnection...");
            wifi_connected = false;
        }
        
        // Try to reconnect every 30 seconds
        static unsigned long last_wifi_attempt = 0;
        if (current_time - last_wifi_attempt > 30000) {
            wifi_connected = connectToWiFi();
            last_wifi_attempt = current_time;
            
            if (wifi_connected) {
                Serial.println("[+] WiFi reconnected successfully");
            }
        }
    } else {
        wifi_connected = true;
    }
    
    // Send data to base station at regular intervals if we have valid data
    if (current_time - last_http_post > HTTP_POST_INTERVAL && 
        latest_beacon.is_valid && 
        (current_time - last_beacon_time < 10000)) { // Only send if beacon is recent (within 10 seconds)
        
        sendToBaseStation();
        last_http_post = current_time;
    }
    
    // Print status every 10 seconds
    static unsigned long last_status = 0;
    if (current_time - last_status > 10000) {
        Serial.printf("[STATUS] WiFi: %s, Last beacon: %lu ms ago, RSSI: %d dBm\n",
                      wifi_connected ? "Connected" : "Disconnected",
                      current_time - last_beacon_time,
                      latest_beacon.rssi);
        last_status = current_time;
    }
    
    delay(100);
}
