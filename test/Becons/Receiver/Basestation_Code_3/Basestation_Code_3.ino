#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <ArduinoJson.h>

// Configuration
uint8_t rocket_mac[] = {0x08, 0xd1, 0xf9, 0x15, 0x9c, 0x40}; // Rocket MAC
uint8_t my_mac[] = {0x10, 0x06, 0x1c, 0xa6, 0x18, 0x20}; // Base MAC

bool rocketArmed = false;

// Connection status tracking
const uint32_t CONNECTION_TIMEOUT = 15000; // 15 seconds
bool hasEverConnected = false;
bool currentlyConnected = false;

// Updated telemetry data structure to match flight computer's 22-field CSV
struct TelemetryData {
  uint32_t record_number;        // 0
  uint8_t operation_mode;        // 1
  uint8_t state;                 // 2
  float ax, ay, az;              // 3-5: acceleration
  float pitch, roll;             // 6-7: attitude
  float gx, gy, gz;              // 8-10: gyroscope
  float latitude, longitude;     // 11-12: GPS coordinates
  float gps_altitude;            // 13: GPS altitude
  uint32_t gps_time;             // 14: GPS time
  float pressure;                // 15: barometric pressure
  float temperature;             // 16: temperature
  float altitude_agl;            // 17: relative altitude (AGL)
  float velocity;                // 18: velocity
  uint8_t drogue_pin_state;      // 19: drogue pin state
  uint8_t main_chute_pin_state;  // 20: main chute pin state
  float battery_voltage;         // 21: battery voltage
  // NO RSSI field - base station measures beacon RSSI separately
};

TelemetryData telemetry;
int8_t beacon_rssi = 0;  // Measured beacon RSSI (separate from CSV data)
uint32_t packetsReceived = 0;
uint32_t lastPacketTime = 0;
bool dataReceived = false;

// Command handling
String lastCommand = "";
bool commandPending = false;

// Function to parse 22-field CSV string (matches flight computer beacon output)
bool parseCSV(const char* csv, TelemetryData& data) {
  return sscanf(csv, "%lu,%hhu,%hhu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%lu,%f,%f,%f,%f,%hhu,%hhu,%f",
                &data.record_number,      // 0
                &data.operation_mode,     // 1
                &data.state,              // 2
                &data.ax, &data.ay, &data.az,           // 3-5
                &data.pitch, &data.roll,                // 6-7
                &data.gx, &data.gy, &data.gz,           // 8-10
                &data.latitude, &data.longitude,        // 11-12
                &data.gps_altitude,                     // 13
                &data.gps_time,                         // 14
                &data.pressure,                         // 15
                &data.temperature,                      // 16
                &data.altitude_agl,                     // 17
                &data.velocity,                         // 18
                &data.drogue_pin_state,                 // 19
                &data.main_chute_pin_state,             // 20
                &data.battery_voltage) == 22;           // 21 (NO RSSI)
}

// Update connection status
void updateConnectionStatus() {
  uint32_t packetAge = millis() - lastPacketTime;
  
  if (dataReceived) {
    hasEverConnected = true;
    currentlyConnected = (packetAge <= CONNECTION_TIMEOUT);
  } else {
    currentlyConnected = false;
  }
}

// Convert telemetry to JSON format exactly as server.py expects
void sendTelemetryJSON() {
  if (!dataReceived) return;
  
  updateConnectionStatus();
  
  // Create JSON document with the structure expected by your React app
  DynamicJsonDocument doc(1024);
  
  // Root level fields
  doc["record_number"] = telemetry.record_number;
  doc["operation_mode"] = telemetry.operation_mode;
  doc["state"] = telemetry.state;
  doc["battery_voltage"] = telemetry.battery_voltage; // Now from flight computer
  doc["wifi_rssi"] = beacon_rssi; // Beacon RSSI measured by base station
  
  // Acceleration data structure
  JsonObject acc_data = doc.createNestedObject("acc_data");
  acc_data["ax"] = telemetry.ax;
  acc_data["ay"] = telemetry.ay;
  acc_data["az"] = telemetry.az;
  acc_data["pitch"] = telemetry.pitch;
  acc_data["roll"] = telemetry.roll;
  
  // Gyro data structure
  JsonObject gyro_data = doc.createNestedObject("gyro_data");
  gyro_data["gx"] = telemetry.gx;
  gyro_data["gy"] = telemetry.gy;
  gyro_data["gz"] = telemetry.gz;
  
  // GPS data structure
  JsonObject gps_data = doc.createNestedObject("gps_data");
  gps_data["latitude"] = telemetry.latitude;
  gps_data["longitude"] = telemetry.longitude;
  gps_data["gps_altitude"] = telemetry.gps_altitude;
  gps_data["time"] = telemetry.gps_time; // Now included from flight computer
  
  // Altimeter data structure
  JsonObject alt_data = doc.createNestedObject("alt_data");
  alt_data["pressure"] = telemetry.pressure;
  alt_data["temperature"] = telemetry.temperature;
  alt_data["AGL"] = telemetry.altitude_agl;
  alt_data["velocity"] = telemetry.velocity; // Now from flight computer
  
  // Chute state structure
  JsonObject chute_state = doc.createNestedObject("chute_state");
  chute_state["pyro1_state"] = telemetry.drogue_pin_state;
  chute_state["pyro2_state"] = telemetry.main_chute_pin_state;
  
  // Enhanced connection status
  JsonObject conn_status = doc.createNestedObject("connection_status");
  conn_status["connected"] = currentlyConnected;
  conn_status["has_ever_connected"] = hasEverConnected;
  conn_status["packet_age_ms"] = millis() - lastPacketTime;
  conn_status["timeout_exceeded"] = (millis() - lastPacketTime) > CONNECTION_TIMEOUT;
  conn_status["rssi"] = beacon_rssi; // Beacon RSSI from base station
  
  // Communication mode
  doc["communication_mode"] = "Beacon";
  
  // Metadata
  doc["timestamp"] = millis();
  doc["packets_received"] = packetsReceived;
  
  // Serialize and send to Python script via Serial
  String jsonString;
  serializeJson(doc, jsonString);
  Serial.println(jsonString);
}

void handleBeacon(const wifi_promiscuous_pkt_t* pkt) {
  const uint8_t* payload = pkt->payload;
  int len = pkt->rx_ctrl.sig_len;

  // Capture beacon RSSI immediately
  beacon_rssi = pkt->rx_ctrl.rssi;

  // Basic validation
  if (len < 60 || payload[0] != 0x80) return;
  if (memcmp(&payload[10], rocket_mac, 6) != 0) return;

  // Find telemetry data (0xDD vendor tag)
  for (int i = 36; i < len - 2; i++) {
    if (payload[i] == 0xDD) {
      uint8_t data_len = payload[i + 1];
      
      // Create null-terminated string
      char csv_data[512]; // Increased buffer size for 22 fields
      memcpy(csv_data, &payload[i + 2], data_len);
      csv_data[data_len] = '\0';
      
      // Parse 22-field CSV data (NO RSSI field from flight computer)
      if (parseCSV(csv_data, telemetry)) {
        // Update connection tracking
        bool wasConnected = currentlyConnected;
        rocketArmed = (telemetry.operation_mode == 1);
        packetsReceived++;
        lastPacketTime = millis();
        dataReceived = true;
        
        updateConnectionStatus();
        
        // Send JSON telemetry to Python script
        sendTelemetryJSON();
        
        // Log connection state changes
        if (!wasConnected && currentlyConnected) {
          sendLogMessage("INFO", "🟢 CONNECTED - Rocket beacon link established", "BaseStation");
        }
        
        // Log operation mode changes
        static uint8_t lastOperationMode = 255;
        if (lastOperationMode != telemetry.operation_mode) {
          const char* mode = (telemetry.operation_mode == 1) ? "ARMED" : "SAFE";
          sendLogMessage("INFO", ("Rocket operation mode: " + String(mode)).c_str(), "BaseStation");
          lastOperationMode = telemetry.operation_mode;
        }
      } else {
        sendLogMessage("ERROR", "Failed to parse 22-field CSV telemetry", "BaseStation");
      }
      break;
    }
  }
}

// Send log messages in format expected by React app
void sendLogMessage(const char* level, const char* message, const char* source) {
  DynamicJsonDocument logDoc(256);
  logDoc["level"] = level;
  logDoc["message"] = message;
  logDoc["source"] = source;
  logDoc["timestamp"] = millis();
  
  String logString;
  serializeJson(logDoc, logString);
  
  Serial.print("LOG:");
  Serial.println(logString);
}

void promiscuousRx(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type == WIFI_PKT_MGMT) {
    handleBeacon((wifi_promiscuous_pkt_t*)buf);
  }
}

// Handle commands from Python script (ARM/DISARM/RESET)
void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "ARM" || command == "DISARM" || command == "RESET") {
      lastCommand = command;
      commandPending = true;
      
      sendLogMessage("INFO", ("Command received from base station: " + command).c_str(), "BaseStation");
    }
  }
}

// Send commands to rocket via ESP-NOW
void sendCommandToRocket() {
  if (!commandPending) return;
  
  esp_err_t result;
  if (lastCommand == "ARM") {
    result = esp_now_send(rocket_mac, (uint8_t*)"ARM", 3);
  } else if (lastCommand == "DISARM") {
    result = esp_now_send(rocket_mac, (uint8_t*)"DISARM", 6);
  } else if (lastCommand == "RESET") {
    result = esp_now_send(rocket_mac, (uint8_t*)"RESET", 5);
  }
  
  if (result == ESP_OK) {
    sendLogMessage("INFO", ("Command sent to rocket: " + lastCommand).c_str(), "BaseStation");
  } else {
    sendLogMessage("ERROR", ("Failed to send command: " + lastCommand).c_str(), "BaseStation");
  }
  
  commandPending = false;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Send startup message
  sendLogMessage("INFO", "🚀 N4 Base Station - 22-Field Beacon Mode", "BaseStation");

  // Configure WiFi
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, my_mac);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    sendLogMessage("ERROR", "ESP-NOW initialization failed", "BaseStation");
    ESP.restart();
  }

  // Register peer (rocket)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, rocket_mac, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    sendLogMessage("ERROR", "Failed to add rocket as peer", "BaseStation");
  } else {
    sendLogMessage("INFO", "Rocket peer registered successfully", "BaseStation");
  }

  // Setup promiscuous mode for beacon reception
  wifi_promiscuous_filter_t filter = {
    .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT
  };
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_filter(&filter);
  esp_wifi_set_promiscuous_rx_cb(promiscuousRx);

  sendLogMessage("INFO", "📡 22-field beacon parsing enabled", "BaseStation");
  sendLogMessage("INFO", "🔧 Beacon RSSI measurement active", "BaseStation");
  sendLogMessage("INFO", "🎯 Velocity & battery data from flight computer", "BaseStation");
  sendLogMessage("INFO", "Use ARM/DISARM/RESET commands to control rocket", "BaseStation");
}

void loop() {
  static uint32_t lastStatusTime = 0;
  static uint32_t lastHeartbeat = 0;
  
  // Update connection status
  updateConnectionStatus();
  
  // Handle incoming serial commands from Python script
  handleSerialCommands();
  
  // Send pending commands to rocket
  sendCommandToRocket();
  
  // Send enhanced heartbeat/status every 10 seconds
  if (millis() - lastHeartbeat > 10000) {
    DynamicJsonDocument statusDoc(512);
    statusDoc["type"] = "status";
    statusDoc["armed"] = rocketArmed;
    statusDoc["packets_received"] = packetsReceived;
    statusDoc["uptime"] = millis();
    
    // Enhanced connection info
    JsonObject conn_info = statusDoc.createNestedObject("connection");
    conn_info["currently_connected"] = currentlyConnected;
    conn_info["has_ever_connected"] = hasEverConnected;
    conn_info["last_packet_age"] = dataReceived ? (millis() - lastPacketTime) : 0;
    conn_info["timeout_exceeded"] = dataReceived && (millis() - lastPacketTime > CONNECTION_TIMEOUT);
    
    // Add telemetry info if available
    if (dataReceived) {
      JsonObject telemetry_info = statusDoc.createNestedObject("telemetry");
      telemetry_info["record_number"] = telemetry.record_number;
      telemetry_info["communication_mode"] = "Beacon";
      telemetry_info["beacon_rssi"] = beacon_rssi; // RSSI from base station
      telemetry_info["battery_voltage"] = telemetry.battery_voltage;
      telemetry_info["velocity"] = telemetry.velocity;
      telemetry_info["altitude_agl"] = telemetry.altitude_agl;
      telemetry_info["operation_mode"] = telemetry.operation_mode;
      telemetry_info["state"] = telemetry.state;
    }
    
    statusDoc["waiting_for_commands"] = !commandPending;
    statusDoc["monitoring_active"] = true;
    statusDoc["beacon_mode_only"] = true;
    statusDoc["csv_fields_expected"] = 22;
    
    String statusString;
    serializeJson(statusDoc, statusString);
    Serial.print("STATUS:");
    Serial.println(statusString);
    
    lastHeartbeat = millis();
  }
  
  // Enhanced connection monitoring messages
  if (dataReceived) {
    uint32_t packetAge = millis() - lastPacketTime;
    
    // Only warn about timeout once per timeout period
    static bool timeoutWarningGiven = false;
    
    if (packetAge > CONNECTION_TIMEOUT) {
      if (!timeoutWarningGiven) {
        sendLogMessage("WARNING", 
          ("🟡 Beacon aged: " + String(packetAge/1000) + "s | RSSI: " + String(beacon_rssi) + "dBm").c_str(), 
          "BaseStation");
        timeoutWarningGiven = true;
      }
    } else {
      timeoutWarningGiven = false; // Reset warning for next timeout
    }
  }
  
  // Periodic waiting message for new connections
  if (!hasEverConnected && millis() - lastStatusTime > 30000) {
    sendLogMessage("INFO", "📡 Waiting for 22-field beacon transmissions...", "BaseStation");
    sendLogMessage("INFO", "Send ARM command to establish communication", "BaseStation");
    lastStatusTime = millis();
  }
  
  delay(10);
}