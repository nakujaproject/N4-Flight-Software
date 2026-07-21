#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <ArduinoJson.h>

// Configuration
uint8_t rocket_mac[] = {0x08, 0xd1, 0xf9, 0x15, 0x9c, 0x40}; // Rocket MAC
uint8_t my_mac[] = {0x10, 0x06, 0x1c, 0xa6, 0x18, 0x20}; // Base MAC

//MAC: 10:06:1c:a6:18:20

bool rocketArmed = false;

// Connection status tracking
const uint32_t CONNECTION_TIMEOUT = 15000; // 15 seconds
bool hasEverConnected = false;
bool currentlyConnected = false;

// Enhanced telemetry data structure for 23-field format
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
  int32_t wifi_rssi;             // 22: WiFi RSSI (or beacon RSSI for beacon mode)
};

TelemetryData telemetry;
uint32_t packetsReceived = 0;
uint32_t lastPacketTime = 0;
bool dataReceived = false;

// Command handling
String lastCommand = "";
bool commandPending = false;

// Function to parse 23-field CSV string into telemetry struct
bool parseCSV(const char* csv, TelemetryData& data) {
  int parsed = sscanf(csv, "%lu,%hhu,%hhu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%lu,%f,%f,%f,%f,%hhu,%hhu,%f,%ld",
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
                &data.battery_voltage,                  // 21
                &data.wifi_rssi);                       // 22

  Serial.print("CSV Parse - Original RSSI field: ");
  Serial.println(data.wifi_rssi);
  
  return parsed == 23; // Ensure all 23 fields were parsed successfully
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

// Convert telemetry to JSON format matching your React app structure
void sendTelemetryJSON() {
  if (!dataReceived) return;
  
  updateConnectionStatus();
  
  // Debug RSSI before JSON creation
  Serial.print("JSON Creation - Current RSSI: ");
  Serial.println(telemetry.wifi_rssi);
  
  // Create JSON document with the structure expected by your React app
  DynamicJsonDocument doc(1024);
  
  // Root level fields
  doc["record_number"] = telemetry.record_number;
  doc["operation_mode"] = telemetry.operation_mode;
  doc["state"] = telemetry.state;
  doc["battery_voltage"] = telemetry.battery_voltage;
  doc["wifi_rssi"] = telemetry.wifi_rssi; // This now contains actual beacon RSSI
  
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
  gps_data["time"] = telemetry.gps_time;
  
  // Altimeter data structure
  JsonObject alt_data = doc.createNestedObject("alt_data");
  alt_data["pressure"] = telemetry.pressure;
  alt_data["temperature"] = telemetry.temperature;
  alt_data["AGL"] = telemetry.altitude_agl;
  alt_data["velocity"] = telemetry.velocity;
  
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
  conn_status["rssi"] = telemetry.wifi_rssi; // Beacon RSSI
  
  // Communication mode detection based on RSSI (beacon mode always has RSSI now)
  String comm_mode = "Beacon"; // Always beacon mode for this base station
  doc["communication_mode"] = comm_mode;
  
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

  // 🔥 CAPTURE ACTUAL BEACON RSSI FROM PACKET
  int8_t beacon_rssi = pkt->rx_ctrl.rssi;
  
  // Debug beacon reception
  static uint32_t lastBeaconDebug = 0;
  if (millis() - lastBeaconDebug > 3000) {
    Serial.print("Beacon received - RSSI: ");
    Serial.print(beacon_rssi);
    Serial.print("dBm, Length: ");
    Serial.println(len);
    lastBeaconDebug = millis();
  }

  // Basic validation
  if (len < 60 || payload[0] != 0x80) return;
  if (memcmp(&payload[10], rocket_mac, 6) != 0) return;

  // Find telemetry data (0xDD vendor tag)
  for (int i = 36; i < len - 2; i++) {
    if (payload[i] == 0xDD) {
      uint8_t data_len = payload[i + 1];
      
      // Create null-terminated string
      char csv_data[512]; // Increased buffer size for 23-field format
      memcpy(csv_data, &payload[i + 2], data_len);
      csv_data[data_len] = '\0';
      
      // Parse 23-field CSV data
      if (parseCSV(csv_data, telemetry)) {
        // 🔥 OVERRIDE WITH ACTUAL BEACON RSSI
        int32_t original_rssi = telemetry.wifi_rssi;
        telemetry.wifi_rssi = beacon_rssi; // Use measured beacon RSSI instead of 0
        
        // Debug the override
        if (millis() % 1000 < 50) { // Print occasionally to avoid spam
          Serial.print("RSSI Override: ");
          Serial.print(original_rssi);
          Serial.print(" -> ");
          Serial.println(beacon_rssi);
        }
        
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
          sendLogMessage("INFO", "🟢 CONNECTED - Rocket telemetry link established", "BaseStation");
        } else if (wasConnected && !currentlyConnected) {
          sendLogMessage("WARNING", "🟡 CONNECTION AGED - Still monitoring", "BaseStation");
        }
        
        // Log operation mode changes
        static uint8_t lastOperationMode = 255;
        if (lastOperationMode != telemetry.operation_mode) {
          const char* mode = (telemetry.operation_mode == 1) ? "ARMED" : "SAFE";
          sendLogMessage("INFO", ("Rocket operation mode: " + String(mode)).c_str(), "BaseStation");
          lastOperationMode = telemetry.operation_mode;
        }
        
        // Enhanced logging with beacon RSSI
        static uint32_t lastDetailedLog = 0;
        if (millis() - lastDetailedLog > 5000) { // Log details every 5 seconds
          sendLogMessage("INFO", 
            ("Record #" + String(telemetry.record_number) + 
             " | Beacon Mode | RSSI: " + String(beacon_rssi) + "dBm" +
             " | Velocity: " + String(telemetry.velocity, 1) + "m/s" +
             " | Alt: " + String(telemetry.altitude_agl, 1) + "m").c_str(), 
            "BaseStation");
          lastDetailedLog = millis();
        }
        
        // Debug RSSI override
        static uint32_t lastRSSIDebug = 0;
        if (millis() - lastRSSIDebug > 2000) { // Debug every 2 seconds
          Serial.print("DEBUG: Original RSSI=");
          Serial.print(telemetry.wifi_rssi);
          Serial.print(" | Beacon RSSI=");
          Serial.print(beacon_rssi);
          Serial.println("dBm");
          lastRSSIDebug = millis();
        }
        
      } else {
        sendLogMessage("ERROR", "Failed to parse 23-field CSV telemetry", "BaseStation");
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
  
  // Send to stderr so Python can differentiate logs from telemetry
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
  sendLogMessage("INFO", "🚀 N4 Base Station starting - Now with Beacon RSSI!", "BaseStation");

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

  sendLogMessage("INFO", "📡 Beacon RSSI capture enabled - Real signal strength monitoring!", "BaseStation");
  sendLogMessage("INFO", "🔧 Now sends actual beacon RSSI instead of 0", "BaseStation");
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
      telemetry_info["beacon_rssi"] = telemetry.wifi_rssi; // Now real beacon RSSI
      telemetry_info["battery_voltage"] = telemetry.battery_voltage;
      telemetry_info["velocity"] = telemetry.velocity;
      telemetry_info["altitude_agl"] = telemetry.altitude_agl;
    }
    
    statusDoc["waiting_for_commands"] = !commandPending;
    statusDoc["monitoring_active"] = true;
    
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
          ("🟡 Beacon - Packet age: " + String(packetAge/1000) + "s - Still monitoring").c_str(), 
          "BaseStation");
        timeoutWarningGiven = true;
      }
    } else {
      timeoutWarningGiven = false; // Reset warning for next timeout
    }
  }
  
  // Periodic waiting message for new connections
  if (!hasEverConnected && millis() - lastStatusTime > 30000) {
    sendLogMessage("INFO", "📡 Waiting for rocket beacon transmissions...", "BaseStation");
    sendLogMessage("INFO", "🔧 RSSI monitoring: Now captures real beacon signal strength!", "BaseStation");
    lastStatusTime = millis();
  }
  
  delay(10);
}
