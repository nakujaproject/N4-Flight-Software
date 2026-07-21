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

// Telemetry data structure
struct TelemetryData {
  uint32_t record_number;
  uint8_t operation_mode;
  uint8_t state;
  float ax, ay, az;
  float pitch, roll;
  float gx, gy, gz;
  float latitude, longitude;
  float gps_altitude;
  float pressure;
  float temperature;
  float altitude_agl;
  uint8_t drogue_pin_state;
  uint8_t main_chute_pin_state;
};

TelemetryData telemetry;
uint32_t packetsReceived = 0;
uint32_t lastPacketTime = 0;
bool dataReceived = false;

// Command handling
String lastCommand = "";
bool commandPending = false;

// Function to parse CSV string into telemetry struct
bool parseCSV(const char* csv, TelemetryData& data) {
  return sscanf(csv, "%lu,%hhu,%hhu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%hhu,%hhu",
                &data.record_number,
                &data.operation_mode,
                &data.state,
                &data.ax, &data.ay, &data.az,
                &data.pitch, &data.roll,
                &data.gx, &data.gy, &data.gz,
                &data.latitude, &data.longitude,
                &data.gps_altitude,
                &data.pressure,
                &data.temperature,
                &data.altitude_agl,
                &data.drogue_pin_state,
                &data.main_chute_pin_state) == 19;
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

// Convert telemetry to JSON format with enhanced connection info
void sendTelemetryJSON() {
  if (!dataReceived) return;
  
  updateConnectionStatus();
  
  // Create JSON document with the structure expected by your React app
  DynamicJsonDocument doc(1024);
  
  // Root level fields
  doc["record_number"] = telemetry.record_number;
  doc["operation_mode"] = telemetry.operation_mode;
  doc["state"] = telemetry.state;
  doc["battery_voltage"] = 12.0; // Placeholder - add if available from rocket
  
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
  
  // Altimeter data structure
  JsonObject alt_data = doc.createNestedObject("alt_data");
  alt_data["pressure"] = telemetry.pressure;
  alt_data["temperature"] = telemetry.temperature;
  alt_data["AGL"] = telemetry.altitude_agl;
  alt_data["velocity"] = 0.0; // Placeholder - calculate if needed
  
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
  
  // Metadata
  doc["timestamp"] = millis();
  doc["signal_strength"] = -50; // Placeholder
  doc["packets_received"] = packetsReceived;
  
  // Serialize and send to Python script via Serial
  String jsonString;
  serializeJson(doc, jsonString);
  Serial.println(jsonString);
}

void handleBeacon(const wifi_promiscuous_pkt_t* pkt) {
  const uint8_t* payload = pkt->payload;
  int len = pkt->rx_ctrl.sig_len;

  // Basic validation
  if (len < 60 || payload[0] != 0x80) return;
  if (memcmp(&payload[10], rocket_mac, 6) != 0) return;

  // Find telemetry data (0xDD vendor tag)
  for (int i = 36; i < len - 2; i++) {
    if (payload[i] == 0xDD) {
      uint8_t data_len = payload[i + 1];
      
      // Create null-terminated string
      char csv_data[256];
      memcpy(csv_data, &payload[i + 2], data_len);
      csv_data[data_len] = '\0';
      
      // Parse CSV data
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
      } else {
        sendLogMessage("ERROR", "Failed to parse CSV telemetry", "BaseStation");
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

// Handle commands from Python script (ARM/DIS)
void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "ARM" || command == "DISARM" || command == "DIS") {
      lastCommand = command;
      commandPending = true;
      
      // Convert DISARM to DIS for consistency
      if (command == "DISARM") {
        lastCommand = "DIS";
      }
      
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
  } else if (lastCommand == "DIS") {
    // Send "DISARM" to match what flight computer expects
    result = esp_now_send(rocket_mac, (uint8_t*)"DISARM", 6);
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
  sendLogMessage("INFO", "🚀 N4 Base Station starting up...", "BaseStation");

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

  sendLogMessage("INFO", "📡 Continuous monitoring enabled - connection persists beyond 15s", "BaseStation");
  sendLogMessage("INFO", "Use ARM/DISARM commands to control rocket", "BaseStation");
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
          ("🟡 Packet age: " + String(packetAge/1000) + "s - Still monitoring for reconnection").c_str(), 
          "BaseStation");
        timeoutWarningGiven = true;
      }
    } else {
      timeoutWarningGiven = false; // Reset warning for next timeout
    }
  }
  
  // Periodic waiting message for new connections
  if (!hasEverConnected && millis() - lastStatusTime > 30000) {
    sendLogMessage("INFO", "📡 Continuous monitoring active - Waiting for rocket telemetry...", "BaseStation");
    sendLogMessage("INFO", "Send ARM command from dashboard to establish communication", "BaseStation");
    lastStatusTime = millis();
  }
  
  delay(10);
}