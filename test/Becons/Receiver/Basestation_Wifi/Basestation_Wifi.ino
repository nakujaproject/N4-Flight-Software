#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <ArduinoJson.h>
#include <PubSubClient.h>

// WiFi and MQTT Configuration - UPDATE THESE VALUES
const char* ssid = "glenphone"; // Your WiFi network name
const char* password = "012345678900";   // Your WiFi password
const char* mqtt_server = "172.29.160.182"; // Same IP your React app connects to

// MQTT Topics (matching your React app)
const char* TELEMETRY_TOPIC = "n4/app/flight-computer-1";
const char* LOG_TOPIC = "n4/app/logs";
const char* COMMAND_TOPIC = "n4/commands";

// MQTT Client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
bool mqttConnected = false;

// Configuration
uint8_t rocket_mac[] = {0x10, 0x06, 0x1c, 0xa6, 0x11, 0xf0}; // Rocket MAC
uint8_t my_mac[] = {0x08, 0xd1, 0xf9, 0x15, 0xa2, 0x0c}; // Base MAC

bool rocketArmed = false;

// Connection status tracking
const uint32_t CONNECTION_TIMEOUT = 15000; // 15 seconds
bool hasEverConnected = false;
bool currentlyConnected = false;

// Updated telemetry data structure to match flight computer's 25-field CSV with Kalman filter outputs
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
  int32_t wifi_rssi;             // 22: RSSI from telemetry packet (flight computer sends 0 in beacon mode)
  float kalman_altitude;         // 23: 2D Kalman filtered altitude
  float kalman_vertical_velocity; // 24: 2D Kalman filtered vertical velocity
};

TelemetryData telemetry;
int8_t beacon_rssi = 0;  // Measured beacon RSSI (separate from CSV data)
uint32_t packetsReceived = 0;
uint32_t lastPacketTime = 0;
bool dataReceived = false;

// Enhanced command handling with communication mode support
String lastCommand = "";
bool commandPending = false;
uint32_t commandSentTime = 0;
const uint32_t COMMAND_TIMEOUT = 5000; // 5 seconds

// Function to parse 25-field CSV string (matches flight computer beacon output with Kalman filter data)
bool parseCSV(const char* csv, TelemetryData& data) {
  return sscanf(csv, "%lu,%hhu,%hhu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%lu,%f,%f,%f,%f,%hhu,%hhu,%f,%d,%f,%f",
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
                &data.wifi_rssi,                        // 22 (flight computer sends 0 in beacon mode)
                &data.kalman_altitude,                  // 23 - 2D Kalman filtered altitude
                &data.kalman_vertical_velocity) == 25;  // 24 - 2D Kalman filtered vertical velocity
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

// WiFi setup function
void setupWiFi() {
  WiFi.mode(WIFI_AP_STA); // Enable both AP and STA modes for ESP-NOW + WiFi
  WiFi.begin(ssid, password);
  
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("✅ WiFi connected. IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("❌ WiFi connection failed - continuing with beacon-only mode");
  }
}

// MQTT callback for incoming commands
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();
  
  if (String(topic) == COMMAND_TOPIC) {
    processCommand(message, "MQTT");
  }
}

// MQTT connection function
void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(onMqttMessage);
  
  String clientId = "N4_BaseStation_" + String(random(0xffff), HEX);
  
  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("✅ MQTT connected");
    mqttClient.subscribe(COMMAND_TOPIC);
    mqttConnected = true;
    sendLogMessage("INFO", "🟢 MQTT Base Station connected", "BaseStation");
  } else {
    Serial.print("❌ MQTT connection failed, rc=");
    Serial.println(mqttClient.state());
    mqttConnected = false;
  }
}

// Convert telemetry to JSON and send to both Serial and MQTT
void sendTelemetryJSON() {
  if (!dataReceived) return;
  updateConnectionStatus();

  DynamicJsonDocument doc(1024);

  doc["record_number"] = telemetry.record_number;
  doc["operation_mode"] = telemetry.operation_mode;
  doc["state"] = telemetry.state;
  doc["battery_voltage"] = telemetry.battery_voltage;
  doc["wifi_rssi"] = beacon_rssi; // Beacon RSSI measured by base station

  JsonObject acc_data = doc.createNestedObject("acc_data");
  acc_data["ax"] = telemetry.ax;
  acc_data["ay"] = telemetry.ay;
  acc_data["az"] = telemetry.az;
  acc_data["pitch"] = telemetry.pitch;
  acc_data["roll"] = telemetry.roll;

  JsonObject gyro_data = doc.createNestedObject("gyro_data");
  gyro_data["gx"] = telemetry.gx;
  gyro_data["gy"] = telemetry.gy;
  gyro_data["gz"] = telemetry.gz;

  JsonObject gps_data = doc.createNestedObject("gps_data");
  gps_data["latitude"] = telemetry.latitude;
  gps_data["longitude"] = telemetry.longitude;
  gps_data["gps_altitude"] = telemetry.gps_altitude;
  gps_data["time"] = telemetry.gps_time;

  JsonObject alt_data = doc.createNestedObject("alt_data");
  alt_data["pressure"] = telemetry.pressure;
  alt_data["temperature"] = telemetry.temperature;
  alt_data["AGL"] = telemetry.altitude_agl;
  alt_data["velocity"] = telemetry.velocity;
  alt_data["kalman_altitude"] = telemetry.kalman_altitude;           
  alt_data["kalman_vertical_velocity"] = telemetry.kalman_vertical_velocity; 

  JsonObject chute_state = doc.createNestedObject("chute_state");
  chute_state["pyro1_state"] = telemetry.drogue_pin_state;
  chute_state["pyro2_state"] = telemetry.main_chute_pin_state;

  JsonObject conn_status = doc.createNestedObject("connection_status");
  conn_status["connected"] = currentlyConnected;
  conn_status["has_ever_connected"] = hasEverConnected;
  conn_status["packet_age_ms"] = millis() - lastPacketTime;
  conn_status["timeout_exceeded"] = (millis() - lastPacketTime) > CONNECTION_TIMEOUT;
  conn_status["rssi"] = beacon_rssi;

  doc["communication_mode"] = "Beacon";
  doc["timestamp"] = millis();
  doc["packets_received"] = packetsReceived;

  String jsonString;
  serializeJson(doc, jsonString);
  
  // Send to Serial (for debugging/backup)
  Serial.println(jsonString);
  
  // Send to MQTT (for React app)
  if (mqttConnected) {
    mqttClient.publish(TELEMETRY_TOPIC, jsonString.c_str());
  }
}

void handleBeacon(const wifi_promiscuous_pkt_t* pkt) {
  const uint8_t* payload = pkt->payload;
  int len = pkt->rx_ctrl.sig_len;

  beacon_rssi = pkt->rx_ctrl.rssi;

  if (len < 60 || payload[0] != 0x80) return;
  if (memcmp(&payload[10], rocket_mac, 6) != 0) return;

  for (int i = 36; i < len - 2; i++) {
    if (payload[i] == 0xDD) {
      uint8_t data_len = payload[i + 1];
      char csv_data[512];
      memcpy(csv_data, &payload[i + 2], data_len);
      csv_data[data_len] = '\0';

      if (parseCSV(csv_data, telemetry)) {
        bool wasConnected = currentlyConnected;
        rocketArmed = (telemetry.operation_mode == 1);
        packetsReceived++;
        lastPacketTime = millis();
        dataReceived = true;

        updateConnectionStatus();
        sendTelemetryJSON(); // Now sends to both Serial and MQTT

        if (!wasConnected && currentlyConnected) {
          sendLogMessage("INFO", "🟢 CONNECTED - Rocket beacon link established", "BaseStation");
        }

        static uint8_t lastOperationMode = 255;
        if (lastOperationMode != telemetry.operation_mode) {
          const char* mode = (telemetry.operation_mode == 1) ? "ARMED" : "SAFE";
          sendLogMessage("INFO", ("Rocket operation mode: " + String(mode)).c_str(), "BaseStation");
          lastOperationMode = telemetry.operation_mode;
        }
      } else {
        sendLogMessage("ERROR", "Failed to parse 25-field CSV telemetry", "BaseStation");
      }
      break;
    }
  }
}

void sendLogMessage(const char* level, const char* message, const char* source) {
  DynamicJsonDocument logDoc(256);
  logDoc["level"] = level;
  logDoc["message"] = message;
  logDoc["source"] = source;
  logDoc["timestamp"] = millis();

  String logString;
  serializeJson(logDoc, logString);

  // Send to Serial (for debugging)
  Serial.print("LOG:");
  Serial.println(logString);
  
  // Send to MQTT (for React app logs)
  if (mqttConnected) {
    mqttClient.publish(LOG_TOPIC, logString.c_str());
  }
}

void promiscuousRx(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type == WIFI_PKT_MGMT) {
    handleBeacon((wifi_promiscuous_pkt_t*)buf);
  }
}

// Updated ESP-NOW receive callback for Arduino-ESP32 v3.0+ API
void onESPNowDataReceived(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  String response = "";
  for (int i = 0; i < len; i++) {
    response += (char)incomingData[i];
  }
  response.trim();

  sendLogMessage("INFO", ("ESP-NOW response from rocket: " + response).c_str(), "BaseStation");

  if (response.indexOf("Mode:") != -1) {
    sendLogMessage("INFO", ("Rocket communication status: " + response).c_str(), "BaseStation");
  }
  else if (response.indexOf("ARMED") != -1 || response.indexOf("DISARMED") != -1) {
    sendLogMessage("INFO", ("Rocket flight status: " + response).c_str(), "BaseStation");
  }
  else if (response.indexOf("Auto-fallback") != -1) {
    sendLogMessage("INFO", ("Rocket auto-fallback: " + response).c_str(), "BaseStation");
  }
}

// Process commands from MQTT or Serial
void processCommand(String command, String source) {
  command.trim();
  command.toUpperCase();

  if (command == "ARM" || command == "DISARM" || command == "RESET") {
    lastCommand = command;
    commandPending = true;
    commandSentTime = millis();
    sendLogMessage("INFO", ("Flight command received: " + command + " (from " + source + ")").c_str(), "BaseStation");
  }
  else if (command == "CMD_MQTT_MODE" || command == "MQTT_MODE" || command == "MQTT") {
    lastCommand = "CMD_MQTT_MODE";
    commandPending = true;
    commandSentTime = millis();
    sendLogMessage("INFO", ("Communication mode command: Switch to MQTT mode (from " + source + ")").c_str(), "BaseStation");
  }
  else if (command == "CMD_BEACON_MODE" || command == "BEACON_MODE" || command == "BEACON") {
    lastCommand = "CMD_BEACON_MODE";
    commandPending = true;
    commandSentTime = millis();
    sendLogMessage("INFO", ("Communication mode command: Switch to Beacon mode (from " + source + ")").c_str(), "BaseStation");
  }
  else if (command == "CMD_AUTO_FALLBACK_ON" || command == "AUTO_FALLBACK_ON" || command == "AUTO_ON") {
    lastCommand = "CMD_AUTO_FALLBACK_ON";
    commandPending = true;
    commandSentTime = millis();
    sendLogMessage("INFO", ("Communication mode command: Enable auto-fallback (from " + source + ")").c_str(), "BaseStation");
  }
  else if (command == "CMD_AUTO_FALLBACK_OFF" || command == "AUTO_FALLBACK_OFF" || command == "AUTO_OFF") {
    lastCommand = "CMD_AUTO_FALLBACK_OFF";
    commandPending = true;
    commandSentTime = millis();
    sendLogMessage("INFO", ("Communication mode command: Disable auto-fallback (from " + source + ")").c_str(), "BaseStation");
  }
  else if (command == "CMD_GET_MODE" || command == "GET_MODE" || command == "STATUS") {
    lastCommand = "CMD_GET_MODE";
    commandPending = true;
    commandSentTime = millis();
    sendLogMessage("INFO", ("Communication mode command: Get current mode status (from " + source + ")").c_str(), "BaseStation");
  }
  else if (command == "HELP") {
    sendLogMessage("INFO", "Available commands:", "BaseStation");
    sendLogMessage("INFO", "Flight: ARM, DISARM, RESET", "BaseStation");
    sendLogMessage("INFO", "Communication: MQTT, BEACON, AUTO_ON, AUTO_OFF, STATUS", "BaseStation");
  }
  else {
    sendLogMessage("WARNING", ("Unknown command: " + command + " (try HELP) from " + source).c_str(), "BaseStation");
  }
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    processCommand(command, "SERIAL");
  }
}

void sendCommandToRocket() {
  if (!commandPending) return;

  if (millis() - commandSentTime > COMMAND_TIMEOUT) {
    sendLogMessage("WARNING", ("Command timeout: " + lastCommand).c_str(), "BaseStation");
    commandPending = false;
    return;
  }

  esp_err_t result = ESP_FAIL;

  if (lastCommand == "ARM") {
    result = esp_now_send(rocket_mac, (uint8_t*)"ARM", 3);
  }
  else if (lastCommand == "DISARM") {
    result = esp_now_send(rocket_mac, (uint8_t*)"DISARM", 6);
  }
  else if (lastCommand == "RESET") {
    result = esp_now_send(rocket_mac, (uint8_t*)"RESET", 5);
  }
  else if (lastCommand == "CMD_MQTT_MODE") {
    result = esp_now_send(rocket_mac, (uint8_t*)"CMD_MQTT_MODE", 13);
  }
  else if (lastCommand == "CMD_BEACON_MODE") {
    result = esp_now_send(rocket_mac, (uint8_t*)"CMD_BEACON_MODE", 15);
  }
  else if (lastCommand == "CMD_AUTO_FALLBACK_ON") {
    result = esp_now_send(rocket_mac, (uint8_t*)"CMD_AUTO_FALLBACK_ON", 20);
  }
  else if (lastCommand == "CMD_AUTO_FALLBACK_OFF") {
    result = esp_now_send(rocket_mac, (uint8_t*)"CMD_AUTO_FALLBACK_OFF", 21);
  }
  else if (lastCommand == "CMD_GET_MODE") {
    result = esp_now_send(rocket_mac, (uint8_t*)"CMD_GET_MODE", 12);
  }

  if (result == ESP_OK) {
    sendLogMessage("INFO", ("ESP-NOW command sent: " + lastCommand).c_str(), "BaseStation");
  } else {
    sendLogMessage("ERROR", ("ESP-NOW command failed: " + lastCommand).c_str(), "BaseStation");
  }

  commandPending = false;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  sendLogMessage("INFO", "🚀 N4 Base Station - Beacon + MQTT Gateway Mode", "BaseStation");
  sendLogMessage("INFO", "📡 Supporting 25-field CSV beacon parsing with Kalman filter data", "BaseStation");

  // Setup WiFi for MQTT
  setupWiFi();

  // Setup ESP-NOW
  WiFi.mode(WIFI_AP_STA); // Ensure we're in dual mode
  esp_wifi_set_mac(WIFI_IF_STA, my_mac);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    sendLogMessage("ERROR", "ESP-NOW initialization failed", "BaseStation");
    ESP.restart();
  }

  // Register ESP-NOW receive callback (new API)
  esp_now_register_recv_cb(onESPNowDataReceived);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, rocket_mac, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    sendLogMessage("ERROR", "Failed to add rocket as peer", "BaseStation");
  } else {
    sendLogMessage("INFO", "Rocket peer registered successfully", "BaseStation");
  }

  // Setup beacon sniffing
  wifi_promiscuous_filter_t filter = {
    .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT
  };
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_filter(&filter);
  esp_wifi_set_promiscuous_rx_cb(promiscuousRx);

  // Connect to MQTT
  if (WiFi.status() == WL_CONNECTED) {
    connectMQTT();
  }

  sendLogMessage("INFO", "📡 25-field beacon parsing enabled with Kalman filter data", "BaseStation");
  sendLogMessage("INFO", "🔧 Beacon RSSI measurement active", "BaseStation");
  sendLogMessage("INFO", "🎯 Velocity, battery & Kalman filter data from flight computer", "BaseStation");
  sendLogMessage("INFO", "🎛️ Smart command interface enabled (Serial + MQTT)", "BaseStation");
  sendLogMessage("INFO", "📨 Publishing to: n4/app/flight-computer-1", "BaseStation");
  sendLogMessage("INFO", "📥 Listening for commands on: n4/commands", "BaseStation");
  sendLogMessage("INFO", "Commands: ARM, DISARM, RESET, MQTT, BEACON, AUTO_ON, AUTO_OFF, STATUS", "BaseStation");
}

void loop() {
  static uint32_t lastStatusTime = 0;
  static uint32_t lastHeartbeat = 0;
  static uint32_t lastMqttReconnect = 0;

  updateConnectionStatus();
  handleSerialCommands();
  sendCommandToRocket();

  // Handle MQTT
  if (mqttConnected) {
    mqttClient.loop();
    if (!mqttClient.connected()) {
      mqttConnected = false;
      sendLogMessage("WARNING", "MQTT connection lost", "BaseStation");
    }
  } else if (WiFi.status() == WL_CONNECTED && millis() - lastMqttReconnect > 5000) {
    // Try to reconnect MQTT every 5 seconds if WiFi is connected
    connectMQTT();
    lastMqttReconnect = millis();
  }

  if (millis() - lastHeartbeat > 10000) {
    DynamicJsonDocument statusDoc(512);
    statusDoc["type"] = "status";
    statusDoc["armed"] = rocketArmed;
    statusDoc["packets_received"] = packetsReceived;
    statusDoc["uptime"] = millis();
    statusDoc["mqtt_connected"] = mqttConnected;
    statusDoc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);

    JsonObject conn_info = statusDoc.createNestedObject("connection");
    conn_info["currently_connected"] = currentlyConnected;
    conn_info["has_ever_connected"] = hasEverConnected;
    conn_info["last_packet_age"] = dataReceived ? (millis() - lastPacketTime) : 0;
    conn_info["timeout_exceeded"] = dataReceived && (millis() - lastPacketTime > CONNECTION_TIMEOUT);

    if (dataReceived) {
      JsonObject telemetry_info = statusDoc.createNestedObject("telemetry");
      telemetry_info["record_number"] = telemetry.record_number;
      telemetry_info["communication_mode"] = "Beacon";
      telemetry_info["beacon_rssi"] = beacon_rssi;
      telemetry_info["battery_voltage"] = telemetry.battery_voltage;
      telemetry_info["velocity"] = telemetry.velocity;
      telemetry_info["altitude_agl"] = telemetry.altitude_agl;
      telemetry_info["kalman_altitude"] = telemetry.kalman_altitude;
      telemetry_info["kalman_vertical_velocity"] = telemetry.kalman_vertical_velocity;
      telemetry_info["operation_mode"] = telemetry.operation_mode;
      telemetry_info["state"] = telemetry.state;
    }

    statusDoc["waiting_for_commands"] = !commandPending;
    statusDoc["monitoring_active"] = true;
    statusDoc["beacon_mode_enabled"] = true;
    statusDoc["mqtt_mode_enabled"] = mqttConnected;
    statusDoc["csv_fields_expected"] = 25;
    statusDoc["kalman_filter_data"] = true;
    statusDoc["smart_commands_enabled"] = true;
    statusDoc["command_timeout_ms"] = COMMAND_TIMEOUT;

    String statusString;
    serializeJson(statusDoc, statusString);
    Serial.print("STATUS:");
    Serial.println(statusString);

    // Send status summary via log
    String statusMsg = "📊 Status - Packets: " + String(packetsReceived) + 
                      ", Beacon: " + String(currentlyConnected ? "✅" : "❌") +
                      ", MQTT: " + String(mqttConnected ? "✅" : "❌") +
                      ", RSSI: " + String(beacon_rssi) + "dBm";
    sendLogMessage("INFO", statusMsg.c_str(), "BaseStation");

    lastHeartbeat = millis();
  }

  if (dataReceived) {
    uint32_t packetAge = millis() - lastPacketTime;
    static bool timeoutWarningGiven = false;

    if (packetAge > CONNECTION_TIMEOUT) {
      if (!timeoutWarningGiven) {
        sendLogMessage("WARNING",
          ("🟡 Beacon aged: " + String(packetAge/1000) + "s | RSSI: " + String(beacon_rssi) + "dBm").c_str(),
          "BaseStation");
        timeoutWarningGiven = true;
      }
    } else {
      timeoutWarningGiven = false;
    }
  }

  if (!hasEverConnected && millis() - lastStatusTime > 30000) {
    sendLogMessage("INFO", "📡 Waiting for 25-field beacon transmissions with Kalman filter data...", "BaseStation");
    sendLogMessage("INFO", "🌐 MQTT Gateway mode - publishing to React app", "BaseStation");
    sendLogMessage("INFO", "Send ARM or STATUS command to establish communication", "BaseStation");
    lastStatusTime = millis();
  }

  delay(10);
}