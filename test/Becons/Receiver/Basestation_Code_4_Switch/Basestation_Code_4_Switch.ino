#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <ArduinoJson.h>

// Configuration
//uint8_t rocket_mac[] =  {0x10, 0x06, 0x1c, 0xa6, 0x18, 0x20}; // Rocket MAC
//uint8_t my_mac[] = {0x10, 0x06, 0x1c, 0xa6, 0x11, 0xf0}; // Base MAC
//10:06:1c:a6:11:f0

uint8_t rocket_mac[] = {0x84, 0x1f, 0xe8, 0x36, 0x73, 0xac}; // Rocket MAC
uint8_t my_mac[] = {0x08, 0xd1, 0xf9, 0x15, 0xa2, 0x0c}; // Base MAC


//uint8_t my_mac[] = {0xf4, 0x65, 0x0b, 0x48, 0x5c, 0xf8}; // Base MAC
//MAC: f4:65:0b:48:5c:f8
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

// Convert telemetry to JSON format exactly as server.py expects
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
  alt_data["kalman_altitude"] = telemetry.kalman_altitude;           // 2D Kalman filtered altitude
  alt_data["kalman_vertical_velocity"] = telemetry.kalman_vertical_velocity; // 2D Kalman filtered vertical velocity

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
  Serial.println(jsonString);
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
        sendTelemetryJSON();

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

  Serial.print("LOG:");
  Serial.println(logString);
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

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();

    if (command == "ARM" || command == "DISARM" || command == "RESET") {
      lastCommand = command;
      commandPending = true;
      commandSentTime = millis();
      sendLogMessage("INFO", ("Flight command received: " + command).c_str(), "BaseStation");
    }
    else if (command == "CMD_MQTT_MODE" || command == "MQTT_MODE" || command == "MQTT") {
      lastCommand = "CMD_MQTT_MODE";
      commandPending = true;
      commandSentTime = millis();
      sendLogMessage("INFO", "Communication mode command: Switch to MQTT mode", "BaseStation");
    }
    else if (command == "CMD_BEACON_MODE" || command == "BEACON_MODE" || command == "BEACON") {
      lastCommand = "CMD_BEACON_MODE";
      commandPending = true;
      commandSentTime = millis();
      sendLogMessage("INFO", "Communication mode command: Switch to Beacon mode", "BaseStation");
    }
    else if (command == "CMD_AUTO_FALLBACK_ON" || command == "AUTO_FALLBACK_ON" || command == "AUTO_ON") {
      lastCommand = "CMD_AUTO_FALLBACK_ON";
      commandPending = true;
      commandSentTime = millis();
      sendLogMessage("INFO", "Communication mode command: Enable auto-fallback", "BaseStation");
    }
    else if (command == "CMD_AUTO_FALLBACK_OFF" || command == "AUTO_FALLBACK_OFF" || command == "AUTO_OFF") {
      lastCommand = "CMD_AUTO_FALLBACK_OFF";
      commandPending = true;
      commandSentTime = millis();
      sendLogMessage("INFO", "Communication mode command: Disable auto-fallback", "BaseStation");
    }
    else if (command == "CMD_GET_MODE" || command == "GET_MODE" || command == "STATUS") {
      lastCommand = "CMD_GET_MODE";
      commandPending = true;
      commandSentTime = millis();
      sendLogMessage("INFO", "Communication mode command: Get current mode status", "BaseStation");
    }
    else if (command == "HELP") {
      sendLogMessage("INFO", "Available commands:", "BaseStation");
      sendLogMessage("INFO", "Flight: ARM, DISARM, RESET", "BaseStation");
      sendLogMessage("INFO", "Communication: MQTT, BEACON, AUTO_ON, AUTO_OFF, STATUS", "BaseStation");
    }
    else {
      sendLogMessage("WARNING", ("Unknown command: " + command + " (try HELP)").c_str(), "BaseStation");
    }
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

  sendLogMessage("INFO", "🚀 N4 Base Station - Beacon Mode with Smart Commands", "BaseStation");
  sendLogMessage("INFO", "📡 Supporting 25-field CSV beacon parsing with Kalman filter data", "BaseStation");

  WiFi.mode(WIFI_STA);
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

  wifi_promiscuous_filter_t filter = {
    .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT
  };
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_filter(&filter);
  esp_wifi_set_promiscuous_rx_cb(promiscuousRx);

  sendLogMessage("INFO", "📡 25-field beacon parsing enabled with Kalman filter data", "BaseStation");
  sendLogMessage("INFO", "🔧 Beacon RSSI measurement active", "BaseStation");
  sendLogMessage("INFO", "🎯 Velocity, battery & Kalman filter data from flight computer", "BaseStation");
  sendLogMessage("INFO", "🎛️ Smart command interface enabled", "BaseStation");
  sendLogMessage("INFO", "Commands: ARM, DISARM, RESET, MQTT, BEACON, AUTO_ON, AUTO_OFF, STATUS", "BaseStation");
}

void loop() {
  static uint32_t lastStatusTime = 0;
  static uint32_t lastHeartbeat = 0;

  updateConnectionStatus();
  handleSerialCommands();
  sendCommandToRocket();

  if (millis() - lastHeartbeat > 10000) {
    DynamicJsonDocument statusDoc(512);
    statusDoc["type"] = "status";
    statusDoc["armed"] = rocketArmed;
    statusDoc["packets_received"] = packetsReceived;
    statusDoc["uptime"] = millis();

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
    statusDoc["beacon_mode_only"] = true;
    statusDoc["csv_fields_expected"] = 25;
    statusDoc["kalman_filter_data"] = true;
    statusDoc["smart_commands_enabled"] = true;
    statusDoc["command_timeout_ms"] = COMMAND_TIMEOUT;

    String statusString;
    serializeJson(statusDoc, statusString);
    Serial.print("STATUS:");
    Serial.println(statusString);

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
    sendLogMessage("INFO", "Send ARM or STATUS command to establish communication", "BaseStation");
    lastStatusTime = millis();
  }

  delay(10);
}