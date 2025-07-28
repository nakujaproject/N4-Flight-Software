#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <ArduinoJson.h>

// Configuration
uint8_t rocket_mac[] = {0x08, 0xd1, 0xf9, 0x15, 0x9c, 0x40}; // Rocket MAC
uint8_t my_mac[] = {0xf4, 0x65, 0x0b, 0x48, 0x5c, 0xf8}; // Base MAC
bool rocketArmed = false;

// WiFi Configuration with custom IP
const char* ap_ssid = "N4-BaseStation";
const char* ap_password = "rocket2024";

// Custom IP configuration for the Access Point
IPAddress local_IP(10, 0, 1, 1);      // Base station IP
IPAddress gateway(10, 0, 1, 1);       // Gateway (same as base station)
IPAddress subnet(255, 255, 255, 0);   // Subnet mask

// MQTT Configuration for rocket communication
const char* mqtt_server = "10.0.1.1";  // Updated to match new base station IP
const int mqtt_port = 1883;
const char* rocket_telemetry_topic = "rocket/telemetry";
const char* rocket_command_topic = "rocket/commands";

// Web server and MQTT client
WebServer server(80);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

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

// Latest data for HTTP endpoints
String latestTelemetryJSON = "{}";
String latestLogJSON = "{}";

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

// Convert telemetry to JSON
void updateTelemetryJSON() {
  if (!dataReceived) return;
  
  DynamicJsonDocument doc(1024);
  
  // Root level fields
  doc["record_number"] = telemetry.record_number;
  doc["operation_mode"] = telemetry.operation_mode;
  doc["state"] = telemetry.state;
  doc["battery_voltage"] = 12.0;
  
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
  alt_data["velocity"] = 0.0;
  
  // Chute state structure
  JsonObject chute_state = doc.createNestedObject("chute_state");
  chute_state["pyro1_state"] = telemetry.drogue_pin_state;
  chute_state["pyro2_state"] = telemetry.main_chute_pin_state;
  
  // Metadata
  doc["timestamp"] = millis();
  doc["signal_strength"] = -50;
  doc["packets_received"] = packetsReceived;
  
  latestTelemetryJSON = "";
  serializeJson(doc, latestTelemetryJSON);
}

// Create log message
void createLogMessage(const char* level, const char* message, const char* source) {
  DynamicJsonDocument logDoc(256);
  logDoc["level"] = level;
  logDoc["message"] = message;
  logDoc["source"] = source;
  logDoc["timestamp"] = millis();
  
  latestLogJSON = "";
  serializeJson(logDoc, latestLogJSON);
  
  Serial.print("LOG:");
  Serial.println(latestLogJSON);
}

// Web server handlers
void handleTelemetry() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Content-Type", "application/json");
  
  if (dataReceived) {
    server.send(200, "application/json", latestTelemetryJSON);
  } else {
    server.send(503, "application/json", "{\"error\":\"No telemetry data available\"}");
  }
}

void handleCommand() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  
  if (server.method() == HTTP_OPTIONS) {
    server.send(200, "text/plain", "");
    return;
  }
  
  if (server.method() == HTTP_POST) {
    String body = server.arg("plain");
    DynamicJsonDocument doc(256);
    deserializeJson(doc, body);
    String command = doc["command"];
    
    if (command == "ARM" || command == "DISARM" || command == "DIS") {
      lastCommand = command;
      commandPending = true;
      
      if (command == "DISARM") {
        lastCommand = "DIS";
      }
      
      createLogMessage("INFO", ("HTTP Command received: " + command).c_str(), "BaseStation");
      server.send(200, "application/json", "{\"status\":\"Command received\",\"command\":\"" + command + "\"}");
    } else {
      server.send(400, "application/json", "{\"error\":\"Invalid command\"}");
    }
  } else {
    server.send(405, "application/json", "{\"error\":\"Method not allowed\"}");
  }
}

void handleStatus() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  
  DynamicJsonDocument statusDoc(512);
  statusDoc["type"] = "status";
  statusDoc["armed"] = rocketArmed;
  statusDoc["packets_received"] = packetsReceived;
  statusDoc["uptime"] = millis();
  statusDoc["last_packet_age"] = dataReceived ? (millis() - lastPacketTime) : 0;
  statusDoc["waiting_for_commands"] = !commandPending;
  statusDoc["wifi_clients"] = WiFi.softAPgetStationNum();
  statusDoc["mqtt_connected"] = mqttClient.connected();
  statusDoc["ap_ip"] = WiFi.softAPIP().toString();
  
  String statusString;
  serializeJson(statusDoc, statusString);
  
  server.send(200, "application/json", statusString);
}

void handleLogs() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", latestLogJSON);
}

// MQTT callback for rocket commands - FIXED TYPE CASTING
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, rocket_command_topic) == 0) {
    char command[20];
    // FIX: Cast to unsigned int to match types
    unsigned int copyLen = (length < 19U) ? length : 19U;
    strncpy(command, (char*)payload, copyLen);
    command[copyLen] = '\0';
    
    String cmd = String(command);
    cmd.trim();
    
    if (cmd == "ARM" || cmd == "DISARM" || cmd == "DIS") {
      lastCommand = cmd;
      commandPending = true;
      
      if (cmd == "DISARM") {
        lastCommand = "DIS";
      }
      
      createLogMessage("INFO", ("MQTT Command received: " + cmd).c_str(), "BaseStation");
    }
  }
}

// Setup MQTT client
void setupMQTT() {
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(onMqttMessage);
}

// Connect to MQTT
void connectMQTT() {
  while (!mqttClient.connected()) {
    createLogMessage("INFO", "Attempting MQTT connection...", "BaseStation");
    
    if (mqttClient.connect("N4-BaseStation")) {
      createLogMessage("INFO", "MQTT connected", "BaseStation");
      mqttClient.subscribe(rocket_command_topic);
      createLogMessage("INFO", ("Subscribed to " + String(rocket_command_topic)).c_str(), "BaseStation");
    } else {
      createLogMessage("ERROR", ("MQTT connection failed, rc=" + String(mqttClient.state())).c_str(), "BaseStation");
      delay(2000);
    }
  }
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
        rocketArmed = (telemetry.operation_mode == 1);
        packetsReceived++;
        lastPacketTime = millis();
        dataReceived = true;
        
        // Update JSON for web serving
        updateTelemetryJSON();
        
        // Send via MQTT if connected
        if (mqttClient.connected()) {
          mqttClient.publish(rocket_telemetry_topic, latestTelemetryJSON.c_str());
        }
        
        // Optional: Send log message for state changes
        static uint8_t lastOperationMode = 255;
        if (lastOperationMode != telemetry.operation_mode) {
          const char* mode = (telemetry.operation_mode == 1) ? "ARMED" : "SAFE";
          createLogMessage("INFO", ("Rocket operation mode: " + String(mode)).c_str(), "BaseStation");
          lastOperationMode = telemetry.operation_mode;
        }
      } else {
        createLogMessage("ERROR", "Failed to parse CSV telemetry", "BaseStation");
      }
      break;
    }
  }
}

void promiscuousRx(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type == WIFI_PKT_MGMT) {
    handleBeacon((wifi_promiscuous_pkt_t*)buf);
  }
}

// Send commands to rocket via ESP-NOW and MQTT
void sendCommandToRocket() {
  if (!commandPending) return;
  
  esp_err_t espnow_result = ESP_FAIL;
  bool mqtt_sent = false;
  
  // Send via ESP-NOW
  if (lastCommand == "ARM") {
    espnow_result = esp_now_send(rocket_mac, (uint8_t*)"ARM", 3);
  } else if (lastCommand == "DIS") {
    espnow_result = esp_now_send(rocket_mac, (uint8_t*)"DISARM", 6);
  }
  
  // Send via MQTT if connected
  if (mqttClient.connected()) {
    String mqtt_command = (lastCommand == "DIS") ? "DISARM" : lastCommand;
    mqtt_sent = mqttClient.publish(rocket_command_topic, mqtt_command.c_str());
  }
  
  // Log results
  if (espnow_result == ESP_OK || mqtt_sent) {
    String methods = "";
    if (espnow_result == ESP_OK) methods += "ESP-NOW ";
    if (mqtt_sent) methods += "MQTT ";
    createLogMessage("INFO", ("Command sent via " + methods + ": " + lastCommand).c_str(), "BaseStation");
  } else {
    createLogMessage("ERROR", ("Failed to send command: " + lastCommand).c_str(), "BaseStation");
  }
  
  commandPending = false;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("N4 Base Station starting up...");

  // Configure WiFi Access Point with custom IP
  WiFi.mode(WIFI_AP_STA);
  
  // Set custom IP configuration BEFORE creating the AP
  if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("Failed to configure AP IP");
  }
  
  // Create the Access Point
  if (!WiFi.softAP(ap_ssid, ap_password)) {
    Serial.println("Failed to create Access Point");
    ESP.restart();
  }
  
  // Set custom MAC for STA mode
  esp_wifi_set_mac(WIFI_IF_STA, my_mac);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  
  // Display network information
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Base Station IP: ");
  Serial.println(IP);
  Serial.println("WiFi Network: " + String(ap_ssid));
  Serial.println("Password: " + String(ap_password));
  Serial.println("Gateway: " + gateway.toString());
  Serial.println("Subnet: " + subnet.toString());

  // Setup MQTT
  setupMQTT();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    ESP.restart();
  }

  // Register peer (rocket)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, rocket_mac, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add rocket as peer");
  } else {
    Serial.println("Rocket peer registered successfully");
  }

  // Setup promiscuous mode for beacon reception
  wifi_promiscuous_filter_t filter = {
    .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT
  };
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_filter(&filter);
  esp_wifi_set_promiscuous_rx_cb(promiscuousRx);

  // Setup web server endpoints
  server.on("/telemetry", HTTP_GET, handleTelemetry);
  server.on("/command", HTTP_POST, handleCommand);
  server.on("/command", HTTP_OPTIONS, handleCommand); // CORS preflight
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/logs", HTTP_GET, handleLogs);
  
  // Start web server
  server.begin();
  Serial.println("Web server started on port 80");
  Serial.println("Access the base station at: http://" + IP.toString());
  
  createLogMessage("INFO", "Base station ready - WiFi + MQTT mode", "BaseStation");
}

void loop() {
  static uint32_t lastMqttCheck = 0;
  
  // Handle web server requests
  server.handleClient();
  
  // Handle MQTT
  if (!mqttClient.connected() && millis() - lastMqttCheck > 5000) {
    connectMQTT();
    lastMqttCheck = millis();
  }
  mqttClient.loop();
  
  // Send pending commands to rocket
  sendCommandToRocket();
  
  // Check for data timeout
  if (dataReceived && (millis() - lastPacketTime > 15000)) {
    createLogMessage("WARNING", "No telemetry received for >15 seconds", "BaseStation");
  }
  
  delay(10);
}