#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

// Configuration
uint8_t rocket_mac[] = {0x08, 0xd1, 0xf9, 0x15, 0x9c, 0x40}; // Rocket MAC
uint8_t my_mac[] = {0xf4, 0x65, 0x0b, 0x48, 0x5c, 0xf8}; // Base MAC
bool rocketArmed = false;

// Telemetry data structure to match your CSV format
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

// Function to get state name
const char* getStateName(uint8_t state) {
  switch(state) {
    case 0: return "PRE_FLIGHT";
    case 1: return "POWERED_FLIGHT";
    case 2: return "COASTING";
    case 3: return "APOGEE";
    case 4: return "DROGUE_DEPLOY";
    case 5: return "DROGUE_DESCENT";
    case 6: return "MAIN_DEPLOY";
    case 7: return "MAIN_DESCENT";
    case 8: return "POST_FLIGHT";
    default: return "UNKNOWN";
  }
}

// Function to get operation mode name
const char* getModeName(uint8_t mode) {
  return (mode == 1) ? "ARMED" : "SAFE";
}

// Function to get chute state
const char* getChuteState(uint8_t state) {
  return (state == 1) ? "DEPLOYED" : "STOWED";
}

void handleBeacon(const wifi_promiscuous_pkt_t* pkt) {
  const uint8_t* payload = pkt->payload;
  int len = pkt->rx_ctrl.sig_len;

  // Basic validation - check if it's a beacon frame
  if (len < 60 || payload[0] != 0x80) return;
  
  // Check if it's from our rocket
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
        rocketArmed = true;
        packetsReceived++;
        lastPacketTime = millis();
        
        // Display telemetry data
        Serial.println("\n🚀═══════════════════════════════════════════════════════════");
        Serial.printf("📡 TELEMETRY RECEIVED - Packet #%lu (Total: %lu)\n", 
                     telemetry.record_number, packetsReceived);
        Serial.println("═══════════════════════════════════════════════════════════");
        
        Serial.printf("🎯 Status: %s | State: %s\n", 
                     getModeName(telemetry.operation_mode), 
                     getStateName(telemetry.state));
        
        Serial.printf("📐 Acceleration: X=%.2fg Y=%.2fg Z=%.2fg\n", 
                     telemetry.ax, telemetry.ay, telemetry.az);
        
        Serial.printf("🎯 Attitude: Pitch=%.2f° Roll=%.2f°\n", 
                     telemetry.pitch, telemetry.roll);
        
        Serial.printf("🌀 Gyro: X=%.2f°/s Y=%.2f°/s Z=%.2f°/s\n", 
                     telemetry.gx, telemetry.gy, telemetry.gz);
        
        Serial.printf("🌍 GPS: Lat=%.4f° Lon=%.4f° Alt=%.2fm\n", 
                     telemetry.latitude, telemetry.longitude, telemetry.gps_altitude);
        
        Serial.printf("🏔️  Barometric: Pressure=%.2fPa Temp=%.2f°C Alt=%.2fm\n", 
                     telemetry.pressure, telemetry.temperature, telemetry.altitude_agl);
        
        Serial.printf("🪂 Parachutes: Drogue=%s Main=%s\n", 
                     getChuteState(telemetry.drogue_pin_state),
                     getChuteState(telemetry.main_chute_pin_state));
        
        Serial.printf("📶 Signal: %ddBm | ⏱️  Uptime: %lums\n", 
                     pkt->rx_ctrl.rssi, millis());
        
        Serial.printf("📊 Raw CSV: %s\n", csv_data);
        Serial.println("═══════════════════════════════════════════════════════════\n");
      } else {
        Serial.printf("❌ Failed to parse CSV: %s\n", csv_data);
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

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("🚀 N4 Flight Computer Beacon Receiver");
  Serial.println("═══════════════════════════════════════");
  Serial.printf("🎯 Listening for rocket: %02X:%02X:%02X:%02X:%02X:%02X\n",
               rocket_mac[0], rocket_mac[1], rocket_mac[2], 
               rocket_mac[3], rocket_mac[4], rocket_mac[5]);
  Serial.printf("📡 Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
               my_mac[0], my_mac[1], my_mac[2], 
               my_mac[3], my_mac[4], my_mac[5]);
  Serial.println("═══════════════════════════════════════\n");

  // Configure WiFi
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, my_mac);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed");
    ESP.restart();
  }

  // Register peer (rocket)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, rocket_mac, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add peer");
  } else {
    Serial.println("✅ ESP-NOW peer registered");
  }

  // Setup promiscuous mode for beacon reception
  wifi_promiscuous_filter_t filter = {
    .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT
  };
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_filter(&filter);
  esp_wifi_set_promiscuous_rx_cb(promiscuousRx);

  Serial.println("🔍 Ready to receive telemetry beacons...\n");
}

void loop() {
  static uint32_t lastArmTime = 0;
  static uint32_t lastStatusTime = 0;
  
  // Send ARM command every 2 seconds until rocket responds
  if (!rocketArmed && millis() - lastArmTime > 2000) {
    esp_err_t result = esp_now_send(rocket_mac, (uint8_t*)"ARM", 3);
    if (result == ESP_OK) {
      Serial.println("🛜 Sent ARM command to rocket");
    } else {
      Serial.println("❌ Failed to send ARM command");
    }
    lastArmTime = millis();
  }
  
  // Status update every 15 seconds
  if (millis() - lastStatusTime > 15000) {
    if (rocketArmed) {
      uint32_t timeSinceLastPacket = millis() - lastPacketTime;
      Serial.printf("💓 Status: %lu packets received | Last packet: %lu ms ago\n", 
                   packetsReceived, timeSinceLastPacket);
      
      if (timeSinceLastPacket > 10000) {
        Serial.println("⚠️  Warning: No packets received for >10 seconds");
      }
    } else {
      Serial.println("⏳ Waiting for rocket response...");
    }
    lastStatusTime = millis();
  }
  
  delay(10);
}