#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

// Configuration
uint8_t rocket_mac[] = {0x08, 0xd1, 0xf9, 0x15, 0x9c, 0x40}; // Rocket MAC
uint8_t my_mac[] = {0x10, 0x06, 0x1c, 0xa6, 0x18, 0x20};     // Base MAC
// Add these near your other MAC address definitions
//const uint8_t ROCKET_MAC[] = {0x08, 0xd1, 0xf9, 0x15, 0x9c, 0x40};
//MAC: 08:d1:f9:15:9c:40


//const uint8_t BASE_MAC[] = {0x10, 0x06, 0x1c, 0xa6, 0x18, 0x20}; // This ESP MAC


bool rocketArmed = false;

#pragma pack(push, 1)
struct TelemetryPacket {
  uint16_t record_number;
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
  float velocity;
  uint8_t drogue_pin_state;
  uint8_t main_chute_pin_state;
};
#pragma pack(pop)

void handleBeacon(const wifi_promiscuous_pkt_t* pkt) {
  const uint8_t* payload = pkt->payload;
  int len = pkt->rx_ctrl.sig_len;

  // Basic validation
  if (len < 60 || payload[0] != 0x80) return;
  if (memcmp(&payload[10], rocket_mac, 6) != 0) return;

  // Find telemetry data
  for (int i = 36; i < len - 2; i++) {
    if (payload[i] == 0xDD && payload[i + 1] == sizeof(TelemetryPacket)) {
      TelemetryPacket packet;
      memcpy(&packet, &payload[i + 2], sizeof(packet));
      rocketArmed = true;

      // Print full telemetry as requested
      Serial.println("\n📡 RECEIVED FULL BEACON:");
      Serial.printf("📦 Record #%d | Mode: %d | State: %d\n", 
                   packet.record_number, packet.operation_mode, packet.state);
      Serial.printf("📈 Acc: ax=%.2f ay=%.2f az=%.2f\n", 
                   packet.ax, packet.ay, packet.az);
      Serial.printf("🎯 Orient: pitch=%.2f roll=%.2f\n", 
                   packet.pitch, packet.roll);
      Serial.printf("🌀 Gyro: gx=%.2f gy=%.2f gz=%.2f\n", 
                   packet.gx, packet.gy, packet.gz);
      Serial.printf("📍 Lat: %.4f | Long: %.4f | GPS Alt: %.2f\n", 
                   packet.latitude, packet.longitude, packet.gps_altitude);
      Serial.printf("🛰️ Pressure: %.2f | Temp: %.2f\n", 
                   packet.pressure, packet.temperature);
      Serial.printf("🪂 AGL: %.2f | Velocity: %.2f\n", 
                   packet.altitude_agl, packet.velocity);
      Serial.printf("🔋 Drogue: %d | Main: %d\n", 
                   packet.drogue_pin_state, packet.main_chute_pin_state);
      Serial.printf("📶 RSSI: %d dBm | Channel: %d\n", 
                   pkt->rx_ctrl.rssi, pkt->rx_ctrl.channel);
      break;
    }
  }
}

void promiscuousRx(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type == WIFI_PKT_MGMT) handleBeacon((wifi_promiscuous_pkt_t*)buf);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🔍 Ready to receive full telemetry beacons...");

  // Configure WiFi
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, my_mac);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed");
    ESP.restart();
  }

  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, rocket_mac, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // Setup promiscuous mode
  wifi_promiscuous_filter_t filter = {
    .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT
  };
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_filter(&filter);
  esp_wifi_set_promiscuous_rx_cb(promiscuousRx);
}

void loop() {
  static uint32_t lastArmTime = 0;
  
  if (!rocketArmed && millis() - lastArmTime > 2000) {
    esp_err_t result = esp_now_send(rocket_mac, (uint8_t*)"ARM", 3);
    Serial.println(result == ESP_OK ? "🛜 Sent ARM command" : "❌ ARM send failed");
    lastArmTime = millis();
  }
  delay(10);
}