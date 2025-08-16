#include <WiFi.h>
#include "esp_wifi.h"

struct __attribute__((packed)) TelemetryPacket {
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

TelemetryPacket packet = {
  .record_number = 42,
  .operation_mode = 1,
  .state = 3,
  .ax = 0.98, .ay = -0.12, .az = 9.81,
  .pitch = 5.4, .roll = 2.1,
  .gx = 0.01, .gy = -0.02, .gz = 0.03,
  .latitude = -1.2833, .longitude = 36.8167,
  .gps_altitude = 1532.0,
  .pressure = 1013.25,
  .temperature = 26.7,
  .altitude_agl = 1450.0,
  .velocity = 123.45,
  .drogue_pin_state = 1,
  .main_chute_pin_state = 0
};

uint8_t my_mac[6] = {0x08, 0xD1, 0xF9, 0x15, 0x9C, 0x04};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🚀 Full Telemetry Beacon Transmitter Starting...");

  WiFi.mode(WIFI_MODE_AP);
  esp_wifi_set_ps(WIFI_PS_NONE);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_AP);
  esp_wifi_start();

  Serial.printf("📡 Transmitter MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5]);
}

void loop() {
  uint8_t beacon[] = {
    0x80, 0x00, 0x00, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   // Destination
    0, 0, 0, 0, 0, 0,                     // Source MAC placeholder
    0, 0, 0, 0, 0, 0,                     // BSSID placeholder
    0x00, 0x00,

    // Fixed params
    0,0,0,0,0,0,0,0,
    0x64, 0x00,
    0x01, 0x04,

    // SSID
    0x00, 0x04, 'T','E','L','M',
    0x01, 0x08, 0x82, 0x84, 0x8B, 0x96, 0x24, 0x30, 0x48, 0x6C,
    0x03, 0x01, 0x01
  };

  memcpy(&beacon[10], my_mac, 6);
  memcpy(&beacon[16], my_mac, 6);

  size_t base_len = sizeof(beacon);
  size_t full_len = base_len + 2 + sizeof(packet);
  uint8_t* full_frame = (uint8_t*)malloc(full_len);

  memcpy(full_frame, beacon, base_len);
  full_frame[base_len] = 0xDD;
  full_frame[base_len + 1] = sizeof(packet);
  memcpy(&full_frame[base_len + 2], &packet, sizeof(packet));

  esp_err_t result = esp_wifi_80211_tx(WIFI_IF_AP, full_frame, full_len, false);
  if (result != ESP_OK) {
    Serial.printf("❌ TX Failed: %s\n", esp_err_to_name(result));
  } else {
    Serial.printf("✅ Sent beacon: Record#%d | Mode: %d | State: %d | Lat: %.4f | Alt: %.2f\n",
                  packet.record_number, packet.operation_mode, packet.state, packet.latitude, packet.gps_altitude);
  }

  free(full_frame);
  delay(1000);
}
