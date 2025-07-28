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

const uint8_t transmitter_mac[6] = {0x08, 0xD1, 0xF9, 0x15, 0x9C, 0x04};

void onPacket(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;

  const wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
  const uint8_t* payload = pkt->payload;
  int len = pkt->rx_ctrl.sig_len;

  const uint8_t* src_mac = &payload[10];
  if (memcmp(src_mac, transmitter_mac, 6) != 0) return;

  for (int i = 36; i < len - 2; i++) {
    if (payload[i] == 0xDD && payload[i + 1] == sizeof(TelemetryPacket)) {
      TelemetryPacket packet;
      memcpy(&packet, &payload[i + 2], sizeof(packet));

      Serial.println("📡 RECEIVED FULL BEACON:");
      Serial.printf("📦 Record #%d | Mode: %d | State: %d\n", packet.record_number, packet.operation_mode, packet.state);
      Serial.printf("📈 Acc: ax=%.2f ay=%.2f az=%.2f\n", packet.ax, packet.ay, packet.az);
      Serial.printf("🎯 Orient: pitch=%.2f roll=%.2f\n", packet.pitch, packet.roll);
      Serial.printf("🌀 Gyro: gx=%.2f gy=%.2f gz=%.2f\n", packet.gx, packet.gy, packet.gz);
      Serial.printf("📍 Lat: %.4f | Long: %.4f | GPS Alt: %.2f\n", packet.latitude, packet.longitude, packet.gps_altitude);
      Serial.printf("🛰️ Pressure: %.2f | Temp: %.2f\n", packet.pressure, packet.temperature);
      Serial.printf("🪂 AGL: %.2f | Velocity: %.2f\n", packet.altitude_agl, packet.velocity);
      Serial.printf("🔋 Drogue: %d | Main: %d\n", packet.drogue_pin_state, packet.main_chute_pin_state);
      Serial.printf("📶 RSSI: %d dBm | Channel: %d\n\n", pkt->rx_ctrl.rssi, pkt->rx_ctrl.channel);
      break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_MODE_NULL);
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_wifi_start();

  wifi_promiscuous_filter_t filter = {
    .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT
  };
  esp_wifi_set_promiscuous_filter(&filter);
  esp_wifi_set_promiscuous_rx_cb(&onPacket);
  esp_wifi_set_promiscuous(true);

  Serial.println("🔍 Ready to receive full telemetry beacons...");
}

void loop() {
  delay(500); // Avoid watchdog reset
}
