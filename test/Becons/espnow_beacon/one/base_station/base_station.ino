#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

uint8_t rocket_mac[] = {0xBC, 0xFF, 0x4D, 0xC4, 0x4A, 0xBB};
bool sniffing = false;
unsigned long lastSent = 0;
unsigned long timeoutStart = 0;
bool received_beacon = false;

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

void onPacket(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;

  const wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
  const uint8_t* payload = pkt->payload;
  int len = pkt->rx_ctrl.sig_len;

  Serial.println("📶 Packet detected...");

  for (int i = 36; i < len - 2; i++) {
    if (payload[i] == 0xDD && payload[i + 1] == sizeof(TelemetryPacket)) {
      TelemetryPacket packet;
      memcpy(&packet, &payload[i + 2], sizeof(packet));
      Serial.println("📡 RECEIVED BEACON:");
      Serial.printf("Record #%d | ax=%.2f | lat=%.4f\n", packet.record_number, packet.ax, packet.latitude);
      Serial.printf("RSSI: %d dBm | CH: %d\n\n", pkt->rx_ctrl.rssi, pkt->rx_ctrl.channel);
      received_beacon = true;
      break;
    }
  }
}

void switchToSniffing() {
  Serial.println("🔁 Switching to promiscuous mode...");

  esp_now_deinit();
  WiFi.disconnect(true);
  esp_wifi_stop();  // 🔧 important
  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_wifi_start();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
  esp_wifi_set_promiscuous_filter(&filter);
  esp_wifi_set_promiscuous_rx_cb(&onPacket);
  esp_wifi_set_promiscuous(true);

  Serial.println("👂 Listening for beacons...");
}

void setup() {
  Serial.begin(115200);
  delay(100);

  WiFi.mode(WIFI_STA);
  if (esp_wifi_start() != ESP_OK) {
    Serial.println("❌ Failed to start WiFi");
    return;
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed");
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, rocket_mac, 6);
  peer.channel = 1;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("❌ Failed to add peer");
    return;
  }

  Serial.println("📡 Ready. Sending ARM...");
  lastSent = millis();
  timeoutStart = millis();
}

void loop() {
  if (!sniffing && millis() - lastSent > 1000) {
    const char* cmd = "ARM";
    esp_err_t result = esp_now_send(rocket_mac, (uint8_t*)cmd, strlen(cmd));
    if (result == ESP_OK) {
      Serial.println("✅ Sent ARM command");
    } else {
      Serial.printf("❌ Failed to send ARM: %s\n", esp_err_to_name(result));
    }
    lastSent = millis();
  }

  if (!sniffing && millis() - timeoutStart > 5000) {
    sniffing = true;
    switchToSniffing();
  }
}
