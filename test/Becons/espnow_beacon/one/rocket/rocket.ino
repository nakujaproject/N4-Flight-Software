extern "C" {
  #include <user_interface.h>
}
#include <ESP8266WiFi.h>
#include <espnow.h>

bool armed = false;
uint8_t mac[6];

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

void sendBeaconWithTelemetry() {
  uint8_t beacon[] = {
    0x80, 0x00, 0x00, 0x00,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0x00, 0x00,
    0,0,0,0,0,0,0,0,
    0x64, 0x00,
    0x31, 0x04,
    0x00, 0x04, 'T','E','L','M',
    0x01, 0x08, 0x82, 0x84, 0x8B, 0x96, 0x24, 0x30, 0x48, 0x6C,
    0x03, 0x01, 0x01
  };

  memcpy(&beacon[10], mac, 6);
  memcpy(&beacon[16], mac, 6);

  size_t base_len = sizeof(beacon);
  size_t total_len = base_len + 2 + sizeof(packet);
  uint8_t* frame = (uint8_t*)malloc(total_len);
  memcpy(frame, beacon, base_len);
  frame[base_len] = 0xDD;
  frame[base_len + 1] = sizeof(packet);
  memcpy(&frame[base_len + 2], &packet, sizeof(packet));

  wifi_send_pkt_freedom(frame, total_len, false);
  free(frame);

  Serial.printf("📡 Beacon sent: #%d | ax=%.2f\n", packet.record_number, packet.ax);
}

void onDataRecv(uint8_t *mac_addr, uint8_t *data, uint8_t len) {
  if (!armed && strncmp((char*)data, "ARM", 3) == 0) {
    armed = true;
    Serial.println("🛡️ ARM command received — entering beacon mode.");
  }
}

void setup() {
  Serial.begin(74880);
  delay(100);
  WiFi.mode(WIFI_STA);
  wifi_get_macaddr(STATION_IF, mac);

  if (esp_now_init() != 0) {
    Serial.println("❌ ESP-NOW init failed");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(onDataRecv);
  wifi_set_channel(1);

  Serial.printf("🛰️ Rocket ready — MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void loop() {
  if (armed) {
    sendBeaconWithTelemetry();
    delay(1000);
  }
}
