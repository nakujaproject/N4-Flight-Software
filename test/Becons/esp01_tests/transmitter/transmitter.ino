extern "C" {
  #include <user_interface.h>
}

#include <ESP8266WiFi.h>

// Same struct (keep small!)
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

uint8_t mac[6];

void sendBeaconWithTelemetry() {
  uint8_t beacon[] = {
    0x80, 0x00, 0x00, 0x00,                     // beacon frame
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,         // dest = broadcast
    0, 0, 0, 0, 0, 0,                           // src mac
    0, 0, 0, 0, 0, 0,                           // bssid
    0x00, 0x00,                                 // seq

    // Fixed params
    0,0,0,0,0,0,0,0,
    0x64, 0x00,
    0x31, 0x04,

    // SSID
    0x00, 0x04, 'T','E','L','M',
    0x01, 0x08, 0x82, 0x84, 0x8B, 0x96, 0x24, 0x30, 0x48, 0x6C,
    0x03, 0x01, 0x01
  };

  memcpy(&beacon[10], mac, 6);  // src MAC
  memcpy(&beacon[16], mac, 6);  // bssid MAC

  size_t base_len = sizeof(beacon);
  size_t total_len = base_len + 2 + sizeof(TelemetryPacket);
  uint8_t* frame = (uint8_t*)malloc(total_len);
  memcpy(frame, beacon, base_len);

  frame[base_len] = 0xDD; // Vendor-specific tag
  frame[base_len + 1] = sizeof(TelemetryPacket);
  memcpy(&frame[base_len + 2], &packet, sizeof(packet));

  // Send the raw beacon
  wifi_send_pkt_freedom(frame, total_len, false);
  free(frame);

  Serial.printf("✅ Sent beacon: Record#%d | Mode: %d | Lat: %.4f\n",
                packet.record_number, packet.operation_mode, packet.latitude);
}

void setup() {
  Serial.begin(74880); // Safe default baud for ESP8266-01 boot mode
  delay(500);

  WiFi.mode(WIFI_OFF);
  wifi_set_opmode(STATION_MODE); // Must be in STATION mode to send raw

  wifi_get_macaddr(STATION_IF, mac);
  Serial.printf("📡 MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  wifi_promiscuous_enable(0); // Required to enable raw tx
  wifi_set_channel(1);
}

void loop() {
  sendBeaconWithTelemetry();
  delay(1000);
}
