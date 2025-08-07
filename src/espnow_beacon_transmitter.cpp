#include "espnow_beacon_transmitter.h"
#include "defs.h"  // Include defs.h to access TEST flag
#include <WiFi.h>  // Include WiFi for mode checking

ESPNowBeaconTransmitter* ESPNowBeaconTransmitter::instance = nullptr;

ESPNowBeaconTransmitter::ESPNowBeaconTransmitter(const uint8_t* rocket_mac, const uint8_t* base_mac) {
    memcpy(rocketMAC, rocket_mac, 6);
    memcpy(baseMAC, base_mac, 6);
    instance = this;
    commandQueue = xQueueCreate(10, sizeof(CommandPacket));
}

ESPNowBeaconTransmitter::~ESPNowBeaconTransmitter() {
    if (commandQueue) vQueueDelete(commandQueue);
}

bool ESPNowBeaconTransmitter::begin() {
    // Safety check: Ensure WiFi is available before initializing ESP-NOW
    if (WiFi.getMode() == WIFI_MODE_NULL) {
        Serial.println("❌ WiFi not initialized - cannot start ESP-NOW");
        return false;
    }
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ ESP-NOW init failed");
        return false;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, baseMAC, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("❌ Peer add failed");
        return false;
    }

    if (esp_now_register_recv_cb(OnDataRecv) != ESP_OK) {
        Serial.println("❌ Callback register failed");
        return false;
    }

    Serial.println("🚀 ESP-NOW Beacon Transmitter Ready");
    return true;
}


void ESPNowBeaconTransmitter::OnDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
    if (!instance || len > MAX_COMMAND_LENGTH) return;
    if (memcmp(mac, instance->baseMAC, 6) != 0) return;

    CommandPacket packet;
    packet.timestamp = millis();
    packet.length = len;
    memcpy(packet.command, data, len);

    if (xQueueSend(instance->commandQueue, &packet, 0) != pdTRUE) {
        Serial.println("⚠️ Command queue full!");
    }
}

bool ESPNowBeaconTransmitter::getNextCommand(CommandPacket* packet) {
    return commandQueue && xQueueReceive(commandQueue, packet, 0) == pdTRUE;
}

bool ESPNowBeaconTransmitter::sendBeacon(const void* telemetry_data, size_t telemetry_size) {
    // Only send beacon if armed OR if in test mode (similar to MQTT logic)
    if ((!armed && !TEST) || !telemetry_data || telemetry_size == 0) return false;

    uint8_t frame[MAX_BEACON_SIZE];
    size_t frame_size = 0;
    buildBeaconFrame(frame, &frame_size, telemetry_data, telemetry_size);

    if (frame_size == 0 || esp_wifi_80211_tx(WIFI_IF_AP, frame, frame_size, false) != ESP_OK) {
        return false;
    }
    Serial.printf("✅ Beacon sent #%lu | Size: %d bytes\n", beaconCounter, frame_size);
    beaconCounter++;
    if (telemetryCallback) telemetryCallback((void*)telemetry_data, telemetry_size);
    return true;
}

void ESPNowBeaconTransmitter::buildBeaconFrame(uint8_t* frame, size_t* frame_size, 
                                                const void* telemetry_data, size_t telemetry_size) {
    if (telemetry_size + 60 > MAX_BEACON_SIZE) {  // safety check
        *frame_size = 0;
        return;
    }

    // Base beacon frame (same structure as working example)
    const uint8_t beacon_template[] = {
        0x80, 0x00, 0x00, 0x00,                            // Frame Control, Duration
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,                // Destination (broadcast)
        0, 0, 0, 0, 0, 0,                                  // Source MAC (to be filled)
        0, 0, 0, 0, 0, 0,                                  // BSSID (to be filled)
        0x00, 0x00,                                        // Sequence Control
        0,0,0,0,0,0,0,0,                                   // Timestamp
        0x64, 0x00,                                        // Beacon interval
        0x31, 0x04,                                        // Capability info
        0x00, 0x04, 'T','E','L','M',                       // SSID
        0x01, 0x08, 0x82, 0x84, 0x8B, 0x96, 0x24, 0x30, 0x48, 0x6C, // Supported rates
        0x03, 0x01, 0x01                                   // DS Parameter Set (Channel 1)
    };

    size_t header_len = sizeof(beacon_template);
    memcpy(frame, beacon_template, header_len);

    // Fill MAC addresses
    memcpy(frame + 10, rocketMAC, 6);  // Source
    memcpy(frame + 16, rocketMAC, 6);  // BSSID

    // Insert vendor-specific tag + telemetry payload
    frame[header_len] = 0xDD;                      // Vendor-specific tag
    frame[header_len + 1] = telemetry_size;        // Length
    memcpy(&frame[header_len + 2], telemetry_data, telemetry_size);

    *frame_size = header_len + 2 + telemetry_size;
}
