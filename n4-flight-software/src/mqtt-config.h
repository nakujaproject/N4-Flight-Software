/*!**********************
 * @file mqtt-config.h
 * Handle connection to MQTT broker and data transmission over MQTT
 *  *
 */


#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "defs.h"
#include "string.h"

class MQTTConfig {
    /* create Wi-Fi Client */
    WiFiClient wifi_client;

    /* create a pub-sub client */
    PubSubClient mqtt_client(wifi_client);

    public:
        char mqtt_broker_ip_addr[BROKER_IP_ADDRESS_LENGTH];
        int16_t mqtt_broker_port;
        char mqtt_topic[MQTT_TOPIC_LENGTH];

        MQTTConfig(char* broker_IP, int16_t port, char* topic);
        uint8_t MQTT_Setparameters(PubSubClient);
        void MQTT_Reconnect();
};

#endif
