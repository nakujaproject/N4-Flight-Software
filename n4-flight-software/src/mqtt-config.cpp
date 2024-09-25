/*!****************
 * @file mqtt-config.cpp
 * This file implements functions that handle connection to MQTT
 *
 */

#include "mqtt-config.h"

/*!
 *
 * @brief initialize MQTT connection parameters
 * @param broker_IP IP address of the broker being used for telemetry transmission
 * @param broker_port Broker port to use for connection
 * @param topic Topic name on which to publish flight data
 *
 */
MQTTConfig::MQTTConfig(char* broker_IP, int broker_port, char* topic) {
    // copy the passed MQTT parameters to the local variables
    strcpy(mqtt_broker_ip_addr, broker_IP);
    strcpy(mqtt_broker_port, broker_port);
    strcpy(mqtt_topic, topic);

    /* create Wi-Fi Client */
    WiFiClient wifi_client;

    /* create a pub-sub client */
    PubSubClient mqtt_client(wifi_client);

    this->MQTT_Setparameters(mqtt_client);

}

/*!
 * @brief set MQTT connection parameters
 * @param w_client
 * @param mqtt_client
 * @return
 */
uint8_t MQTTConfig::MQTT_Setparameters(PubSubClient mqtt_client) {
    mqtt_client.setServer(this->mqtt_broker_ip_addr, this->mqtt_broker_port);
}

/*!
 * @brief auto reconnect MQTT broker and client if connection is lost
 *
 */
void MQTTConfig::MQTT_Reconnect() {
    // TODO: log to system logger
    while(!)
}