/**
* @file		mqtt.hpp
* @brief	MQTT API made easy.
* @author	PiniponSelvagem
*
* Copyright(C) PiniponSelvagem
*
***********************************************************************
* Software that is described here, is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
**********************************************************************/

#pragma once

#ifndef _PINICORE_MQTT_H_
#define _PINICORE_MQTT_H_

#include <stdint.h>
#include <functional>
#include <Client.h>
#include <PubSubClient.h>

#define MQTT_BUFFER_SIZE 1024
#define MQTT_SUBSCRIBE_SIZE_MAX 64 // Maximum number of topics that can be subscribed. I want to avoid using 'malloc'.

// user callbacks
typedef std::function<void(void)> MqttOnConnectCallback;                    // Callback for on connect/reconnect.
typedef std::function<void(void)> MqttOnDisconnectCallback;                 // Callback for on disconnect.
typedef std::function<void(const char* topic)> MqttOnSubscribeCallback;     // Callback for on subscribe to a topic.
typedef std::function<void(const char* topic)> MqttOnUnsubscribeCallback;   // Callback for on unsubscribe from a topic.
typedef std::function<void(const char* payload, uint32_t length)> MqttOnTopicCallback;  // Callback for subscribed topic.

typedef struct {
    const char *topic;
    MqttOnTopicCallback callback;
} MqttOnTopicCallback_t;


class MQTT {
    public:
        /**
         * @brief   Set network client and uniqueId to be used in the MQTT service.
         * @param	client Pointer to network client to be used for MQTT.
         * @param   uniqueId Controller unique identifier in the MQTT service.
         */
        void setClient(Client* client, const char* uniqueId);

        /**
         * @brief   Configure to which server connect when \ref 'connect' is called.
         * @param   domain MQTT server IP or domain name.
         * @param   port MQTT server port.
         */
        void setServer(const char* domain, uint16_t port);

        /**
         * @brief   Configure credentials to be used for connection.
         * @param   username MQTT username.
         * @param   password MQTT password.
         */
        void setCredentials(const char* username, const char* password);

        /**
         * @brief   Configure a payload for a topic to be set on disconnect / connection lost.
         * @param   topic The topic.
         * @param   payload The payload to be set for the topic.
         * @param   qos Quality of service level.
         * @param   retain Should the message be retained on MQTT server.
         */
        void setWill(const char* topic, const char* payload, uint8_t qos, bool retain);

        /**
         * @brief   Connects to the assigned MQTT service.
         * @return  True if was able to connect, false otherwise.
         * @warning These functions must be called first: \ref 'setClient', \ref 'setServer', \ref 'setCredentials', \ref 'setWill'.
         */
        bool connect();

        /**
         * @brief   Disconnects from the assigned MQTT service.
         */
        void disconnect();

        /**
         * @brief   Checks if the MQTT client is connected.
         * @return  True if the client is connected, false otherwise.
         */
        bool isConnected();

        /**
         * @brief   Handle and mantain the MQTT connection.
         * @note    Should be called often, once per main loop.
         */
        void maintain();

        /**
         * @brief   Get the number of connection retries so far.
         * @return  0 when connected, \ref 'connect' not called or \ref 'disconnect' called.
         *          Returns the retry count if \ref 'connect' was called and then lost connection.
         */
        uint32_t getReconnectCount();

        /**
         * @brief   Registers a callback function to be called when the MQTT client is connected/reconnected.
         * @param   callback The callback function with the signature void(void) to be registered.
         */
        void onConnect(MqttOnConnectCallback callback);
        /**
         * @brief   Registers a callback function to be called when the MQTT client is disconnected.
         * @param   callback The callback function with the signature void(void) to be registered.
         */
        void onDisconnect(MqttOnDisconnectCallback callback);
        /**
         * @brief   Registers a callback function to be called when the MQTT client subscribes to a topic.
         * @param   callback The callback function with the signature void(const char* topic) to be registered.
         */
        void onSubscribe(MqttOnSubscribeCallback callback);
        /**
         * @brief   Registers a callback function to be called when the MQTT client unsubscribes to a topic.
         * @param   callback The callback function with the signature void(const char* topic) to be registered.
         */
        void onUnsubscribe(MqttOnUnsubscribeCallback callback);
        /**
         * @brief   Registers a callback function to be called when the MQTT client receives a sepecific topic.
         * @param   topic The topic.
         * @param   callback The callback function with the signature void(const char* payload, const uint32_t length) to be registered.
         * @return  True if there was space in the internal 'm_onTopicCallbacks' array to add the topic, false if full and could not be added.
         * @note    Calling this function for same topic will replace old callback. Topic will be subscribed on connect.
         */
        bool onTopic(const char* topic, MqttOnTopicCallback callback);

        /**
         * @brief   Unregisters a callback function from being called when the MQTT client receives a sepecific topic.
         * @param   topic The topic.
         * @note    Will unsubscribe if the topic is currently subscribed.
         */
        void removeOnTopic(const char* topic);

        /**
         * @brief   Send payload to a topic.
         * @param   topic Index of the topic in the 'm_topicPublish' data structure to be used to publish the payload
         * @param   payload Payload to be sent, null terminated
         * @param   retain True and will tell broker to retain the message.
         */
        void publish(const char* topic, const char* payload, const bool retain);


    private:
        /**
         * @brief   Callback that receives the subscribed topics.
         * @param   topic The topic.
         * @param   payload The payload.
         * @param   length Size of the payload.
         */
        void receiveCallback(char* topic, uint8_t* payload, unsigned int length);

        /**
         * @brief   Subscribe to all topics that were added in \ref 'onTopic'.
         */
        void subscribeAll();
        
        /**
         * @brief   Safely call 'onConnect' callback.
         */
        void _onConnect();
        /**
         * @brief   Safely call 'onDisconnect' callback.
         */
        void _onDisconnect();
        /**
         * @brief   Safely call 'onSubscribe' callback.
         * @param   topic The topic.
         */
        void _onSubscribe(const char* topic);
        /**
         * @brief   Safely call 'onSubscribe' callback.
         * @param   topic The topic.
         */
        void _onUnsubscribe(const char* topic);


        PubSubClient m_mqttClient;
        const char* m_clientId;
        char m_buffer[MQTT_BUFFER_SIZE+1];  // Buffer used to place a null terminated payload since PubSubClient does not null terminate it.

        const char* m_username;
        const char* m_password;

        uint64_t m_timeOfLastTryConnect = 0;
        uint32_t m_reconnectRetryCount  = 0;
        bool m_wasConnected = false; // This is used to call 'onDisconnect' callback only once per disconnect

        const char* m_willTopic;
        const char* m_willPayload;
        uint8_t m_willQos;
        bool m_willRetain;

        /** Callbacks **/
        MqttOnConnectCallback     m_onConnectCallback     = NULL;
        MqttOnDisconnectCallback  m_onDisconnectCallback  = NULL;
        MqttOnSubscribeCallback   m_onSubscribeCallback   = NULL;
        MqttOnUnsubscribeCallback m_onUnsubscribeCallback = NULL;
        MqttOnTopicCallback_t     m_onTopicCallbacks[MQTT_SUBSCRIBE_SIZE_MAX] = {};
};

#endif // _PINICORE_MQTT_H_