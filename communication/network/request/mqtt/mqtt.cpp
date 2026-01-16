#include "mqtt.hpp"
#include "utils/log.hpp"

#define PINICORE_TAG_MQTT    "pcore_mqtt"
#define PINICORE_TAG_MQTT_CB "pcore_mqtt_cb"

#define MQTT_RECONNECT_TIMER_IN_MILLIS     (15 * 1000)

void MQTT::setClient(Client* client, const char* uniqueId) {
    m_mqttClient.setClient(*client);
    m_mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
    m_mqttClient.setCallback([this](char* topic, byte* payload, unsigned int length) {
        this->receiveCallback(topic, payload, length);
    });
    m_clientId = uniqueId;
}

void MQTT::setServer(const char* domain, uint16_t port) {
    m_mqttClient.setServer(domain, port);
}

void MQTT::setCredentials(const char* username, const char* password) {
    m_username = username;
    m_password = password;
}

void MQTT::setWill(const char* topic, const char* payload, uint8_t qos, bool retain) {
    m_willTopic = topic;
    m_willPayload = payload;
    m_willQos = qos;
    m_willRetain = retain;
}

bool MQTT::connect() {
    bool connected = m_mqttClient.connect(m_clientId, m_username, m_password, m_willTopic, m_willQos, m_willRetain, m_willPayload);
    if (connected) {
        subscribeAll();
        _onConnect();
    }
    return connected;
}

void MQTT::disconnect() {
    m_mqttClient.disconnect();
}

bool MQTT::isConnected() {
    return m_mqttClient.connected();
}

void MQTT::maintain() {
    if (isConnected()) {
        m_mqttClient.loop();
        m_reconnectRetryCount = 0;
        m_wasConnected = true;
        return;
    }
    else if (m_wasConnected) {
        _onDisconnect();
        m_wasConnected = false;
    }
    
    if ((getMillis()-m_timeOfLastTryConnect >= MQTT_RECONNECT_TIMER_IN_MILLIS)) {
        LOG_W(PINICORE_TAG_MQTT, "Reconnecting... Reason: %d", m_mqttClient.state());
        m_mqttClient.disconnect();
        connect();
        ++m_reconnectRetryCount;
        m_timeOfLastTryConnect = getMillis();
    }
}

uint32_t MQTT::getReconnectCount() {
    return m_reconnectRetryCount;
}

void MQTT::onConnect(MqttOnConnectCallback callback) {
    m_onConnectCallback = callback;
}
void MQTT::onDisconnect(MqttOnDisconnectCallback callback) {
    m_onDisconnectCallback = callback;
}
void MQTT::onSubscribe(MqttOnSubscribeCallback callback) {
    m_onSubscribeCallback = callback;
}
void MQTT::onUnsubscribe(MqttOnUnsubscribeCallback callback) {
    m_onUnsubscribeCallback = callback;
}
bool MQTT::onTopic(const char* topic, MqttOnTopicCallback callback) {
    for (int i=0; i<MQTT_SUBSCRIBE_SIZE_MAX; ++i) {
        MqttOnTopicCallback_t* onTopic = &m_onTopicCallbacks[i];
        if (onTopic->topic == NULL) {
            onTopic->topic    = topic;
            onTopic->callback = callback;
            return true;
        }
        if (strcmp(onTopic->topic, topic) == 0) {
            onTopic->callback = callback;
            return true;
        }
    }
    return false;
}

void MQTT::removeOnTopic(const char* topic) {
    for (int i=0; i<MQTT_SUBSCRIBE_SIZE_MAX; ++i) {
        MqttOnTopicCallback_t* onTopic = &m_onTopicCallbacks[i];
        if (strcmp(onTopic->topic, topic) == 0) {
            m_mqttClient.unsubscribe(topic);
            _onUnsubscribe(topic);
            onTopic->topic = NULL;
            return;
        }
    }
}

void MQTT::publish(const char* topic, const char* payload, const bool retain) {
    LOG_D(PINICORE_TAG_MQTT, "Publish: [topic: %s] [payload: %s] [retain: %s]", topic, payload, retain?"true":"false");
    m_mqttClient.publish(topic, payload, retain);
}


void MQTT::receiveCallback(char* topic, uint8_t* payload, unsigned int length) {
    strncpy(m_buffer, (char*)payload, length);
    m_buffer[length] = '\0';    // I would like to find a better solution for this
    LOG_D(PINICORE_TAG_MQTT_CB, "Received: [topic: %s] [payload: %s] [length: %d]", topic, m_buffer, length);
    
    for (int i=0; i<MQTT_SUBSCRIBE_SIZE_MAX; ++i) {
        MqttOnTopicCallback_t* onTopic = &m_onTopicCallbacks[i];
        if (strcmp(onTopic->topic, topic) == 0) {
            onTopic->callback(m_buffer, length);
            break;
        }
    }
}

void MQTT::subscribeAll() {
    for (int i=0; i<MQTT_SUBSCRIBE_SIZE_MAX; ++i) {
        MqttOnTopicCallback_t* onTopic = &m_onTopicCallbacks[i];
        if (onTopic->topic != NULL) {
            m_mqttClient.subscribe(onTopic->topic);
            _onSubscribe(onTopic->topic);
        }
    }
}

void MQTT::_onConnect() {
    if (m_onConnectCallback != NULL)
        m_onConnectCallback();
}
void MQTT::_onDisconnect() {
    if (m_onDisconnectCallback != NULL)
        m_onDisconnectCallback();
}
void MQTT::_onSubscribe(const char* topic) {
    if (m_onSubscribeCallback != NULL)
        m_onSubscribeCallback(topic);
}
void MQTT::_onUnsubscribe(const char* topic) {
    if (m_onUnsubscribeCallback != NULL)
        m_onUnsubscribeCallback(topic);
}