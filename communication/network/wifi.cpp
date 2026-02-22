#include "wifi.hpp"
#include "utils/log.hpp"

#define PINICORE_TAG_WIFI   "pcore_wifi"

#define WIFI_AUTORECONNECT_TIMEOUT_MS   (1*60*1000)

void WiFiComm::init() {
    // Currently nothing to do
}

void WiFiComm::config(const char* ssid, const char* pass) {
    strncpy(m_ssid, ssid, sizeof(m_ssid));
    strncpy(m_pass, pass, sizeof(m_pass));
    LOG_D(PINICORE_TAG_WIFI, "Configured WiFi: [SSID: %s]", m_ssid);
}

void WiFiComm::configAP(const char* ssid, const char* pass, bool hidden) {
    strncpy(m_ssidAP, ssid, sizeof(m_ssidAP));
    strncpy(m_passAP, pass, sizeof(m_passAP));
    m_isHiddenAP = hidden;
    LOG_D(PINICORE_TAG_WIFI, "Configured WiFi AP: [SSID: %s]", m_ssidAP);
}

void WiFiComm::maintain() {
    if (isConnected()) {
        m_connectionLost   = false;
        m_connectionLostAt = 0;
        return;
    }

    if (!m_isActive) {
        return;
    }

    uint64_t currMillis = getMillis();
    if (m_connectionLostAt == 0) {
        m_connectionLost   = true;
        m_connectionLostAt = currMillis;
        return;
    }

    if (currMillis - m_connectionLostAt >= WIFI_AUTORECONNECT_TIMEOUT_MS) {
        LOG_W(PINICORE_TAG_WIFI, "Connection lost");
        connect();
        m_connectionLost = false;
        m_connectionLostAt = 0;
    }
}

bool WiFiComm::connect() {
    m_isActive = true;

    if (m_isActiveAP)
        WiFi.mode(WIFI_MODE_APSTA);
    else
        WiFi.mode(WIFI_MODE_STA);
    
    m_isActiveStation = true;
    wl_status_t status = WiFi.begin(m_ssid, m_pass);
    WiFi.setAutoReconnect(true);

    // WiFi.begin turns off the WiFi AP, so reenable it
    if (m_isActiveAP)
        connectAP();
    
    return (status == WL_CONNECTED);
}

void WiFiComm::disconnect() {
    m_isActiveStation = false;
    WiFi.disconnect(false, true);
    m_isActive = false;
}

bool WiFiComm::connectAP() {
    if (m_isActiveStation)
        WiFi.mode(WIFI_MODE_APSTA);
    else
        WiFi.mode(WIFI_MODE_AP);
    
    m_isActiveAP = true;
    return WiFi.softAP(m_ssidAP, m_passAP, 1, m_isHiddenAP);
}

void WiFiComm::disconnectAP() {
    m_isActiveAP = false;
    WiFi.softAPdisconnect(false);
}

uint8_t WiFiComm::stationsConnectedAP() {
    return WiFi.softAPgetStationNum();
}

void WiFiComm::enable() {
    WiFi.begin();
}

void WiFiComm::disable() {
    m_isActiveStation = false;
    m_isActiveAP = false;
    WiFi.disconnect(true);
}