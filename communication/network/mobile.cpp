#include "mobile.hpp"
#include "utils/log.hpp"

namespace pinicore {

#define PINICORE_TAG_MOBILE   "pcore_mobile"

#define GPRS_USER "" // empty -> auto
#define GRPS_PASS "" // empty -> auto

void MobileComm::init(uint8_t pinPowerOn, uint8_t pinPowerKey, uint8_t pinReset, uint8_t pinTx, uint8_t pinRx) {
    p_pinPowerOn  = pinPowerOn;
    p_pinPowerKey = pinPowerKey;
    p_pinReset    = pinReset;
    p_pinTx       = pinTx;
    p_pinRx       = pinRx;
    SerialAT.begin(115200, SERIAL_8N1, p_pinRx, p_pinTx);
    pinMode(p_pinPowerKey, OUTPUT);
    pinMode(p_pinReset, OUTPUT);
    pinMode(p_pinPowerOn, OUTPUT);
}

void MobileComm::config(const char* apn, const char* simcardPin) {
    strncpy(m_apn, apn, sizeof(m_apn));
    strncpy(m_simcardPin, simcardPin, sizeof(m_simcardPin));
    if (m_apn[0]==0)
        LOG_D(PINICORE_TAG_MOBILE, "Configured Mobile: [APN auto search]");
    else
        LOG_D(PINICORE_TAG_MOBILE, "Configured Mobile: [APN: %s]", m_apn);
}

void MobileComm::maintain() {
    m_modem.maintain();
    
    if (!m_modem.isGprsConnected() && m_isActive) {
        LOG_W(PINICORE_TAG_MOBILE, "Connection lost");
        connect();
    }
}

bool MobileComm::connect() {
    m_isActive = true;
    m_provider[0] = '\0'; // Clear provider network name since no longer connected

    LOG_D(PINICORE_TAG_MOBILE, "Starting modem");
    if (!m_modem.restart()) {
        LOG_E(PINICORE_TAG_MOBILE, "Unable to restart Mobile module! Possible problem with hardware.");
        return false;
    }

    // Unlock your SIM card with a PIN is needed
    if (m_modem.getSimStatus()==SIM_LOCKED && m_simcardPin[0]!=0) {
        m_modem.simUnlock(m_simcardPin);
    }

    bool gprsConnected = m_modem.gprsConnect(m_apn, GPRS_USER, GRPS_PASS);
    if (!gprsConnected) {
        LOG_W(PINICORE_TAG_MOBILE, "Unable to connect. Check if Mobile antenna and SIM card are connected correctly.");
        return false;
    }
    else {
        strncpy(m_provider, m_modem.getProvider().c_str(), sizeof(m_provider)); // Set provider network name
        LOG_I(PINICORE_TAG_MOBILE, "Connected to '%s'", m_provider);
        p_connectedOnce = true;
    }
    return true;
}

void MobileComm::disconnect() {
    m_modem.gprsDisconnect();
    m_isActive = false;
}

void MobileComm::enable() {
    digitalWrite(p_pinReset,    HIGH);
    digitalWrite(p_pinPowerOn,  HIGH);
    digitalWrite(p_pinPowerKey, LOW);
    delay(1000);
    digitalWrite(p_pinPowerKey, HIGH);
}

void MobileComm::disable() {
    digitalWrite(p_pinPowerKey, LOW);
    delay(1000);
    digitalWrite(p_pinPowerKey, HIGH);
    digitalWrite(p_pinPowerOn,  LOW);
}

} // pinicore
