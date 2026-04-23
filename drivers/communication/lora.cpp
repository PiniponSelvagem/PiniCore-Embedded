#include "lora.hpp"
#include "utils/log.hpp"

#include <LoRa.h>
#include <SPI.h>

namespace pinicore {

#define PINICORE_TAG_LORA   "pcore_lora"

bool LoRaTxRx::init(
    uint8_t pinMOSI, uint8_t pinMISO, uint8_t pinSCLK, uint8_t pinCS,
    uint8_t pinReset, uint8_t pinDIO0,
    uint16_t carrierFrequency
) {
    SPI.begin(pinSCLK, pinMISO, pinMOSI, pinCS);
    LoRa.setPins(pinCS, pinReset, pinDIO0);
    if (!LoRa.begin(carrierFrequency*1E6)) {
        LOG_E(PINICORE_TAG_LORA, "Unable to initialize LoRa hardware, check if defined pins and module is installed correctly");
        return false;
    }
    enable();
    setSpreadingFactor(LORA_INIT_DEFAULT_SF);
    setTxPower(LORA_INIT_DEFAULT_POWER);
    setBandwidth(LORA_INIT_DEFAULT_BAND);
    return true;
}

void LoRaTxRx::setSpreadingFactor(uint8_t sf) {
    if (sf < 6) {
        sf = 6;
    } else if (sf > 12) {
        sf = 12;
    }
    m_spreadingFactor = sf;
    LoRa.setSpreadingFactor(sf);
}

void LoRaTxRx::setTxPower(uint8_t power) {
    if (power > 20) {
        power = 20;
    }
    //
    int outputPin;
    if (power < 15) {
        outputPin = PA_OUTPUT_RFO_PIN;
    }
    else {
        outputPin = PA_OUTPUT_PA_BOOST_PIN;
    }
    m_txPower = power;
    LoRa.setTxPower(power, outputPin);
}

void LoRaTxRx::setBandwidth(ELoRaBandwidth bandwidth) {
    m_bandwidth = bandwidth;
    LoRa.setSignalBandwidth((long)bandwidth);
}

void LoRaTxRx::maintain() {
    if (!isEnabled()) return;

    if (!receive()) return; // check if new packet received, otherwise return

    LOG_T(PINICORE_TAG_LORA, "Processing received payload, %lu bytes", m_packetReceived.size);
    _onReceive(m_packetReceived.payload, m_packetReceived.size, m_packetReceived.rssi, m_packetReceived.snr);
}

void LoRaTxRx::enable() {
    LoRa.idle();
    m_isActive = true;
}

void LoRaTxRx::disable() {
    LoRa.sleep();
    m_isActive = false;
}

void LoRaTxRx::send(const uint8_t* payload, size_t size) {
    size_t safeSize = (size>LORA_PACKET_MAX_SIZE) ? LORA_PACKET_MAX_SIZE : size;
    LOG_T(PINICORE_TAG_LORA, "Preparing to send %lu bytes", safeSize);
    LoRa.beginPacket();
    LoRa.write(payload, safeSize);
    LoRa.endPacket();
    LoRa.receive();
    LOG_T(PINICORE_TAG_LORA, "Sent %lu bytes", safeSize);

    /* Statistics */
    m_statsBytesSent += safeSize;
    ++m_statsPacketsSent;    
}

void LoRaTxRx::onReceive(LoRaTxRxOnReceiveCallback callback) {
    m_onReceiveCallback = callback;
}


bool LoRaTxRx::receive() {
    size_t size = LoRa.parsePacket();
    if (size <= 0) { return false; }

    /* Statistics */
    m_statsBytesReceived += size;
    ++m_statsPacketsReceived;

    int rssi  = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    m_packetReceived.size = (size>LORA_PACKET_MAX_SIZE) ? LORA_PACKET_MAX_SIZE : size;
    m_packetReceived.rssi = rssi;
    m_packetReceived.snr  = snr;

    for (size_t i=0; i<size && LoRa.available(); ++i) {
        if (i < m_packetReceived.size) {
            m_packetReceived.payload[i] = LoRa.read();
        }
        else {
            LoRa.read(); // discard the rest since it does not fit in packet buffer
        }
    }
    return true;
}

void LoRaTxRx::_onReceive(const uint8_t* payload, size_t size, int rssi, float snr) {
    if (m_onReceiveCallback != NULL)
        m_onReceiveCallback(payload, size, rssi, snr);
}

} // pinicore
