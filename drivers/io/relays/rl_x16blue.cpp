#include "rl_x16blue.hpp"
#include "utils/log.hpp"

namespace pinicore {

#define PINICORE_TAG_RELAYS_X16BLUE "pcore_relaysx16blue"


void RelaysX16Blue::init(uint8_t pinEnable, uint8_t pinLatch, uint8_t pinClock, uint8_t pinData, bool isActiveLow) {
    m_pinLatch    = pinLatch;
    m_pinClock    = pinClock;
    m_pinData     = pinData;
    m_isActiveLow = isActiveLow;

    pinMode(pinEnable, OUTPUT);
    pinMode(m_pinLatch, OUTPUT);
    pinMode(m_pinClock, OUTPUT);
    pinMode(m_pinData, OUTPUT);

    digitalWrite(pinEnable, LOW);   // active low, enable it and forget about it
    
    initModules();
}

bool RelaysX16Blue::isModuleConnected(uint8_t module) {
    return (module == 0);
}


void RelaysX16Blue::initModules() {
    p_modules = 1;
    p_relaysPerModule = 16;
    resetModuleState(0);
}

bool RelaysX16Blue::setHardware(uint8_t module, uint8_t relay, bool state) {
    uint16_t stateBeforeSet = 0;
    for (int r=0; r<p_relaysPerModule; ++r) {
        uint8_t relayState = get(0, r) ? 0x1 : 0x0;
        stateBeforeSet |= (relayState << r);
    }

    // Preparation
    uint8_t bitState = state ? 1 : 0;   
    uint16_t requestedState = stateBeforeSet;
    requestedState &= ~(0x1 << relay);
    requestedState |= (bitState << relay);

    // Invert requestedState if board works active LOW
    requestedState = m_isActiveLow ? ~requestedState : requestedState;

    digitalWrite(m_pinLatch, LOW);
    shiftOut(m_pinData, m_pinClock, MSBFIRST, (requestedState >> 8));
    shiftOut(m_pinData, m_pinClock, MSBFIRST, requestedState);
    digitalWrite(m_pinLatch, HIGH);
    return true;
}

} // pinicore
