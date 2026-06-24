#include "rl_gpio.hpp"
#include "utils/log.hpp"

namespace pinicore {

#define PINICORE_TAG_RELAYS_GPIO "pcore_relaysgpio"


void RelaysGPIO::init(uint8_t* pinRelays, uint8_t size, bool isActiveLow) {
    m_isActiveLow = isActiveLow;
    m_nRelays = 0;
    for (uint8_t i=0; i<size && i<RELAYS_GPIO_MAX; ++i) {
        m_pinRelays[i] = pinRelays[i];
        ++m_nRelays;

        pinMode(m_pinRelays[i], OUTPUT);
    }

    initModules();
}

bool RelaysGPIO::isModuleConnected(uint8_t module) {
    return (module == 0);
}


void RelaysGPIO::initModules() {
    p_modules = 1;
    p_relaysPerModule = RELAYS_GPIO_MAX;
    resetModuleState(0);
}

bool RelaysGPIO::setHardware(uint8_t module, uint8_t relay, bool state) {
    bool currState = get(module, relay);
    if (currState != state) {
        digitalWrite(m_pinRelays[relay], state);
        return true;
    }
    return false;
}

} // pinicore
