#include "irelays.hpp"
#include "utils/log.hpp"
#include "utils/print.hpp"
#include "utils/array.hpp"

namespace pinicore {

#define PINICORE_TAG_IRELAYS "pcore_irelays"

#define RELAYS_ON_COLOR  "\e[1;42m"
#define RELAYS_OFF_COLOR "\e[1;41m"
#define RELAYS_RST_COLOR "\e[0m"
#define RELAYS_ON_TXT    RELAYS_ON_COLOR  " ON" RELAYS_RST_COLOR
#define RELAYS_OFF_TXT   RELAYS_OFF_COLOR "OFF" RELAYS_RST_COLOR

void IRelays::invalidateAll() {
    for (int i=0; i<p_modules; ++i) {
        resetModuleState(i);
    }
}

bool IRelays::set(uint8_t module, uint8_t relay, bool state) {
    if (module >= p_modules || relay >= p_relaysPerModule)
        return false;

    if (!isModuleConnected(module)) {
        LOG_W(PINICORE_TAG_IRELAYS, "Unable to update module %2d relay %2d to %s, module not found",
            module, relay, state ? RELAYS_ON_TXT : RELAYS_OFF_TXT
        );
        return false;
    }

    bool changedState = setHardware(module, relay, state);
    if (changedState) {
        uint32_t wordIndex, bitIndex;
        bool isValid = calculateRelayIndex(module, relay, &wordIndex, &bitIndex);
        if (!isValid)
            return false;

        if (state) {
            m_relaysState[wordIndex] |= (0x1 << bitIndex);   // set bit
        } else {
            m_relaysState[wordIndex] &= ~(0x1 << bitIndex);  // clear bit
        }

        updateActiveCount();
        _onRelay(module, relay, state);
        
        LOG_I(PINICORE_TAG_IRELAYS, "Updated module %2d relay %2d to %s",
            module, relay, state ? RELAYS_ON_TXT : RELAYS_OFF_TXT
        );
    }
    return changedState;
}

bool IRelays::setAll(bool state) {
    bool anyChangedState = false;
    for (int m=0; m<p_modules; ++m) {
        for (int r=0; r<p_relaysPerModule; ++r) {
            anyChangedState |= set(m, r, state);
        }
    }
    return anyChangedState;
}

bool IRelays::get(uint8_t module, uint8_t relay) {
    if (module >= p_modules || relay >= p_relaysPerModule)
        return false;
    
    uint32_t wordIndex, bitIndex;
    bool isValid = calculateRelayIndex(module, relay, &wordIndex, &bitIndex);
    if (!isValid)
        return false;
    return (m_relaysState[wordIndex] >> bitIndex) & 0x1;
}

const int IRelays::getActiveCount() { return m_relaysActiveCount; }
const int IRelays::getModulesMaxSupported() { return p_modules; }
const int IRelays::getRelaysPerModule() { return p_relaysPerModule; }

bool IRelays::calculateRelayIndex(uint8_t module, uint8_t relay, uint32_t* wordIndex, uint32_t* bitIndex) {
    bool isValid = calculateBitIndex(
        RELAYS_STATE_SIZE_MAX,
        p_relaysPerModule,
        module, relay,
        wordIndex, bitIndex
    );
    return isValid;
}

void IRelays::onRelay(RelaysOnRelayCallback callback) {
    m_onRelayCallback = callback;
}
void IRelays::onModule(RelaysOnModuleCallback callback) {
    m_onModuleCallback = callback;
}


void IRelays::_onRelay(uint8_t module, uint8_t relay, bool state) {
    if (m_onRelayCallback != NULL)
        m_onRelayCallback(module, relay, state);
}
void IRelays::_onModule(uint8_t module, bool state) {
    if (m_onModuleCallback != NULL)
        m_onModuleCallback(module, state);
}

void IRelays::resetModuleState(uint8_t module) {
    uint32_t wordIndex, bitIndex;
    for (int r=0; r<p_relaysPerModule; ++r) {
        bool isValid = calculateRelayIndex(module, r, &wordIndex, &bitIndex);
        if (!isValid)   // Might not be necessary, but just in case
            break;
        m_relaysState[wordIndex] &= ~(0x1 << bitIndex);  // clear bit
        _onRelay(module, r, false);
    }
}


void IRelays::updateActiveCount() {
    m_relaysActiveCount = 0;
    for (int i=0; i<RELAYS_STATE_SIZE_MAX; ++i) {
        // Counts how many bits are '1' in the passed value
        m_relaysActiveCount += __builtin_popcount(m_relaysState[i]);
    }
}

void IRelays::debugRelaysState() {
    char buffer[RELAYS_STORAGE_BIT_SIZE+1];
    for (int i=0; i<RELAYS_STATE_SIZE_MAX; ++i) {
        convertToBinaryString(buffer, m_relaysState[i], RELAYS_STORAGE_BIT_SIZE);
        LOG_D(PINICORE_TAG_IRELAYS, "[%d] -> 0b%s", i, buffer);
    }
}

} // pinicore
