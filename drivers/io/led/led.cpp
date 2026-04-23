#include "led.hpp"
#include <Arduino.h>

namespace pinicore {

/**
 * @brief	Led state mask.
 */
#define LED_STATE_MASK	0x1

#define LED_CHECK_ISVALID           (id>=LED_MAX || id>(m_nLeds-1))
#define LED_GET_STATE(id)           (m_ledsCurrentState & (LED_STATE_MASK << id))
#define LED_SET_STATE(id, state)    m_ledsCurrentState = ((m_ledsCurrentState & ~(LED_STATE_MASK << id)) | ((state & LED_STATE_MASK) << id))

void Led::init(uint8_t* pinLeds, uint8_t size) {
    for (uint8_t i=0; i<size && i<LED_MAX; ++i) {
        m_leds[i] = pinLeds[i];
        ++m_nLeds;

        pinMode(m_leds[i], OUTPUT);
        digitalWrite(m_leds[i], 0);
    }
}

bool Led::getState(uint8_t id) {
    if (LED_CHECK_ISVALID)
        return false;
	return LED_GET_STATE(id);
}

void Led::set(uint8_t id, bool ledState) {
    if (LED_CHECK_ISVALID)
        return;
    uint8_t state = ledState ? LED_STATE_MASK : 0x0;
    digitalWrite(m_leds[id], state);
    LED_SET_STATE(id, state);
}

void Led::toggle(uint8_t id) {
    if (LED_CHECK_ISVALID)
        return;
    uint8_t state = LED_GET_STATE(id) >> id;
    state = state ^ LED_STATE_MASK;
    digitalWrite(m_leds[id], state);
    LED_SET_STATE(id, state);
}

} // pinicore
