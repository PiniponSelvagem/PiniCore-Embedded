#include "lm35.hpp"
#include <Arduino.h>

void LM35::init(uint8_t pin, float offset) {
    m_pin = pin;
    m_offset = offset;
    pinMode(m_pin, INPUT_PULLUP);
}

float LM35::readTemperature() {
    uint32_t raw = analogReadMilliVolts(m_pin);
    return (raw / 10.f) + m_offset;
}
