#include "rotaryencoder.hpp"
#include <Arduino.h>

namespace pinicore {

// The array holds the values -1 for the entries where a position was decremented,
// a 1 for the entries where the position was incremented
// and 0 in all the other (no change or not valid) cases.
const int8_t KNOBDIR[] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0};

// Configuration for different types of rotary encoders.
// For more information, refer to http://svglobe.com/arduino/in_encoder.html
//
// The detents in the encoder type settings below are indexed
// by value, not by the order in the quadrature cycle. For example, a rotary
// encoder with detents at 00 and 11 (positions 0 and 2 in the
// quadrature) but are indexed based on their decimal values (0 and 3).
// This allows simple lookup of detent positions by value.
const encoderType encoderTypes[] {
    /**
     * DO NOT CHANGE THE ORDER OF THESE WITHOUT
     * REORDERING THE RotaryEncoderType!!! 
     */

    // 1 detents per cycle:  00 ,  10 , [11],  01
    {{false, false, false, true}, 2},

    // 1 detents per cycle: [00],  10 ,  11 ,  01
    {{true, false, false, false}, 2},

    // 2 detents per cycle: [00],  10 , [11],  01
    {{true, false, false, true}, 1},

    // 2 detents per cycle:  00 , [10],  11,  [01]
    {{false, true, true, false}, 1},

    // 4 detents per cycle: [00], [10], [11], [01]
    {{true, true, true, true}, 0},
};


void RotaryEncoder::init(uint8_t pin0, uint8_t pin1, RotaryEncoderType typeEncoder) {
    m_pos = 0;
    m_pin0 = pin0;
    m_pin1 = pin1;
    m_encoderType = encoderTypes[typeEncoder];

    // start with position 0;
    m_oldState         = 0;
    m_position         = 0;
    m_positionExt      = 0;
    m_detentCounter    = 0;
    m_positionTimePrev = 0;   // for first startup avoid calculation of a high speed, m_positionTimePrev
    m_positionTime     = 500; // and m_positionTime must be initialized to avoid that they have the same value

    pinMode(m_pin0, INPUT_PULLUP);
    pinMode(m_pin1, INPUT_PULLUP);
}

void RotaryEncoder::pullEvents() {
    tick();

    int16_t pos = getPosition();
    if (pos == m_pos) {
        // nothing happened
        return;
    }

    bool dir = true;
    int16_t delta = pos - m_pos;
    m_delta += delta;

    if (delta < 0)
        dir = false;

    // protect from overflow
    if ((dir && (pos + delta * 2) > ROTENC_MAX) || (!dir && (pos - delta * 2) < -ROTENC_MAX)) {
        setPosition(0);
        pos = 0;
    }
    m_pos = pos;
}

void RotaryEncoder::resetPosition() {
    setPosition(0);
}

int16_t RotaryEncoder::getPosition() {
    return m_positionExt;
}

int32_t RotaryEncoder::getDelta() {
    uint32_t retDelta = m_delta;
    m_delta = 0;
    return retDelta;
}



void RotaryEncoder::tick() {
    bool     sig1      = !digitalRead(m_pin0); // to keep backwards compatibility for encoder type digitalRead must be negated
    bool     sig2      = !digitalRead(m_pin1); // to keep backwards compatibility for encoder type digitalRead must be negated
    int      speed     = 0;
    uint32_t currentMs = millis();

    int8_t thisState = sig1 | (sig2 << 1);

    if (currentMs - m_lastFastDec > 100 && m_detentCounter > 1) {
        m_lastFastDec = currentMs;
        m_detentCounter--;
    } else if (currentMs - m_positionTimePrev > 500) { // if more than 500ms no step detected, set fast acceleration to 0
        m_lastFastDec   = currentMs;
        m_detentCounter = 0;
    }

    if (m_oldState != thisState) {
        if (m_detentCounter > ROTENC_FAST_TRIGGER) { // at minimum X detents have to be detected before fast step can be detected
            speed = ROTENC_FAST_SPEED;
        } else {
            speed = 1;
        }

        m_position += ((KNOBDIR[thisState | (m_oldState << 2)] * speed)) << m_encoderType.resolutionShift;
        if (m_encoderType.detents[thisState]) {
            m_positionTimePrev = m_positionTime;
            m_positionTime     = currentMs;
            m_positionExt      = m_position >> 2; //m_encoderType.resolutionShift;
            m_detentCounter    = min(m_detentCounter + 1, 12);
        }
        m_oldState = thisState;
    }
}

void RotaryEncoder::setPosition(int16_t newPosition) {
    // only adjust the external part of the position.
    m_position    = ((newPosition >> m_encoderType.resolutionShift) | (m_position & 0x03));
    m_positionExt = newPosition;
}

} // pinicore
