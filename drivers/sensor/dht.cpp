#include "dht.hpp"
#include <Arduino.h>
#include "utils/time.hpp"
#include "utils/log.hpp"
#include "system/interrupt.hpp"

#define PINICORE_TAG_DHT "pcore_dht"

#define MIN_READ_INTERVAL_MS    2000

void DHT::init(uint8_t pin, EDHT type) {
    m_pin  = pin;
    m_type = type;
}

float DHT::readTemperature() {
    read();
    return m_temperature;
}

float DHT::readHumidity() {
    read();
    return m_humidity;
}


void DHT::read() {
    // Check if sensor was read less than two seconds ago and return early
    // to use last reading.
    uint32_t currMillis = getMillis();
    if (m_lastReadAt != 0 && ((currMillis - m_lastReadAt) < MIN_READ_INTERVAL_MS)) {
        return;
    }
    m_lastReadAt = currMillis;

    // Reset 40 bits of received data to zero.
    uint8_t data[5];
    data[0] = data[1] = data[2] = data[3] = data[4] = 0;

    // Send start signal.  See DHT datasheet for full signal diagram:
    //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

    // Go into high impedence state to let pull-up raise data line level and
    // start the reading process.
    pinMode(m_pin, INPUT_PULLUP);
    delay(1);

    // First set data line low for a period according to sensor type
    pinMode(m_pin, OUTPUT);
    digitalWrite(m_pin, LOW);
    switch (m_type) {
        case EDHT::DHT_22:
        case EDHT::DHT_21:
            delayMicroseconds(1100); // data sheet says "at least 1ms"
            break;
        case EDHT::DHT_11:
        default:
            delay(20); // data sheet says at least 18ms, 20ms just to be safe
            break;
    }

    uint32_t cycles[80];
    int errorState = 0; // 0 -> no error, 1 -> timeout wait for low, 2 -> timeout wait for high
    {
        // End the start signal by setting data line high for 40 microseconds.
        pinMode(m_pin, INPUT_PULLUP);

        // Delay a moment to let sensor pull data line low.
        delayMicroseconds(55);  // pullTime

        // Now start reading the data line to get the value from the DHT sensor.

        // Turn off interrupts temporarily because the next sections
        // are timing critical and we don't want any interruptions.
        // Dont do serial prints here, otherwise ESP will panic.
        AutoDisableInterrupt lock;

        // First expect a low signal for ~80 microseconds followed by a high signal
        // for ~80 microseconds again.
        if (expectPulse(LOW) == UINT32_MAX) {
            errorState = 1;
            goto LABEL_EXIT_W_ERROR;
        }
        if (expectPulse(HIGH) == UINT32_MAX) {
            errorState = 2;
            goto LABEL_EXIT_W_ERROR;
        }

        // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
        // microsecond low pulse followed by a variable length high pulse.  If the
        // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
        // then it's a 1.  We measure the cycle count of the initial 50us low pulse
        // and use that to compare to the cycle count of the high pulse to determine
        // if the bit is a 0 (high state cycle count < low state cycle count), or a
        // 1 (high state cycle count > low state cycle count). Note that for speed
        // all the pulses are read into a array and then examined in a later step.
        for (int i = 0; i < 80; i += 2) {
            cycles[i] = expectPulse(LOW);
            cycles[i + 1] = expectPulse(HIGH);
        }
    } // Timing critical code is now complete.

LABEL_EXIT_W_ERROR:
    switch (errorState) {
        case 0:
            break;
        case 1:
            LOG_W(PINICORE_TAG_DHT, "Timeout waiting for start signal low pulse");
            return;
        case 2:
            LOG_W(PINICORE_TAG_DHT, "Timeout waiting for start signal high pulse");
            return;
        default:
            break;
    }

    // Inspect pulses and determine which ones are 0 (high state cycle count < low
    // state cycle count), or 1 (high state cycle count > low state cycle count).
    for (int i = 0; i < 40; ++i) {
        uint32_t lowCycles = cycles[2 * i];
        uint32_t highCycles = cycles[2 * i + 1];
        if ((lowCycles == UINT32_MAX) || (highCycles == UINT32_MAX)) {
            LOG_W(PINICORE_TAG_DHT, "Timeout waiting for pulse");
            return;
        }
        data[i / 8] <<= 1;
        // Now compare the low and high cycle times to see if the bit is a 0 or 1.
        if (highCycles > lowCycles) {
            // High cycles are greater than 50us low cycle count, must be a 1.
            data[i / 8] |= 1;
        }
        // Else high cycles are less than (or equal to, a weird case) the 50us low
        // cycle count so this must be a zero.  Nothing needs to be changed in the
        // stored data.
    }

    //LOG_T(PINICORE_TAG_DHT, "Received: 0x%02x%02x%02x%02x", data[0], data[1], data[2], data[3]);
    uint8_t checksumExpected = data[0] + data[1] + data[2] + data[3];
    //LOG_T(PINICORE_TAG_DHT, "Checksum: [actual: 0x%02x] [expected: 0x%02x]", data[4], checksumExpected);

    // Check we read 40 bits and that the checksum matches.
    if (data[4] != (checksumExpected & 0xFF)) {
        LOG_E(PINICORE_TAG_DHT, "DHT checksum failed");
        return;
    }

    // Decode received data and cache it
    m_temperature = decodeTemperature(data[2], data[3]);
    m_humidity    = decodeHumidity(data[0], data[1]);
}

uint32_t DHT::expectPulse(bool level) {
    uint32_t count = 0;
    while (digitalRead(m_pin) == level) {
        if (count++ >= microsecondsToClockCycles(1000)) {
            return UINT32_MAX; // Exceeded timeout, fail.
        }
    }
    return count;
}

float DHT::decodeTemperature(uint8_t data2, uint8_t data3) {
    float result;
    switch (m_type) {
        case EDHT::DHT_11:
            result = data2;
            if (data3 & 0x80) {
                result = -1 - result;
            }
            result += (data3 & 0x0f) * 0.1;
            break;
        case EDHT::DHT_12:
            result = data2;
            result += (data3 & 0x0f) * 0.1;
            if (data2 & 0x80) {
                result *= -1;
            }
            break;
        case EDHT::DHT_22:
        case EDHT::DHT_21:
            result = ((uint32_t)(data2 & 0x7F)) << 8 | data3;
            result *= 0.1;
            if (data2 & 0x80) {
                result *= -1;
            }
            break;
    }
    return result;
}

float DHT::decodeHumidity(uint8_t data0, uint8_t data1) {
    float result;
    switch (m_type) {
        case EDHT::DHT_11:
        case EDHT::DHT_12:
            result = data0 + data1 * 0.1;
        break;
        case EDHT::DHT_22:
        case EDHT::DHT_21:
            result = ((uint32_t)data0) << 8 | data1;
            result *= 0.1;
            break;
    }
    return result;
}