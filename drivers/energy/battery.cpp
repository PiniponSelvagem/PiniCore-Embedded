#include "battery.hpp"
#include "utils/time.hpp"
#include <Arduino.h>

#define BATTERY_ADC_MAX     4095.f
#define BATTERY_ADC_REF     3.3f
#define BATTERY_ADC_DIVIDER 2.f

#define BATTERY_READ_SAMPLE_COUNT   16      // Number of samples to take to average the voltage read.

/**
 * @brief   Battery voltage look up table.
 * @note    Taken from a personal project for IoT university class using micropython.
 *          https://github.com/PiniponSelvagem/IoT-2122v/blob/master/project/micropython/entrace_system/lib/devices/axp.py
 */
static const float m_vbatLUT[101] = {
    3.0000, // 0
    3.1097, 3.1966, 3.2626, 3.3495, 3.3891, // 5
    3.4144, 3.4265, 3.4507, 3.4595, 3.4727, // 10
    3.4815, 3.4848, 3.4925, 3.4969, 3.5046, // 15
    3.5101, 3.5244, 3.5398, 3.5475, 3.5607, // 20
    3.5706, 3.5926, 3.6003, 3.6201, 3.6256, // 25
    3.6432, 3.6553, 3.6685, 3.6861, 3.6938, // 30
    3.7081, 3.7147, 3.7345, 3.7400, 3.7499, // 35
    3.7554, 3.7609, 3.7642, 3.7675, 3.7675, // 40
    3.7741, 3.7796, 3.7818, 3.7873, 3.7917, // 45
    3.7994, 3.8038, 3.8060, 3.8093, 3.8115, // 50
    3.8159, 3.8203, 3.8236, 3.8247, 3.8258, // 55
    3.8269, 3.8302, 3.8313, 3.8324, 3.8335, // 60
    3.8357, 3.8412, 3.8478, 3.8588, 3.8654, // 65
    3.8830, 3.8907, 3.9028, 3.9072, 3.9193, // 70
    3.9259, 3.9303, 3.9479, 3.9512, 3.9611, // 75
    3.9644, 3.9721, 3.9754, 3.9765, 3.9809, // 80
    3.9831, 3.9853, 3.9886, 3.9897, 3.9919, // 85
    3.9930, 3.9952, 3.9963, 3.9963, 3.9985, // 90
    3.9996, 4.0029, 4.0040, 4.0062, 4.0150, // 95
    4.0205, 4.0249, 4.0359, 4.0403, 4.0500  // 100
};


void Battery::init(uint8_t pin, float correction) {
    m_pin = pin;
    m_correction = correction;
    analogSetAttenuation(ADC_11db);
    pinMode(m_pin, INPUT);
}

float Battery::voltage() {
    return readVoltage();
}

uint8_t Battery::percentage() {
    float volts = voltage();
    if (volts <= m_vbatLUT[0]) return 0;

    for (int i=100; i>0; --i) {
        if (volts >= m_vbatLUT[i]) {
            return i;
        }
    }
    return 0;
}

float Battery::readVoltage() {
    uint64_t currMillis = getMillis();
    if (currMillis < m_lastReadAt + BATTERY_READ_INTERVAL_MS && m_lastReadAt != 0)
        return m_lastVoltage;
    m_lastReadAt = currMillis;

    uint32_t readSum = 0;
    for (int i=0; i<BATTERY_READ_SAMPLE_COUNT; ++i) {
        readSum += analogRead(m_pin);
        delay(4);
    }
    float readAvg = readSum / BATTERY_READ_SAMPLE_COUNT;
    float voltage = (readAvg / BATTERY_ADC_MAX) * BATTERY_ADC_REF * BATTERY_ADC_DIVIDER;
    m_lastVoltage = voltage * m_correction;
    return m_lastVoltage;
}
