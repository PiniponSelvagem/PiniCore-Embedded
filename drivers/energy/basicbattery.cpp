#include "basicbattery.hpp"
#include "utils/time.hpp"
#include <Arduino.h>

namespace pinicore {

#define BATTERY_ADC_DIVIDER         2.f
#define BATTERY_ADC_DEFAULT_VREF    1100

#define BATTERY_READ_SAMPLE_COUNT   16      // Number of samples to take to average the voltage read.

/**
 * @brief   Battery voltage look up table.
 * @note    Taken from a personal project for IoT university class using micropython.
 *          https://github.com/PiniponSelvagem/IoT-2122v/blob/master/project/micropython/entrace_system/lib/devices/axp.py
 */
static const float m_vbatLUT[101] = {
    3.0000, // 0
    3.1097, 3.1850, 3.2450, 3.3100, 3.3600, // 5
    3.4000, 3.4300, 3.4520, 3.4680, 3.4800, // 10
    3.4880, 3.4937, 3.4994, 3.5051, 3.5108, // 15
    3.5165, 3.5222, 3.5279, 3.5336, 3.5393, // 20
    3.5450, 3.5507, 3.5564, 3.5621, 3.5678, // 25
    3.5735, 3.5792, 3.5849, 3.5906, 3.5963, // 30
    3.6020, 3.6077, 3.6134, 3.6191, 3.6248, // 35
    3.6305, 3.6362, 3.6419, 3.6476, 3.6533, // 40
    3.6590, 3.6647, 3.6704, 3.6761, 3.6818, // 45
    3.6875, 3.6932, 3.6989, 3.7046, 3.7103, // 50
    3.7160, 3.7217, 3.7274, 3.7331, 3.7388, // 55
    3.7445, 3.7502, 3.7559, 3.7616, 3.7673, // 60
    3.7730, 3.7787, 3.7844, 3.7901, 3.7958, // 65
    3.8015, 3.8072, 3.8129, 3.8186, 3.8243, // 70
    3.8305, 3.8366, 3.8428, 3.8490, 3.8551, // 75
    3.8613, 3.8675, 3.8736, 3.8798, 3.8860, // 80
    3.8926, 3.8994, 3.9063, 3.9132, 3.9203, // 85
    3.9275, 3.9347, 3.9422, 3.9498, 3.9575, // 90
    3.9655, 3.9737, 3.9820, 3.9907, 3.9997, // 95
    4.0089, 4.0186, 4.0286, 4.0390, 4.0500  // 100
};


void BasicBattery::init(uint8_t pin) {
    m_pin = pin;
    
    analogReadResolution(12);
    analogSetPinAttenuation(pin, ADC_11db);

    esp_adc_cal_characterize(
        ADC_UNIT_1,
        ADC_ATTEN_DB_12,
        ADC_WIDTH_BIT_12,
        BATTERY_ADC_DEFAULT_VREF,
        &m_adcChars
    );

    pinMode(m_pin, INPUT);
}

float BasicBattery::getVoltage() {
    return readVoltage();
}

uint8_t BasicBattery::getPercentage() {
    float volts = getVoltage();
    if (volts <= m_vbatLUT[0]) return 0;

    for (int i=100; i>0; --i) {
        if (volts >= m_vbatLUT[i]) {
            return i;
        }
    }
    return 0;
}

float BasicBattery::readVoltage() {
    uint64_t currMillis = getMillis();
    if (currMillis < m_lastReadAt + BASICBATTERY_READ_INTERVAL_MS && m_lastReadAt != 0)
        return m_lastVoltage;
    m_lastReadAt = currMillis;

    uint32_t readSum = 0;
    for (int i=0; i<BATTERY_READ_SAMPLE_COUNT; ++i) {
        readSum += analogRead(m_pin);
        delay(4);
    }
    uint32_t adc = readSum / BATTERY_READ_SAMPLE_COUNT;
    uint32_t mv = esp_adc_cal_raw_to_voltage(adc, &m_adcChars);
    m_lastVoltage = (mv / 1000.0) * BATTERY_ADC_DIVIDER;
    return m_lastVoltage;
}

} // pinicore
