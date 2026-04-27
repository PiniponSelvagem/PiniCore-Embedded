#include "basicsoilspriglabs.hpp"
#include "utils/time.hpp"
#include <Arduino.h>

namespace pinicore {

#define SOIL_ADC_DIVIDER 2.f
#define SOIL_ADC_DEFAULT_VREF    1100

#define SOIL_READ_SAMPLE_COUNT   16      // Number of samples to take to average the voltage read.

#define SOIL_DRY_VALUE  1.98
#define SOIL_WET_VALUE  0.89

void BasicSoilSprigLabs::init(uint8_t pin) {
    m_pin = pin;
    
    analogReadResolution(12);
    analogSetPinAttenuation(pin, ADC_11db);

    esp_adc_cal_characterize(
        ADC_UNIT_1,
        ADC_ATTEN_DB_12,
        ADC_WIDTH_BIT_12,
        SOIL_ADC_DEFAULT_VREF,
        &m_adcChars
    );

    pinMode(m_pin, INPUT);
}

float BasicSoilSprigLabs::readSoilHumidity() {
    uint64_t currMillis = getMillis();
    if (currMillis < m_lastReadAt + BASICSOIL_READ_INTERVAL_MS && m_lastReadAt != 0)
        return m_lastSoilHumidity;
    m_lastReadAt = currMillis;

    uint32_t readSum = 0;
    for (int i=0; i<SOIL_READ_SAMPLE_COUNT; ++i) {
        readSum += analogRead(m_pin);
        delay(4);
    }
    uint32_t adc = readSum / SOIL_READ_SAMPLE_COUNT;
    uint32_t mv = esp_adc_cal_raw_to_voltage(adc, &m_adcChars);
    float voltage = (mv / 1000.0) * SOIL_ADC_DIVIDER;

    m_lastSoilHumidity = (SOIL_DRY_VALUE - voltage) * 100.0 / (SOIL_DRY_VALUE - SOIL_WET_VALUE);
    if (m_lastSoilHumidity < 0)   m_lastSoilHumidity = 0;
    if (m_lastSoilHumidity > 100) m_lastSoilHumidity = 100;
    return m_lastSoilHumidity;
}

} // pinicore
