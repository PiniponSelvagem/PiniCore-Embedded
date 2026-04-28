#include "sprigroot.hpp"
#include <Arduino.h>

namespace pinicore {

#define PIN_SPRIG_ROOT_SOIL 4

void SprigRoot::init(uint8_t pinSoilEnable, uint8_t pinTempHumEnable, uint8_t pinLightEnable, uint8_t pinSoil) {
    m_pinSoil = pinSoil;
    
    pinMode(pinSoilEnable, OUTPUT);
    pinMode(pinTempHumEnable, OUTPUT);
    pinMode(pinLightEnable, OUTPUT);

    digitalWrite(pinSoilEnable, HIGH);
    digitalWrite(pinTempHumEnable, HIGH);
    digitalWrite(pinLightEnable, HIGH);

    m_veml7700.init();
    m_hdc1080.init();
    m_sprigsoil.init(m_pinSoil);
}

float SprigRoot::getLux() {
    return m_veml7700.readLux();
}

float SprigRoot::getAmbientTemperature() {
    return m_hdc1080.readTemperature();
}

float SprigRoot::getAmbientHumidity() {
    return m_hdc1080.readHumidity();
}

float SprigRoot::getSoilHumidity() {
    return m_sprigsoil.readSoilHumidity();
}

} // pinicore
