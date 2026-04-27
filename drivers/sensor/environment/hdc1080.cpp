#include "hdc1080.hpp"
#include "utils/log.hpp"
#include <Arduino.h>

namespace pinicore {

#define PINICORE_TAG_HDC1080  "pcore_hdc1080"
#define HDC1080_ADDRESS       0x40

void HDC1080::init() {
    m_hdc1080.begin(HDC1080_ADDRESS);
}

float HDC1080::readTemperature() {
    return m_hdc1080.readTemperature();
}

float HDC1080::readHumidity() {
    return m_hdc1080.readHumidity();
}

} // pinicore
