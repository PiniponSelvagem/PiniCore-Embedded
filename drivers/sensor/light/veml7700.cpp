#include "veml7700.hpp"
#include "utils/log.hpp"
#include <Arduino.h>

namespace pinicore {

#define PINICORE_TAG_VEML7700  "pcore_veml7700"

void VEML7700::init() {
    if (!m_veml7700.begin())
        LOG_E(PINICORE_TAG_VEML7700, "VEML7700 not detected");
}

float VEML7700::readLux() {
    return m_veml7700.readLux();
}

} // pinicore
