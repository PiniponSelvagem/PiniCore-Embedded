#include "max1704x.hpp"
#include "utils/log.hpp"
#include <Arduino.h>

namespace pinicore {

#define PINICORE_TAG_MAX1704X  "pcore_max1704x"

void MAX1704X::init() {
    if (!m_max1704x.begin())
        LOG_E(PINICORE_TAG_MAX1704X, "MAX17043 not detected");
}

float MAX1704X::getVoltage() {
    return m_max1704x.getVoltage();
}

uint8_t MAX1704X::getPercentage() {
    return m_max1704x.getSOC();
}

} // pinicore
