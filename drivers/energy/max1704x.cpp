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
    float soc = m_max1704x.getSOC();
    return static_cast<uint8_t>(
        soc < 0.0f ? 0.0f :
        soc > 100.0f ? 100.0f :
        soc
    );
}

} // pinicore
