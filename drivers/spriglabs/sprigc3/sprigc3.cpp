#include "sprigc3.hpp"

namespace pinicore {

void SprigC3::init() {
    m_max1704x.init();
}

float SprigC3::getBatteryVoltage() {
    return m_max1704x.getVoltage();
}

uint8_t SprigC3::getBatteryPercentage() {
    return m_max1704x.getPercentage();
}

} // pinicore
