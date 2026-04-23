#include "interrupt.hpp"
#include <Arduino.h>

namespace pinicore {

AutoDisableInterrupt::AutoDisableInterrupt()  { noInterrupts(); }
AutoDisableInterrupt::~AutoDisableInterrupt() {   interrupts(); }

} // pinicore