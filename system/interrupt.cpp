#include "interrupt.hpp"
#include <Arduino.h>

AutoDisableInterrupt::AutoDisableInterrupt()  { noInterrupts(); }
AutoDisableInterrupt::~AutoDisableInterrupt() {   interrupts(); }
