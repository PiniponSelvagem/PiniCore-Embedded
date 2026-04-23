#include "watchdog.hpp"
#include "log.hpp"

#include <esp_task_wdt.h>

namespace pinicore {

#define PINICORE_TAG_WDG "pcore_watchdog"

void watchdogSetup(uint32_t timeout) {
    LOG_I(PINICORE_TAG_WDG, "Internal watchdog configured to %d seconds", timeout);
    esp_task_wdt_init(timeout, true); //enable panic so ESP32 restarts
}

void watchdogEnable() {
    LOG_I(PINICORE_TAG_WDG, "Internal watchdog enabled");
    esp_task_wdt_add(NULL); //add current thread to WDT watch
}

void watchdogDisable() {
    LOG_W(PINICORE_TAG_WDG, "Internal watchdog disabled");
    esp_task_wdt_delete(NULL); //remove current thread from WDT watch
}

void watchdogIamAlive() {
    esp_task_wdt_reset();
}

} // pinicore
