#include "board.hpp"
#include "utils/log.hpp"

#include <esp_system.h>

#define PINICORE_TAG_BOARD "pcore_board"

static bool __hasUniqueId = false;
static char __uniqueId[13] = {0};      // 12 -> mac address size; 1 -> termination character

const char* getUniqueId() {
    if (!__hasUniqueId) {
        uint8_t baseMac[6];
        if (esp_read_mac(baseMac, ESP_MAC_WIFI_STA) == ESP_OK) { // Get MAC address for WiFi station
            __hasUniqueId = true;
            sprintf(__uniqueId, "%02x%02x%02x%02x%02x%02x", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
        }
    }
    return __uniqueId;
};

bool wasLastResetManual() {
    bool wasManual = false;
    esp_reset_reason_t resetReason = esp_reset_reason();
    switch (resetReason) {
        case ESP_RST_POWERON:
        case ESP_RST_EXT:
        case ESP_RST_WDT:   // The reset 'button' can trigger this reason
            wasManual = true;
            break;
        default:    // There are more cases but not relevant
            break;
    }

    LOG_D(PINICORE_TAG_BOARD, "Last reset was '%s' and caused by 'esp_reset_reason_t = %d'", wasManual ? "manual" : "auto", resetReason);
    return wasManual;
}

bool wasLastResetFatal() {
    return esp_reset_reason() == ESP_RST_PANIC;
}
