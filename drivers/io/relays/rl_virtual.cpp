#include "rl_virtual.hpp"
#include "utils/log.hpp"

namespace pinicore {

#define PINICORE_TAG_RELAYS_VIRTUAL "pcore_relaysvirtual"


void RelaysVirtual::init(uint8_t modules, uint8_t relaysPerModule) {
    p_modules = modules;
    p_relaysPerModule = relaysPerModule;
    initModules();
}

void RelaysVirtual::setModules(uint8_t modules, uint8_t relaysPerModule) {
    p_modules = modules;
    p_relaysPerModule = relaysPerModule;
}

bool RelaysVirtual::isModuleConnected(uint8_t module) {
    return (module < p_modules);
}

void RelaysVirtual::initModules() {
    if (p_modules == 0) {
        p_modules = 8;
    }
    if (p_relaysPerModule == 0) {
        p_relaysPerModule = 16;
    }
    LOG_I(PINICORE_TAG_RELAYS_VIRTUAL,
        "Virtual relays configured (%d): [modules: %d] [relaysPerModule: %d]",
        (p_modules*p_relaysPerModule), p_modules, p_relaysPerModule
    );
}

bool RelaysVirtual::setHardware(uint8_t module, uint8_t relay, bool state) {
    bool currState = get(module, relay);
    if (currState != state) {
        delay(10);  // Simulate hardware access (10ms)
        return true;
    }
    return false;
}

} // pinicore
