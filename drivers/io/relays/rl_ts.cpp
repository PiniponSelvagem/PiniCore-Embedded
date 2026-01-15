#include "rl_ts.hpp"
#include "utils/log.hpp"

#define PINICORE_TAG_RELAYS_TS "pcore_relaysts"

/**
 * Developer note:
 * - There are better ways to detect the presence of the relays modules, other than the feedback pin.
 * - Pinging the MCP using the I2C connection is enough to detect it.
 * - Feedback pin, is nice, but why waste hardware on something that only detects 1 pin, which might give
 *   wrong results if the rest of the pins of the MCP are "broken" except that one.
 * - This relays implementation does not use the feedback pin logic, since that complicates the code and does
 *   not provide the expected reliability. What this will do is ignore that pin on the MCP and use the
 *   'pinRelayReplaceFeedback', for that replaced pin.
 */
#define RELAYS_TS_DEV_NOTE

/**
 * @brief   Time, in ms, after which when \ref 'isModuleConnected' is called, it should detect connected modules.
 */
#define RELAYS_TS_MODULES_DETECT_INTERVAL_MS   (5*1000)

/**
 * @brief   The index position of the relay that was replaced by feedback.
 */
#define RELAYS_TS_REPLACE_FEEDBACK_IDX  15


void RelaysTS::init(
    uint8_t pinSDA, uint8_t pinSCL,
    uint8_t pinRelaysEnable, uint8_t pinResetMCP,
    bool isActiveLow,
    uint8_t pinRelayReplaceFeedback
) {
    m_pinRelaysEnable = pinRelaysEnable;
    m_pinResetMCP = pinResetMCP;
    m_isActiveLow = isActiveLow;
    m_pinRelayReplaceFeedback = pinRelayReplaceFeedback;

    pinMode(m_pinRelaysEnable, OUTPUT);
    pinMode(m_pinResetMCP, OUTPUT);

    if (pinRelayReplaceFeedback != UINT8_MAX) {
        pinMode(pinRelayReplaceFeedback, OUTPUT);
    }

    digitalWrite(m_pinResetMCP, LOW);
    digitalWrite(m_pinRelaysEnable, LOW);

    Wire.begin(pinSDA, pinSCL);
    initModules();
}

bool RelaysTS::isModuleConnected(uint8_t module) {
    if (getMillis() > m_lastTimeCheckedModules + RELAYS_TS_MODULES_DETECT_INTERVAL_MS) {
        // ensures 'm_modulesConnected' is updated, but dont waste resources constantly checking for them
        detectModules();
        m_lastTimeCheckedModules = getMillis();
    }
    return m_modulesConnected & (0x1 << module);
}

void RelaysTS::initModules() {
    p_modules = 8;
    p_relaysPerModule = 16;

    digitalWrite(m_pinResetMCP, HIGH);    
    uint8_t modulesFound = detectModules();
    digitalWrite(m_pinRelaysEnable, HIGH);

    LOG_I(PINICORE_TAG_RELAYS_TS, "Detected %d modules during init", modulesFound);
}

void RelaysTS::initModule(uint8_t module) {
    for (int i=0; i<p_relaysPerModule; ++i) {
        m_mcp[module].pinMode(i, OUTPUT);
        setMCP(module, i, false);
    }
}

bool RelaysTS::setHardware(uint8_t module, uint8_t relay, bool state) {
    if (
        (m_pinRelayReplaceFeedback != UINT8_MAX) &&
        (module == 0 && relay == RELAYS_TS_REPLACE_FEEDBACK_IDX)
    ) {
        digitalWrite(m_pinRelayReplaceFeedback, state);
        return true;
    }
    else if (isModuleConnected(module)) {
        setMCP(module, relay, state);
        return true;
    }
    return false;
}

void RelaysTS::setMCP(uint8_t module, uint8_t relay, bool state) {
    m_mcp[module].digitalWrite(relay,
        m_isActiveLow ? !state : state
    );
}

int RelaysTS::detectModules() {
    uint8_t detected = 0;
    uint8_t modulesConnected = 0;
    
    // Detect new modules
    for (int i=0; i<p_modules; ++i) {
        if (isModuleDetected(i)) {
            if (!(m_modulesConnected & (0x1 << i))) {
                initModule(i);
                resetModuleState(i);   // sync software state with hardware initial state
                LOG_I(PINICORE_TAG_RELAYS_TS, "Detected new module %d, address 0x%02x", i, m_mcpAddress[i]);
                _onModule(i, true);
            }
            modulesConnected = modulesConnected | (0x1 << i);
            ++detected;
        }
    }

    // Detect removed modules since last check
    uint8_t removed = (m_modulesConnected ^ modulesConnected) & m_modulesConnected;
    for (int i=0; i<p_modules; ++i) {
        if (removed & (0x1 << i)) {
            LOG_W(PINICORE_TAG_RELAYS_TS, "Detected removed module %d, address 0x%02x", i, m_mcpAddress[i]);
            resetModuleState(i);
            _onModule(i, false);
        }
    }

    m_modulesConnected = modulesConnected;
    return detected;
}

bool RelaysTS::isModuleDetected(uint8_t module) {
    if (!m_mcpInit[module]) {
        // Calling too many begin_I2C, crashes the controller after a while. This makes sure it is called only once.
        m_mcpInit[module] = true;
        return m_mcp[module].begin_I2C(m_mcpAddress[module], &Wire);
    }
    // Ping module that begin_I2C was already called
    Wire.beginTransmission(m_mcpAddress[module]);
    return (Wire.endTransmission() == 0);
}
