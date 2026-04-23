/**
* @file		rl_ts.hpp
* @brief	Replays implementation using the MCP23017, 8 modules 16 relays, with some special quirks.
* @author	PiniponSelvagem
*
* Copyright(C) PiniponSelvagem
*
***********************************************************************
* Software that is described here, is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
**********************************************************************/

#pragma once

#ifndef PINICORE_IO_RELAYS_TS_H
#define PINICORE_IO_RELAYS_TS_H

#include <stdint.h>
#include "irelays.hpp"
#include <Adafruit_MCP23X17.h>

namespace pinicore {

#define RELAYS_TS_PER_MODULE   16
#define RELAYS_TS_MODULES      8
#define RELAYS_TS_MAX          (RELAYS_TS_PER_MODULE * RELAYS_TS_MODULES)
#if RELAYS_TS_MAX > RELAYS_MAX
    #error "Relays TS max count is above of what the library supports!"
#endif


/**
 * Board configuration based on their version.
 * The list below is from X version until next one of the list, last one being the latest one.
 * --> R01: 'init(21, 22, 19, 18, true, UINT8_MAX)'
 * --> R05: 'init(21, 22, 19, 18, false, UINT8_MAX)'
 * --> R08: 'init(21, 22, 19, 18, false, 12)'
 */
class RelaysTS : public IRelays {
    public:
        /**
         * @brief   Initializes the configured relays.
         * @param   pinSDA I2C sda pin.
         * @param   pinSDA I2C scl pin.
         * @param   pinRelaysEnable Pin that enables power to the relays.
         * @param   pinResetMCP Pin to reset MCP.
         * @param   isActiveLow True for inverted / active low operation.
         * @param   pinRelayReplaceFeedback Pin that is used to operate the relay that got replaced by feedback.
         * @note    This function must be called prior to any other Relays functions.
         */
        void init(
            uint8_t pinSDA = 21, uint8_t pinSCL = 22,
            uint8_t pinRelaysEnable = 19, uint8_t pinResetMCP = 18,
            bool isActiveLow = false,
            uint8_t pinRelayReplaceFeedback = 12
        );

        /**
         * @brief   Check if a module is connected.
         * @param   module Module index.
         * @return  Module connected status.
         */
        bool isModuleConnected(uint8_t module) override;


    private:
        /**
         * @brief   Initializes the relay modules.
         * @warning This function is required to initialize the variables 'p_modules' and 'p_relaysPerModule'.
         */
        void initModules() override;
        
        /**
         * @brief   Initialize and configure a module.
         * @param   module Module index.
         */
        void initModule(uint8_t module);

        /**
         * @brief   Sends the command to the hardware.
         * @param   module The module id.
         * @param   relay The relay id in the module.
         * @param   state The state of the relay to be set to.
         * @return  True if the relay changed to the new state, false otherwise.
         * @note    It is not necessary for this function to check if 'module' and 'relay' are within valid
         *          range, since \ref 'IRelays' class will do that check before calling it.
         */
        bool setHardware(uint8_t module, uint8_t relay, bool state) override;
        
        /**
         * @brief   Sends the command to the respective MCP.
         * @param   module The module id.
         * @param   relay The relay id in the module.
         * @param   state The state of the relay to be set to.
         */
        void setMCP(uint8_t module, uint8_t relay, bool state);

        /**
         * @brief   Detects connected modules and configures them.
         * @return  Number of modules detected.
         */
        int detectModules();

        /**
         * @brief   Detects a module by pining the respective I2C address.
         * @param   module Module index.
         * @return  True if the module replied, false if module not found.
         */
        bool isModuleDetected(uint8_t module);


        uint8_t m_pinRelaysEnable;
        uint8_t m_pinResetMCP;
        bool m_isActiveLow;
        uint8_t m_pinRelayReplaceFeedback = UINT8_MAX;

        Adafruit_MCP23X17 m_mcp[RELAYS_TS_MODULES];
        bool m_mcpInit[RELAYS_TS_MODULES];
        const uint8_t m_mcpAddress[RELAYS_TS_MODULES] = { 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27 };
        uint8_t m_modulesConnected = 0;
        time_t m_lastTimeCheckedModules = 0;
};

} // pinicore

#endif // PINICORE_IO_RELAYS_16MCP_H