/**
* @file		rl_virtual.hpp
* @brief	Virtual relays implementation, useful to test logic without hardware.
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

#ifndef PINICORE_IO_RELAYS_VIRTUAL_H
#define PINICORE_IO_RELAYS_VIRTUAL_H

#include <stdint.h>
#include "irelays.hpp"

namespace pinicore {

class RelaysVirtual : public IRelays {
    public:
        /**
         * @brief   Initializes the configured relays.
         * @param   modules The number of modules.
         * @param   relaysPerModule The number of relays per module, of the largest one if they are different sizes.
         * @note    This function must be called prior to any other Relays functions.
         */
        void init(uint8_t modules, uint8_t relaysPerModule);

        /**
         * @brief   Set a new configuration for virtual modules.
         * @param   modules The number of modules.
         * @param   relaysPerModule The number of relays per module, of the largest one if they are different sizes.
         * @warning Only call this function when all relays are off, otherwise unexpected behaviour will occur.
         */
        void setModules(uint8_t modules, uint8_t relaysPerModule);

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
         * @brief   Sends the command to the hardware.
         * @param   module The module id.
         * @param   relay The relay id in the module.
         * @param   state The state of the relay to be set to.
         * @return  True if the relay changed to the new state, false otherwise.
         * @note    It is not necessary for this function to check if 'module' and 'relay' are within valid
         *          range, since \ref 'IRelays' class will do that check before calling it.
         */
        bool setHardware(uint8_t module, uint8_t relay, bool state) override;
};

} // pinicore

#endif // PINICORE_IO_RELAYS_VIRTUAL_H