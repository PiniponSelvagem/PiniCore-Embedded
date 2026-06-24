/**
* @file		rl_gpio.hpp
* @brief	GPIO relays implementation, control relays directly from GPIO pins, up to 16 relays.
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

#ifndef PINICORE_IO_RELAYS_GPIO_H
#define PINICORE_IO_RELAYS_GPIO_H

#include <stdint.h>
#include "irelays.hpp"

namespace pinicore {

/**
 * @brief	Maximum number of relays supported.
 */
#define RELAYS_GPIO_MAX  16

class RelaysGPIO : public IRelays {
    public:
        /**
         * @brief   Initializes the configured relays.
         * @param   pinBtns: Pins array the relays are connected to.
         * @param   size: Number of relays in the array. If 'size' is above 'RELAYS_GPIO_MAX', only the first 'RELAYS_GPIO_MAX' relays will be configured with the rest being ignored.
         * @param   isActiveLow Invert relays operation.
         * @note    This function must be called prior to any other Relays functions.
         */
        void init(uint8_t* pinRelays, uint8_t size, bool isActiveLow = false);

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


        uint32_t m_nRelays;
        uint8_t m_pinRelays[RELAYS_GPIO_MAX];
        bool m_isActiveLow;
};

} // pinicore

#endif // PINICORE_IO_RELAYS_GPIO_H