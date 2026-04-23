/**
* @file		rl_x16blue.hpp
* @brief	Replays implementation for the ESP32-Relay-X16 board with 16 relays.
*           More information at: https://devices.esphome.io/devices/esp32-relay-x16/
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

#ifndef PINICORE_IO_RELAYS_X16BLUE_H
#define PINICORE_IO_RELAYS_X16BLUE_H

#include <stdint.h>
#include "irelays.hpp"

namespace pinicore {

class RelaysX16Blue : public IRelays {
    public:
        /**
         * @brief   Initializes the configured relays.
         * @param   pinEnable Pin to enable the shift register output.
         * @param   pinLatch Pin that controls the latch signal for updating the relay states.
         * @param   pinClock Pin used to clock data into the shift register.
         * @param   pinData Pin used to shift relay state data into the register.
         * @note    This function must be called prior to any other Relays functions.
         */
        void init(
            uint8_t pinEnable = 5, uint8_t pinLatch = 12,
            uint8_t pinClock = 13, uint8_t pinData = 14,
            bool isActiveLow = false
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
         * @brief   Sends the command to the hardware.
         * @param   module The module id.
         * @param   relay The relay id in the module.
         * @param   state The state of the relay to be set to.
         * @return  True if the relay changed to the new state, false otherwise.
         * @note    It is not necessary for this function to check if 'module' and 'relay' are within valid
         *          range, since \ref 'IRelays' class will do that check before calling it.
         */
        bool setHardware(uint8_t module, uint8_t relay, bool state) override;


        uint8_t m_pinLatch;
        uint8_t m_pinClock;
        uint8_t m_pinData;
        bool m_isActiveLow;
};

} // pinicore

#endif // PINICORE_IO_RELAYS_X16BLUE_H