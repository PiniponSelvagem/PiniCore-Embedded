/**
* @file		led.hpp
* @brief	LED configuration API.
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

#ifndef PINICORE_IO_LED_H
#define PINICORE_IO_LED_H

#include <stdint.h>

namespace pinicore {

/**
 * @brief	Maximum number of leds supported.
 */
#define LED_MAX  32

class Led {
    public:
        /**
         * @brief	Initializes the configured led.
         * @param   pinLeds: Pins array the leds are connected to.
         * @param   size: Number of leds in the array. If 'size' is above 'LED_MAX', only the first 'LED_MAX' leds will be configured with the rest being ignored.
         * @note	This function must be called prior to any other LED functions.
         */
        void init(uint8_t* pinLeds, uint8_t size);

        /**
         * @brief	Checks for led state changes and their active state, and returns it as a mask.
         * @param	id: Led id, [0..LED_MAX-1] values outside this range will be declared as invalid led.
         * @return	'true' when led is ON and 'false' when led is OFF.
         * @warning Since this function uses an internal state, if the led state is changed without using this API, it might return incorrect results.
         */
        bool getState(uint8_t id);

        /**
         * @brief	Turns the led ON or OFF depend on the state.
         * @param	id: Led id, [0..LED_MAX-1] values outside this range will be declared as invalid led.
         * @param   state: State to change the led into.
         */
        void set(uint8_t id, bool state);

        /**
         * @brief	Toggles led state ON/OFF based on last state.
         * @param	id: Led id, [0..LED_MAX-1] values outside this range will be declared as invalid led.
         * @warning Since this function uses an internal state, if the led state is changed without using this API, it might toggle to an unexpected state.
         */
        void toggle(uint8_t id);

    private:
        /**
         * @brief	Array of pins containing all led pins configured when \ref 'init' is called.
         */
        uint8_t m_leds[LED_MAX];

        /**
         * @brief   Number of configured leds.
         */
        int m_nLeds = 0;

        /**
         * @brief	Current state of all leds.
         * 			Each led is represented with 1 bit.
         * 			1st bit -> 1st led active state (0 -> off, 1 -> on)
         * 			2nd bit -> 2nd led active state
         * 			...
         * 			31th bit -> 31th led active state
         * 			32th bit -> 32th led active state
         */
        uint32_t m_ledsCurrentState = 0;
};

} // pinicore

#endif /* PINICORE_IO_LED_H */
