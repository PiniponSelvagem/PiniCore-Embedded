/**
* @file		battery.hpp
* @brief	Battery management API.
* @author	PiniponSelvagem
* @note     Currently only targeted and tested with TTGO/LILYGO LoRa32 Display v1.6.1.
*
* Copyright(C) PiniponSelvagem
*
***********************************************************************
* Software that is described here, is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
**********************************************************************/

#pragma once

#ifndef _PINICORE_ENERGY_BATTERY_H_
#define _PINICORE_ENERGY_BATTERY_H_

#include <stdint.h>

#define BATTERY_CORRECTION_TTGO_LORA32_DISPLAY  (3.855 / 3.625)     // Correction multiplier taken from 'T3_V1.6.1'

#define BATTERY_READ_INTERVAL_MS    5000    // Interval in milliseconds that should update the cached battery status.

class Battery {
    public:
        /**
         * @brief   Initialize the battery.
         * @param   pin Pin the battery read is connected to.
         * @param   correction Correction multiplier, get it using a multimeter and then dividing it with the value
         *          read by the controller when using correction value of '1'.
         * @note    Getting the correction value:
         *              (multimeter_read/controller_read) = correction_value
         */
        void init(uint8_t pin, float correction = 1.f);

        /**
         * @brief   Get voltage (V) of the battery connected.
         * @return  In Volts, if no battery connected the value should be above 4.2V, but not guaranteed.
         * @note    Battery read is cached for efficiency. Cached voltage value is only updated after
         *          'BATTERY_READ_INTERVAL_MS' milliseconds since last read.
         */
        float voltage();

        /**
         * @brief   Get the current battery percentage by reading the current voltage and using a look-up table.
         * @return  Percentage [0..100] %.
         * @note    Internally calls 'voltage' and then uses its return value to convert it to percentage.
         */
        uint8_t percentage();


    private:
        /**
         * @brief   Get voltage (V) of the battery connected.
         * @return  In Volts, if no battery connected the value should be above 4.2V, but not guaranteed.
         * @note    Battery read is cached for efficiency. Cached voltage value is only updated after
         *          'BATTERY_READ_INTERVAL_MS' milliseconds since last read.
         */
        float readVoltage();


        uint8_t m_pin;
        float m_correction;

        float m_lastVoltage;
        uint64_t m_lastReadAt;
};

#endif /* _PINICORE_ENERGY_BATTERY_H_ */
