/**
* @file		plantsensor.hpp
* @brief	Plant Monitor Sensor from Sprig Labs, aka Root, data gather API.
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

#ifndef _PINICORE_SPRIGLABS_ROOT_H_
#define _PINICORE_SPRIGLABS_ROOT_H_

#include <stdint.h>

class SprigRoot {
    public:
        /**
         * @brief   Initialize the battery.
         * @param   pin Pin the battery read is connected to.
         */
        void init(uint8_t pin);

        float readLux();

        float readTemperature();
        float readHumidity();

        float readSoil();



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
};

#endif /* _PINICORE_SPRIGLABS_ROOT_H_ */