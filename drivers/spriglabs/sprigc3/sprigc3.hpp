/**
* @file		sprigc3.hpp
* @brief	Sprig-C3 ESP32 from Sprig Labs.
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

#ifndef PINICORE_SPRIGLABS_SPRIGC3_H
#define PINICORE_SPRIGLABS_SPRIGC3_H

#include <stdint.h>
#include "drivers/energy/max1704x.hpp"

namespace pinicore {

class SprigC3 {
    public:
        /**
         * @brief   Initialize the Sprig-C3.
         */
        void init();

        /**
         * @brief   Get voltage (V) of the battery connected.
         * @return  In Volts, if no battery connected the value should be above 4.2V, but not guaranteed.
         * @note    Battery read is cached for efficiency. Cached voltage value is only updated after
         *          'BASICBATTERY_READ_INTERVAL_MS' milliseconds since last read.
         */
        float getBatteryVoltage();

        /**
         * @brief   Get the current battery percentage by reading the current voltage and using a look-up table.
         * @return  Percentage [0..100] %.
         * @note    Internally calls 'voltage' and then uses its return value to convert it to percentage.
         */
        uint8_t getBatteryPercentage();

    
    private:
        MAX1704X m_max1704x;
};

} // pinicore

#endif /* PINICORE_SPRIGLABS_SPRIGC3_H */