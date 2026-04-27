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

#ifndef PINICORE_ENERGY_BATTERY_H
#define PINICORE_ENERGY_BATTERY_H

#include <stdint.h>
#include "ibattery.hpp"
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

namespace pinicore {

class MAX1704X : public IBattery {
    public:
        /**
         * @brief   Initialize the battery reader MAX1704X.
         * @note	This function must be called prior to any other sensor specific functions.
         */
        void init();

        /**
         * @brief   Get voltage (V) of the battery connected.
         * @return  In Volts, if no battery connected the value should be above 4.2V, but not guaranteed.
         * @note    Battery read is cached for efficiency. Cached voltage value is only updated after
         *          'BASICBATTERY_READ_INTERVAL_MS' milliseconds since last read.
         */
        float getVoltage() override;

        /**
         * @brief   Get the current battery percentage by reading the current voltage and using a look-up table.
         * @return  Percentage [0..100] %.
         * @note    Internally calls 'voltage' and then uses its return value to convert it to percentage.
         */
        uint8_t getPercentage() override;

    
    private:
        SFE_MAX1704X m_max1704x;
};

} // pinicore

#endif /* PINICORE_ENERGY_BATTERY_H */