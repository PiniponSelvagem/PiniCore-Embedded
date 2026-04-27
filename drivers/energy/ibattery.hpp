/**
* @file		ibattery.hpp
* @brief	Interface for Battery management API.
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

#ifndef PINICORE_ENERGY_IBATTERY_H
#define PINICORE_ENERGY_IBATTERY_H

#include <stdint.h>

namespace pinicore {

class IBattery {
    public:
        /**
         * @brief   Get voltage (V) of the battery connected.
         * @return  In Volts.
         */
        virtual float getVoltage() = 0;

        /**
         * @brief   Get state of charge in percentage (%).
         * @return  Percentage [0..100] %.
         */
        virtual uint8_t getPercentage() = 0;
};

} // pinicore

#endif /* PINICORE_ENERGY_IBATTERY_H */