/**
* @file		veml7700.hpp
* @brief	Driver for light sensor VEML7700.
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

#ifndef PINICORE_SENSOR_VEML7700_H
#define PINICORE_SENSOR_VEML7700_H

#include <stdint.h>
#include <Adafruit_VEML7700.h>

namespace pinicore {

class VEML7700 {
    public:
        /**
         * @brief	Initializes the sensor.
         * @note	This function must be called prior to any other sensor specific functions.
         */
        void init();

        /**
         * @brief   Read current lux from hardware.
         * @return  Light in lux.
         */
        float readLux();

    private:
        Adafruit_VEML7700 m_veml7700;
};

} // pinicore

#endif /* PINICORE_SENSOR_VEML7700_H */
