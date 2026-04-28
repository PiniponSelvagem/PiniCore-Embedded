/**
* @file		hdc1080.hpp
* @brief	Driver for temperature and humidity sensor HDC1080.
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

#ifndef PINICORE_SENSOR_HDC1080_H
#define PINICORE_SENSOR_HDC1080_H

#include <stdint.h>
#include <ClosedCube_HDC1080.h>

namespace pinicore {

class HDC1080 {
    public:
        /**
         * @brief	Initializes the sensor.
         * @note	This function must be called prior to any other sensor specific functions.
         */
        void init();

        /**
         * @brief   Read current temperature from hardware.
         * @return  Temperature in ºC.
         */
        float readTemperature();

        /**
         * @brief   Read current humidity from hardware.
         * @return  Humidity in %.
         */
        float readHumidity();

        
    private:
        ClosedCube_HDC1080 m_hdc1080;
};

} // pinicore

#endif /* PINICORE_SENSOR_HDC1080_H */
