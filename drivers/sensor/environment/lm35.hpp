/**
* @file		lm35.hpp
* @brief	Driver for temperature sensor LM35.
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

#ifndef PINICORE_SENSOR_LM35_H
#define PINICORE_SENSOR_LM35_H

#include <stdint.h>

namespace pinicore {

class LM35 {
    public:
        /**
         * @brief	Initializes the sensor.
         * @param   pin Pin the sensor is connected to.
         * @param   offset Optional offset value to add to the temperature reading, calibrating the result value.
         * @note	This function must be called prior to any other sensor specific functions.
         */
        void init(uint8_t pin, float offset = 0.f);

        /**
         * @brief   Read current temperature from hardware.
         * @return  Temperature in ºC.
         */
        float readTemperature();

    private:
        uint8_t m_pin;
        float m_offset;
};

} // pinicore

#endif /* PINICORE_SENSOR_LM35_H */
