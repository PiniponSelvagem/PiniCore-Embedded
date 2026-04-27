/**
* @file		basicsoilspriglabs.hpp
* @brief	Driver for soil sensor of SprigLabs Root.
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

#ifndef PINICORE_SENSOR_BASICSOILSPRIGLABS_H
#define PINICORE_SENSOR_BASICSOILSPRIGLABS_H

#include <stdint.h>
#include "esp_adc_cal.h"

namespace pinicore {

#define BASICSOIL_READ_INTERVAL_MS    5000    // Interval in milliseconds that should update the cached battery status.

class BasicSoilSprigLabs {
    public:
        /**
         * @brief   Initialize the soil humidity reader.
         * @param   pin Pin the soil read is connected to.
         * @note	This function must be called prior to any other sensor specific functions.
         */
        void init(uint8_t pin);

        /**
         * @brief   Get soil humidity.
         * @return  Humidity in %.
         * @note    Humidity read is cached for efficiency. Cached value is only updated after
         *          'BASICSOIL_READ_INTERVAL_MS' milliseconds since last read.
         */
        float readSoilHumidity();


    private:
        esp_adc_cal_characteristics_t m_adcChars;
        uint8_t m_pin;

        float m_lastSoilHumidity;
        uint64_t m_lastReadAt;
};

} // pinicore

#endif /* PINICORE_SENSOR_BASICSOILSPRIGLABS_H */
