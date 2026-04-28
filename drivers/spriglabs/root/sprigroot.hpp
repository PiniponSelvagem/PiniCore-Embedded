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

#ifndef PINICORE_SPRIGLABS_ROOT_H
#define PINICORE_SPRIGLABS_ROOT_H

#include <stdint.h>
#include "drivers/sensor/light/veml7700.hpp"
#include "drivers/sensor/environment/hdc1080.hpp"
#include "drivers/sensor/soil/basicsoilspriglabs.hpp"

namespace pinicore {

class SprigRoot {
    public:
        /**
         * @brief   Initialize the Sprig ROOT.
         * @param   pinSoilEnable Pin to enable soil humidity read.
         * @param   pinTempHumEnable Pin to enable ambient temperature and humidity read.
         * @param   pinLightEnable Pin to enable light read.
         */
        void init(uint8_t pinSoilEnable = 5, uint8_t pinTempHumEnable = 6, uint8_t pinLightEnable = 7, uint8_t pinSoil = 4);

        /**
         * @brief   Read current lux from hardware.
         * @return  Light in lux.
         */
        float getLux();

        /**
         * @brief   Read current temperature from hardware.
         * @return  Temperature in ºC.
         */
        float getAmbientTemperature();

        /**
         * @brief   Read current humidity from hardware.
         * @return  Humidity in %.
         */
        float getAmbientHumidity();

        /**
         * @brief   Read current temperature from hardware.
         * @return  Temperature in ºC.
         */
        float getSoilHumidity();

    
    private:
        uint8_t m_pinSoil;

        VEML7700 m_veml7700;            // Light sensor
        HDC1080 m_hdc1080;              // Ambient temperature and humidity sensor
        BasicSoilSprigLabs m_sprigsoil; // Soil humidity sensor
};

} // pinicore

#endif /* PINICORE_SPRIGLABS_ROOT_H */