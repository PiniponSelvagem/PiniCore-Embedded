/**
* @file		dht.hpp
* @brief	Driver for temperature and humidity sensor DHT.
* @author	PiniponSelvagem
* @note     This driver is highly based of the one from Adafruit Industries.
*           Adjustments were made to go along with the philosophy of the PiniCore
*           library. One that stands out is not declaring the connection PIN
*           on the constructor, since that limits options on how to allocate memory
*           to use the original DHT.
*           Functions in this class that contain portions of code from Adafruit, are
*           marked in their comment header as a copyright.
*
* Copyright(C) PiniponSelvagem
*
***********************************************************************
* Software that is described here, is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
**********************************************************************/

/*!
 *  This is a library for DHT series of low cost temperature/humidity sensors.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Written by Adafruit Industries.
 *
 *  MIT license, all text above must be included in any redistribution
 */

#pragma once

#ifndef _PINICORE_SENSOR_DHT_H_
#define _PINICORE_SENSOR_DHT_H_

#include <stdint.h>

enum EDHT : uint8_t {
    DHT_11 = 11,
    DHT_12 = 12,
    DHT_21 = 21,
    DHT_22 = 22
};

/**
 * @note    If you want to use the Adafruit Industries DHT library,
 *          define '_PINICORE_SENSOR_DHT_H_' before including PiniCore library.
 */
class DHT {
    public:
        /**
         * @brief	Initializes the sensor.
         * @param   pin Pin the sensor is connected to.
         * @param   type DHT sensor variant type.
         * @note	This function must be called prior to any other sensor specific functions.
         */
        void init(uint8_t pin, EDHT type);

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
        /**
         * @brief   Read data from hardware, cache it and avoid unnecessary over reading the hardware.
         * @note    Will only actually read from hardware if 'MIN_READ_INTERVAL_MS' as passed.
         * @copyright Based of Adafruit Industries, library DHT.
         */
        void read();

        /**
         * @brief   Wait with timeout for a specific level to be set on \ref 'm_pin'.
         * @copyright Based of Adafruit Industries, library DHT.
         */
        uint32_t expectPulse(bool level);

        /**
         * @brief   Decode temperature from data.
         * @param   data2 The value at index 2 of the received data.
         * @param   data3 The value at index 3 of the received data.
         * @copyright Based of Adafruit Industries, library DHT.
         */
        float decodeTemperature(uint8_t data2, uint8_t data3);

        /**
         * @brief   Decode humidity from data.
         * @param   data2 The value at index 0 of the received data.
         * @param   data3 The value at index 1 of the received data.
         * @copyright Based of Adafruit Industries, library DHT.
         */
        float decodeHumidity(uint8_t data0, uint8_t data1);


        uint8_t m_pin;
        EDHT m_type;

        uint64_t m_lastReadAt;
        
        float m_temperature;
        float m_humidity;
};

#endif /* _PINICORE_SENSOR_DHT_H_ */
