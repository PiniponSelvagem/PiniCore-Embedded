/**
* @file		rtc_esp.hpp
* @brief	ESP32 clock.
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

#ifndef PINICORE_TIME_RTC_ESP_H
#define PINICORE_TIME_RTC_ESP_H

#include "irtc.hpp"

namespace pinicore {

class RTCesp : public IRTC {
    public:
        /**
         * @brief   Set a clock time.
         * @param   timestamp Unix epoch timestamp.
         */
        void set(time_t timestamp) override;
        
        /**
         * @brief   Set clock time.
         * @param   hour Hours: 0-23
         * @param   min Minutes: 0-59
         * @param   sec Seconds: 0-59
         * @param   day Day: 1-31
         * @param   month Month: 0-11
         * @param   year Year: 1900 and above
         */
        void set(int hour, int min, int sec, int day, int month, int year) override;

        /**
         * @brief   Get the current timestamp.
         * @return  Current unix epoch timestamp.
         */
        time_t get() override;
};

} // pinicore

#endif /* PINICORE_TIME_RTC_ESP_H */
