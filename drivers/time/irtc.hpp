/**
* @file		irtc.hpp
* @brief	Real Time Clock generic API.
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

#ifndef PINICORE_TIME_RTC_H
#define PINICORE_TIME_RTC_H

#include <stdint.h>
#include <time.h>

namespace pinicore {

class IRTC {
    public:
        /**
         * @brief   Set a clock time.
         * @param   timestamp Unix epoch timestamp.
         */
        virtual void set(time_t timestamp) = 0;
        
        /**
         * @brief   Set clock time.
         * @param   hour Hours: 0-23
         * @param   min Minutes: 0-59
         * @param   sec Seconds: 0-59
         * @param   day Day: 1-31
         * @param   month Month: 0-11
         * @param   year Year: 1900 and above
         */
        virtual void set(int hour, int min, int sec, int day, int month, int year) = 0;

        /**
         * @brief   Get the current timestamp.
         * @return  Current unix epoch timestamp.
         */
        virtual time_t get() = 0;
};

} // pinicore

#endif /* PINICORE_TIME_RTC_H */
