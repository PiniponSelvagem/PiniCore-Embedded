/**
* @file		time.hpp
* @brief	Time related helper functions.
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

#ifndef _PINICORE_TIME_H_
#define _PINICORE_TIME_H_

#include <stdint.h>

/**
 * @brief Returns the milliseconds since BOOT.
 * @note Recommended to use this instead of Arduino "millis()" since that overflows after +/-49 days, because it uses 32 bits.
 */
uint64_t getMillis();

#endif // _PINICORE_TIME_H_