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

#ifndef PINICORE_UTILS_TIME_H
#define PINICORE_UTILS_TIME_H

#include <stdint.h>

namespace pinicore {

/**
 * @brief Returns the milliseconds since BOOT.
 * @note Recommended to use this instead of Arduino "millis()" since that overflows after +/-49 days, because it uses 32 bits.
 */
uint64_t getMillis();

} // pinicore

#endif // PINICORE_UTILS_TIME_H