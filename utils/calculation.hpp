/**
* @file		calculation.hpp
* @brief	Calculations, small algorithms, and math related stuff.
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

#ifndef PINICORE_UTILS_CALCULATION_H
#define PINICORE_UTILS_CALCULATION_H

#include <stdint.h>

namespace pinicore {

/**
 * @brief   Generate a random value between 0 and UINT32_MAX.
 * @return  Value range [0..UINT32_MAX].
 */
uint32_t randomGenerator();

} // pinicore

#endif // PINICORE_UTILS_CALCULATION_H