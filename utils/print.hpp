/**
* @file		print.hpp
* @brief	Fancy string manipulation and representation.
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

#ifndef _PINICORE_PRINT_H_
#define _PINICORE_PRINT_H_

#include <stdint.h>

/**
 * @brief Convert value to binary in string format.
 * @param buffer Buffer used to place the string on during printing, must be large enough -> bits+1.
 * @param num The value to convert to binary.
 * @param bits Amount of bits to represent the num.
 */
void convertToBinaryString(char *buffer, uint32_t num, int bits);

#endif // _PINICORE_PRINT_H_