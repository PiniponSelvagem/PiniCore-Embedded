/**
* @file		array.hpp
* @brief	Arrays and custom arrays management.
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

#ifndef PINICORE_UTILS_ARRAY_H
#define PINICORE_UTILS_ARRAY_H

#include <stdint.h>
#include <stddef.h>

namespace pinicore {

/**
 * @brief   Calculate the bit position on the provided 32-bit array that is divided by sections.
 * @param   arraySize The size of the array.
 * @param   sectionSize The number of bits each section has.
 * @param   section The index of the section.
 * @param   bit The index of the bit in that section.
 * @param   wordIndex Return value, the calculated index in the provided array.
 * @param   bitIndex Return value, the calculated bit index in the wordIndex.
 * @return  True if wordIndex and bitIndex are valid, false otherwise.
 * @note    Only 32-bit arrays are supported.
 */
bool calculateBitIndex(
    size_t arraySize,
    uint32_t sectionSize,
    uint32_t section, uint32_t bit,
    uint32_t* wordIndex, uint32_t* bitIndex
);

/**
 * @brief   Calculate the section and bit on the provided 32-bit array that is divided by sections.
 *          Similar to 'calculateBitIndex', but the inverse process.
 * @param   arraySize The size of the array.
 * @param   sectionSize The number of bits each section has.
 * @param   wordIndex The index in the provided array.
 * @param   bitIndex The bit index in the wordIndex.
 * @param   section Return value, the index of the section.
 * @param   bit Return value, the index of the bit in that section.
 * @return  True if section and bit are valid, false otherwise.
 * @note    Only 32-bit arrays are supported.
 */
bool calculateSectionBit(
    size_t arraySize,
    uint32_t sectionSize,
    uint32_t wordIndex, uint32_t bitIndex,
    uint32_t* section, uint32_t* bit
);

} // pinicore

#endif // PINICORE_UTILS_ARRAY_H