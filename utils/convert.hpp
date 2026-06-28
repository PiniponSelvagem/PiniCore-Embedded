/**
* @file		convert.hpp
* @brief	Convertions between data types and custom types.
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

#ifndef PINICORE_UTILS_CONVERTION_H
#define PINICORE_UTILS_CONVERTION_H

#include <stdint.h>
#include <stddef.h>

namespace pinicore {

/**
 * @brief   Converts hexadecimal string of any size to a sectioned 32-bit array.
 * @param   hexStr The hexadecimal string.
 * @param   hexLen The length of the hexadecimal string.
 * @param   section The index of the section.
 * @param   sectionSize The size of each section.
 * @param   array Pointer to the array.
 * @param   arraySize Size of the array.
 * @param   bitIndex Return value, the calculated bit index in the wordIndex.
 * @return  True if array was filled correctly, meaning the hex could fit the section. False otherwise.
 * @note    Section is a sub array that can be of any size.
 *          To use this function without sections, set 'sectionSize' equal to 'arraySize' and 'section' to 0.
 */
bool stringHex2Array(
    const char* hexStr, uint32_t hexLen,
    uint32_t section,   uint32_t sectionSize,
    uint32_t* array,    size_t arraySize
);

} // pinicore

#endif // PINICORE_UTILS_ARRAY_H