/**
* @file     memory.hpp
* @brief    Helper functions to make some memory operations easier to handle.
* @author   PiniponSelvagem
*
* Copyright(C) PiniponSelvagem
*
***********************************************************************
* Software that is described here, is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
**********************************************************************/

#pragma once

#ifndef PINICORE_UTILS_MEMORY_H
#define PINICORE_UTILS_MEMORY_H

#include <stdint.h>
#include <stddef.h>

namespace pinicore {

/**
 * @brief   Get the ammount of bytes available for a continous allocation.
 * @return  Number of bytes available for the largest continous possible allocation.
 */
size_t mavailableLargest();

/**
 * @brief   Dynamically allocate memory up to a target size.
 * @param   pMemory Pointer for the allocated memory.
 * @param   targetBytes Number of bytes to allocate if possible, target allocation.
 * @param   dividerCap If cannot allocate the target size, will try to allocate totalAllocalableSpace/'dividerCap'.
 * @return  Number of bytes allocated.
 * @note    Explanation of 'dividerCap' with an example:
 *              total available = 4096 bytes
 *              dividerCap = 2
 *              ---> maximum allowed allocabable space = 4096/2 = 2048 bytes
 *              > If targetBytes = 2048, then it will allocate 2048 bytes
 *              > If targetBytes = 1024, then it will allocate 1024 bytes
 *              > If targetBytes = 3072, then it will only allocate 2048 bytes
 * @warning Because I know I will forget again... parameter 'pMemory' has to be casted like so:
 *          (void**)&buffer
 */
size_t mallocTarget(void** pMemory, size_t targetBytes, uint32_t dividerCap);

} // pinicore

#endif // PINICORE_UTILS_MEMORY_H