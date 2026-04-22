/**
* @file		crypto.hpp
* @brief	Checksum, encrypt and decrypt utils.
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

#ifndef _PINICORE_CRYPTO_H_
#define _PINICORE_CRYPTO_H_

#include "pinitypes.hpp"

/**
 * @brief   Calculate the checksum of a payload.
 * @param   data The data to calculate the checksum.
 * @param   size Size of the data.
 * @param   phrase A value known by both parties to further improve data validation, if '0' then phrase is not added to checksum calculation.
 */
uint32_t calculateChecksum(const uint8_t* data, size_t size, const uint8_t phrase = 0);

#endif // _PINICORE_CRYPTO_H_