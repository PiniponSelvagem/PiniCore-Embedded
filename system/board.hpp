/**
* @file		board.hpp
* @brief	Get information tied to the current board.
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

#ifndef PINICORE_SYSTEM_BOARD_H
#define PINICORE_SYSTEM_BOARD_H

namespace pinicore {

/**
 * @brief Get device unique identifier, that is equal to the WiFi module MAC ADDRESS.
 * @return Char array null terminated with the device unique identifier string.
 */
const char* getUniqueId();

/**
 * @brief Asks the SOC what was the last reset reason, if was manual, return true.
 * @return True when was a manual reset.
 */
bool wasLastResetManual();

/**
 * @brief Asks the SOC if the last reset was fatal due to bad coding skills.
 * @return If you wrote bad code and this returned TRUE, you know what to do ;)
 */
bool wasLastResetFatal();

} // pinicore

#endif // PINICORE_SYSTEM_BOARD_H