/**
* @file		watchdog.hpp
* @brief	Watchdog simplifed API.
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

#ifndef _PINICORE_WATCHDOG_H_
#define _PINICORE_WATCHDOG_H_

#include <stdint.h>

/**
 * @brief Configure the internal Task watchdog.
 * @param timeout Timeout in seconds
 */
void watchdogSetup(uint32_t timeout);

/**
 * @brief Enable the internal Task watchdog for current thread.
 */
void watchdogEnable();

/**
 * @brief Disable the internal Task watchdog for current thread.
 */
void watchdogDisable();

/**
 * @brief Inform the internal Task watchdog that the code is still running.
 */
void watchdogIamAlive();

#endif // _PINICORE_WATCHDOG_H_