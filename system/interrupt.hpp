/**
* @file		interrupt.hpp
* @brief	Interrupts made easy.
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

#ifndef _PINICORE_INTERRUPT_H_
#define _PINICORE_INTERRUPT_H_

/**
 * @brief   Automatically disable interrupts during the lifetime of the usage of this class.
 * @note    Just declare a variable with with this class inside a block of code.
 *          When that block of code ends, deconstruct will be called and interrupts will be
 *          reenabled.
 */
class AutoDisableInterrupt {
    public:
        AutoDisableInterrupt();
        ~AutoDisableInterrupt();
};

#endif // _PINICORE_INTERRUPT_H_