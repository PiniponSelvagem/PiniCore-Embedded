/**
* @file		rotaryencoder.hpp
* @brief	Contains the rotary encoder API.
* @author	PiniponSelvagem
*
* Based on the rotary encoder from MobiFlight-FirmwareSource.
* Also based on RotaryEncoder library from Matthias Hertel, http://www.mathertel.de/Arduino.
*
* Copyright(C) PiniponSelvagem
*
***********************************************************************
* Software that is described here, is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
**********************************************************************/

#pragma once

#ifndef PINICORE_IO_ROTARYENCODER_H
#define PINICORE_IO_ROTARYENCODER_H

#include <stdint.h>

namespace pinicore {

#define ROTENC_MAX 4000         // prevent internal position overflow
#define ROTENC_FAST_SPEED 5     // the inc/dec to add when during fast mode
#define ROTENC_FAST_TRIGGER 6   // number of detends that will trigger fast mode


enum RotaryEncoderType {
    T_11,
    T_00,
    T_00_11,
    T_10_01,
    T_00_10_11_01,
};

typedef struct {
    // Detent positions in the quadrature (by value, not position)
    bool    detents[4];

    // Bit shift to apply given the detent resolution of this encoder.
    //
    // Example: An encoder with 1 detent per quadrature cycle has a useful resolution of
    // 1/4 of the number of pulses so we can apply a simple bit shift of 2 to
    // determine the effective position of the encoder.
    uint8_t resolutionShift;
} encoderType;

class RotaryEncoder {
    public:
        /**
         * @brief   Initializes the rotary encoder.
         * @param   pin0: Either CLK or DT pin.
         * @param   pin1: Either CLK or DT pin.
         * @param   typeEncoder: The type of detent/s detected while not rotating.
         * @note	This function must be called prior to any other RotaryEncoder functions.
         */
        void init(uint8_t pin0, uint8_t pin1, RotaryEncoderType typeEncoder);

        /**
         * @brief   Checks and updates rotary encoder state.
         * @note    This function syncs the rotary encoder hardware position, with the software position. The delta is also updated.
         *          Can be called multiple times without calling \ref 'getPosition' or \ref 'getDelta'.
         */
        void pullEvents();

        /**
         * @brief   Reset position back to 0.
         */
        void resetPosition();

        /**
         * @brief   Get current position.
         * @return  Current rotary encoder position.
         */
        int16_t getPosition();

        /**
         * @brief   Get rotary encoder rotation delta since last call of this same function.
         * @return  Rotation delta since last call.
         * @note    This function will reset the rotation delta to 0.
         */
        int32_t getDelta();

    private:
        /**
         * @brief   Faster rotary encoder state update checker.
         * @note    Even thought calling this is more efficient than \ref 'pullEvents', it is private because it does not update delta rotation.
         */
        void tick();

        /**
         * @brief   Set a new position.
         * @param   newPosition: New position. Value between positive and negative \ref 'ROTENC_MAX'.
         */
        void setPosition(int16_t newPosition);

        uint8_t     m_pin0;
        uint8_t     m_pin1;
        int16_t     m_pos;
        uint8_t     m_typeEncoder;
        uint8_t     m_detentCounter;
        encoderType m_encoderType;
        int8_t      m_oldState;
        int16_t     m_position;         // Internal position (4 times m_positionExt)
        int16_t     m_positionExt;      // External position
        uint32_t    m_positionTime;     // time last position change was detected
        uint32_t    m_positionTimePrev; // time previous position change was detected
        uint32_t    m_lastFastDec;
        int32_t     m_delta;             // rotation delta since last getDelta call
};

} // pinicore

#endif /* PINICORE_IO_ROTARYENCODER_H */
