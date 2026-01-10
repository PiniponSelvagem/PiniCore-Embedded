/**
* @file		irelays.hpp
* @brief	Interface API for different types of hardware relays.
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

#ifndef _PINICORE_IO_IRELAYS_H_
#define _PINICORE_IO_IRELAYS_H_

#include <stdint.h>
#include <functional>
#include <Adafruit_MCP23X17.h>

#define RELAYS_MAX 256  // Maximum number of total relays supported by the library.
#define RELAYS_STORAGE_BIT_SIZE (sizeof(uint32_t)*8)    // Size in bits of the type uses to store relays state.
#define RELAYS_STATE_SIZE_MAX   (RELAYS_MAX / RELAYS_STORAGE_BIT_SIZE) // Size of the \ref 'm_relaysState' array.

// user callbacks
typedef std::function<void(uint8_t module, uint8_t relay, bool state)> RelaysOnRelayCallback; // This module changed a relay to a new state.
typedef std::function<void(uint8_t module, bool state)> RelaysOnModuleCallback;               // This module connection status cahnged to a new state.

class IRelays {
    public:
        virtual ~IRelays() = default;

        /**
         * @brief   Invalidates all modules and relays state.
         * @note    Useful for example to ensure all relays states are reported to MQTT upon reconnect.
         */
        void invalidateAll();

        /**
         * @brief   Set a relay to a new state.
         * @param   module Module index.
         * @param   relay Relay index inside the module.
         * @param   state New state to set the relay to.
         * @return  True if relay state was changed, false otherwise.
         */
        bool set(uint8_t module, uint8_t relay, bool state);

        /**
         * @brief   Get the current state of a relay.
         * @param   module Module index.
         * @param   relay Relay index inside the module.
         * @return  Current active state of the relay.
         */
        bool get(uint8_t module, uint8_t relay);

        /**
         * @brief   Get the number of relays currently active.
         * @return  Count of how many relays are set to 'true' state in the system.
         */
        const int getActiveCount();

        /**
         * @brief   Get the maximum number of modules supported by the hardwre.
         * @return  Count of max modules supported.
         */
        const int getModulesMaxSupported();

        /**
         * @brief   Get number of relays per module.
         * @return  Count of relays per module.
         */
        const int getRelaysPerModule();

        /**
         * @brief   Check if a module is connected.
         * @param   module Module index.
         * @return  Module connected status.
         */
        virtual bool isModuleConnected(uint8_t module) = 0;

        /**
         * @brief   Registers a callback function to be called when a relay changes state.
         * @param   callback The callback function with the signature void(uint8_t module, uint8_t relay, bool state) to be registered.
         */
        void onRelay(RelaysOnRelayCallback callback);
        /**
         * @brief   Registers a callback function to be called when a module changes its connection state.
         * @param   callback The callback function with the signature void(uint8_t module, bool state) to be registered.
         */
        void onModule(RelaysOnModuleCallback callback);
    

    protected:
        /**
         * @brief   Safely call 'onModule' callback.
         * @param   module Module index.
         * @param   state New connection status of the module.
         */
        void _onModule(uint8_t module, bool state);

        /**
         * @brief   Set the internal relays state data structure of a module to all off.
         * @param   module Module index.
         * @note    Useful example: module 3 is disconnected, call this function to reset the
         *          library internal state for this module to all off.
         */
        void resetModuleState(uint8_t module);


        uint8_t p_modules;          // Number of modules configured.
        uint8_t p_relaysPerModule;  // Number of relays per module.


    private:
        /**
         * @brief   Safely call 'onRelay' callback.
         * @param   module Module index.
         * @param   relay Relay index inside the module.
         * @param   state New state of the relay.
         */
        void _onRelay(uint8_t module, uint8_t relay, bool state);

        /**
         * @brief   Initializes the relay modules.
         * @warning This function is required to initialize the variables 'p_modules' and 'p_relaysPerModule'.
         */
        virtual void initModules() = 0;

        /**
         * @brief   Sends the command to the hardware.
         * @param   module The module id.
         * @param   relay The relay id in the module.
         * @param   state The state of the relay to be set to.
         * @return  True if the relay changed to the new state, false otherwise.
         * @note    It is not necessary for this function to check if 'module' and 'relay' are within valid
         *          range, since \ref 'IRelays' class will do that check before calling it.
         */
        virtual bool setHardware(uint8_t module, uint8_t relay, bool state) = 0;

        /**
         * @brief   Updates the relays active count variable.
         */
        void updateActiveCount();

        /**
         * @brief   Calcuate the bit position on th e \ref 'm_relaysState' of the relay for a particular module.
         * @param   module The index of the module.
         * @param   relay The index of the relay in that module.
         * @param   wordIndex Return value, the calculated index in the \ref 'm_relaysState'.
         * @param   bitIndex Return value, the calculated bit index in the wordIndex.
         * @return  True if is valid, false otherwise.
         * @warning Does NOT validate if module and relay are valid, only if the resulting globalIndex is valid
         *          for the further internal calculations.
         */
        bool calculateRelayIndex(uint8_t module, uint8_t relay, uint16_t* wordIndex, uint8_t* bitIndex);

        /**
         * @brief   Prints to serial using LOG the entire 'm_relaysState' array in binary.
         */
        void debugRelaysState();


        int m_relaysActiveCount = 0;
        uint32_t m_relaysState[RELAYS_STATE_SIZE_MAX];

        /** Callbacks **/
        RelaysOnRelayCallback  m_onRelayCallback;
        RelaysOnModuleCallback m_onModuleCallback;
};

#endif // _PINICORE_IO_IRELAYS_H_