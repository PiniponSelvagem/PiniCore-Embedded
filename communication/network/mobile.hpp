/**
* @file		mobile.hpp
* @brief	Mobile network API.
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

#ifndef PINICORE_COMM_MOBILE_H
#define PINICORE_COMM_MOBILE_H

#include "inetwork.hpp"
#include "piniconst.hpp"

#ifndef PINICORE_CONFIG_H
    #include "piniconfig.hpp"
#endif

#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

// Define the serial console for debug prints, if needed
//#define TINY_GSM_DEBUG Serial

#include <TinyGsmClient.h>

namespace pinicore {

#define PINICORE_MOBILE_APN_SIZE_MAX            64
#define PINICORE_MOBILE_SIMCARD_PIN_SIZE_MAX    8
#define PINICORE_MOBILE_IMSI_SIZE_MAX           16
#define PINICORE_MOBILE_PROVIDER_SIZE_MAX       64

/**
 * Currently only the SIM800 has been tested.
 * Other mobile modules might require adjustments to the code and refactoring.
 */
class MobileComm : public INetwork {
    public:
        /**
         * @brief	Initializes the mobile network interface.
         * @param   pinPowerOn Modem power on pin.
         * @param   pinPowerKey Modem power key pin.
         * @param   pinPowerReset Modem reset pin.
         * @param   pinPowerTx Modem Tx pin.
         * @param   pinPowerRx Modem Rx pin.
         * @note	This function must be called prior to any other MobileComm functions.
         */
        void init(uint8_t pinPowerOn, uint8_t pinPowerKey, uint8_t pinReset, uint8_t pinTx, uint8_t pinRx);

        /**
         * @brief   Configure Mobile connection or replace the current configuration with a new one.
         * @param   apn Cell APN identifier, if empty/blank will auto search based on SIM card provider.
         * @param   simcardPin SIM card unlocking pin.
         * @note    If already connected, requires \ref 'disconnect' and then \ref 'connect' for new configuration to take place.
         */
        void config(const char* apn, const char* m_simcardPin);

        /**
         * @brief   Keeps the Mobile connection alive, reconnects if disconnected.
         * @note    Call this function periodically to maintain the network alive.
         */
        void maintain() override;

        /**
         * @brief   Connect to the assigned Mobile network.
         * @return  'True' if able to connect.
         */
        bool connect() override;

        /**
         * @brief   Disconnect from the current Mobile network.
         */
        void disconnect() override;

        /**
         * @brief   Turn on the Mobile device.
         */
        void enable() override;

        /**
         * @brief   Turn off the Mobile device.
         */
        void disable() override;

        /**
         * @brief   Get the type of this network interface.
         * @return  Enum value of the network type.
         */
        inline ENetworkType getType() const override { return ENetworkType::NET_MOBILE; };

        /**
         * @brief   Get the Client network object.
         * @return  Pointer to Client object.
         */
        inline Client* getClient() override { return m_client; };

        /**
         * @brief   Get the name of the current connected network.
         * @return  Pointer to a null terminated char array with the name of the currently connected network.
         */
        inline const char* getName() const override { return m_provider; }

        /**
         * @brief   Get the IMSI of the SIM card.
         * @return  Pointer to a null terminated char array with the IMSI.
         */
        inline const char* getIMSI() const { return m_imsi; }

        /**
         * @brief   Get signal strength of the current connected network.
         * @return  Value in dB.
         */
        inline int getSignalStrength() const override { return m_modem.getSignalQuality(); }

        /**
         * @brief   Get connection state.
         * @return  'true' if connected 'false' otherwise.
         */
        inline bool isConnected() const override { return m_modem.isNetworkConnected(); };


    protected:
        uint8_t p_pinPowerOn;
        uint8_t p_pinPowerKey;
        uint8_t p_pinReset;
        uint8_t p_pinTx;
        uint8_t p_pinRx;

        bool p_connectedOnce = false;   // True when module connected at least once, meaning hardware is working.


    private:
        bool m_isActive = false;    // If true, 'connect' was called and should reconnect if lost connection

        char m_apn[PINICORE_MOBILE_APN_SIZE_MAX];
        char m_simcardPin[PINICORE_MOBILE_SIMCARD_PIN_SIZE_MAX];
        char m_imsi[PINICORE_MOBILE_IMSI_SIZE_MAX];
        char m_provider[PINICORE_MOBILE_PROVIDER_SIZE_MAX];
        TinyGsmClient *m_client = &m_gsmClient;

        mutable TinyGsm m_modem   = TinyGsm(SerialAT);
        TinyGsmClient m_gsmClient = TinyGsmClient(m_modem, 0);
};

} // pinicore

#endif /* PINICORE_COMM_MOBILE_H */
