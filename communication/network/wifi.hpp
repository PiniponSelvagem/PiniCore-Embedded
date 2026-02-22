/**
* @file		wifi.hpp
* @brief	WiFi network API.
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

#ifndef _PINICORE_WIFI_H_
#define _PINICORE_WIFI_H_

#include "inetwork.hpp"
#include <WiFi.h>

#define WIFI_SSID_SIZE_MAX 32
#define WIFI_PASS_SIZE_MAX 64

/**
 * @brief WiFi configuration data.
 */
struct WiFiConfig {
    const char* ssid;   ///< SSID null terminated
    const char* pass;   ///< Password null terminated
};

class WiFiComm : public INetwork {
    public:
        /**
         * @brief	Initializes the wifi network interface.
         * @note	This function must be called prior to any other WiFiComm functions.
         */
        void init();

        /**
         * @brief   Configure WiFi connection or replace the current configuration with a new one.
         * @param   ssid WiFi SSID to be used to connect, null terminated.
         * @param   pass WiFi Password to be used to connect, null terminated.
         * @note    If already connected, requires \ref 'disconnect' and then \ref 'connect' for new configuration to take place.
         */
        void config(const char* ssid, const char* pass);

        /**
         * @brief   Configure WiFi AP the current configuration with a new one.
         * @param   ssid WiFi SSID to be used to connect WiFi AP, null terminated.
         * @param   pass WiFi Password to be used to connect WiFi AP, null terminated and optional.
         * @param   hidden WiFi AP cloaking state.
         * @note    If already connected, requires \ref 'apDisconnect' and then \ref 'apConnect' for new configuration to take place.
         */
        void configAP(const char* ssid, const char* pass = NULL, bool hidden = false);

        /**
         * @brief   Keeps the network alive and tries to reconnect if network state dropped.
         * @note    Call this function periodically to maintain the network alive.
         */
        void maintain() override;

        /**
         * @brief   Connect to the assigned WiFi network.
         * @return  'True' if able to connect.
         */
        bool connect() override;

        /**
         * @brief   Disconnect from the current WiFi network.
         */
        void disconnect() override;

        /**
         * @brief   Enable WiFi AP based on configuration.
         * @return  'True' if was able to turn on WiFi AP.
         */
        bool connectAP();

        /**
         * @brief   Disconnect WiFi AP.
         */
        void disconnectAP();

        /**
         * @brief   Get the number of connected stations to the AP.
         * @return  Stations count.
         */
        uint8_t stationsConnectedAP();

        /**
         * @brief   Turn on the WiFi device.
         */
        void enable() override;

        /**
         * @brief   Turn off the WiFi device.
         * @note    If WiFi AP is connected, it will disconnect it and the stations connected.
         */
        void disable() override;

        /**
         * @brief   Get the type of this network interface.
         * @return  Enum value of the network type.
         */
        inline ENetworkType getType() const override { return ENetworkType::NET_WIFI; };

        /**
         * @brief   Get the Client network object.
         * @return  Pointer to Client object.
         */
        inline Client* getClient() override { return &m_client; };

        /**
         * @brief   Get the name of the current connected network.
         * @return  Pointer to a null terminated char array with the name of the currently connected network.
         */
        inline const char* getName() const override { return m_ssid; }

        /**
         * @brief   Get signal strength of the current connected network.
         * @return  Value in dB.
         */
        inline int getSignalStrength() const override { return WiFi.RSSI(); }

        /**
         * @brief   Get connection state.
         * @return  'true' if connected 'false' otherwise.
         */
        inline bool isConnected() const override { return WiFi.isConnected(); };

        /**
         * @brief   Get AP connection state.
         * @return  'true' if connected 'false' otherwise.
         */
        inline bool isConnectedAP() const { return m_isActiveAP; };

        
    private:
        bool m_isActive = false;        // If true, 'connect' was called and should reconnect if lost connection
        WiFiClient m_client;

        char m_ssid[WIFI_SSID_SIZE_MAX];   ///< SSID used in 'connect'.
        char m_pass[WIFI_PASS_SIZE_MAX];   ///< PASS used in 'connect'.

        char m_ssidAP[WIFI_SSID_SIZE_MAX]; ///< SSID used in 'apConnect'.
        char m_passAP[WIFI_PASS_SIZE_MAX]; ///< PASS used in 'apConnect'.
        bool m_isHiddenAP;                 ///< AP hidden state used in 'apConnect'.

        bool m_isActiveStation = false; ///< True when WiFi using station mode.
        bool m_isActiveAP = false;      ///< True when WiFi AP is active.
   
        uint64_t m_connectionLostAt;    // Millis when last wifi connection.
        bool m_connectionLost;          // In lost connection state.
};

#endif /* _PINICORE_WIFI_H_ */