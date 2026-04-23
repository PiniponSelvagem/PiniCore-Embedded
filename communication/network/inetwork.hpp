/**
* @file		inetwork.hpp
* @brief	Interface for network API.
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

#ifndef PINICORE_COMM_INETWORK_H
#define PINICORE_COMM_INETWORK_H

#include <stdint.h>
#include <Client.h>

namespace pinicore {

enum ENetworkType : uint8_t {
    NET_WIFI,
    NET_MOBILE,
};

class INetwork {
    public:
        virtual ~INetwork() = default;

        /**
         * @brief   Keeps the network alive and tries to reconnect if network state dropped.
         * @note    Call this function periodically to maintain the network alive.
         */
        virtual void maintain() = 0;

        /**
         * @brief   Connect to the assigned network.
         * @return  'True' if able to connect.
         */
        virtual bool connect() = 0;

        /**
         * @brief   Disconnect from the current network.
         */
        virtual void disconnect() = 0;

        /**
         * @brief   Turn on the network device.
         */
        virtual void enable() = 0;

        /**
         * @brief   Turn off the network device.
         */
        virtual void disable() = 0;

        /**
         * @brief   Get the type of this network interface.
         * @return  Enum value of the network type.
         */
        virtual ENetworkType getType() const = 0;

        /**
         * @brief   Get the Client network object.
         * @return  Pointer to Client object.
         */
        virtual Client* getClient() = 0;

        /**
         * @brief   Get the name of the current connected network.
         * @return  Pointer to a null terminated char array with the name of the currently connected network.
         */
        virtual const char* getName() const = 0;

        /**
         * @brief   Get signal strength of the current connected network.
         * @return  Value in dB.
         */
        virtual int getSignalStrength() const = 0;

        /**
         * @brief   Get connection state.
         * @return  'true' if connected 'false' otherwise.
         */
        virtual bool isConnected() const = 0;
};

} // pinicore

#endif /* PINICORE_COMM_INETWORK_H */
