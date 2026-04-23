/**
* @file     lora.hpp
* @brief    LoRa driver API made simple.
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

#ifndef _PINICORE_LORA_H_
#define _PINICORE_LORA_H_

#include <stdint.h>
#include <functional>

/**
 * Note: This table was taken from ChatGPT, take it with a grain of salt and just as a general idea.
 *  SNR (dB)  |  Signal Quality
 * -----------+-----------------
 *  above 10  |    Excellent
 *   6 to 10  |      Good
 *   0 to 6   |      Fair
 *  -6 to 0   |      Poor
 * -10 to -6  |    Very Poor
 * below -10  |      Bad
 */

/**
 * Usable radio frequencies based on country/region: (These frequencies are for the 868/915MHz models)
 * - [863, 873] : Europe
 * - [902, 928] : North America
 * - [915, 928] : South America
 * - [865, 867] : India
 * - [915, 928] : Asia
 */

/**
 * @brief   LoRa bandwidth definition in Hz.
 */
enum ELoRaBandwidth : long {
    LR_BW_7_8_KHZ   = 7800,
    LR_BW_10_4_KHZ  = 10400,
    LR_BW_15_6_KHZ  = 15600,
    LR_BW_20_8_KHZ  = 20800,
    LR_BW_31_25_KHZ = 31250,
    LR_BW_41_7_KHZ  = 41700,
    LR_BW_62_5_KHZ  = 62500,
    LR_BW_125_KHZ   = 125000,
    LR_BW_250_KHZ   = 250000,
    LR_BW_500_KHZ   = 500000
};

// user callbacks
typedef std::function<void(const uint8_t* payload, size_t size, int rssi, float snr)> LoRaTxRxOnReceiveCallback; // Callback for on receive a message

#define LORA_INIT_DEFAULT_SF    7
#define LORA_INIT_DEFAULT_POWER 20
#define LORA_INIT_DEFAULT_BAND  ELoRaBandwidth::LR_BW_125_KHZ

#define LORA_PACKET_MAX_SIZE            255     // Taken from 'LoRa' -> 'MAX_PKT_LENGTH'
#define LORA_RECEIVED_PACKET_MAX_COUNT  8       // Max number of packets received that can queue before start dropping.

typedef struct {
    size_t size;
    int rssi;
    float snr;
    uint8_t payload[LORA_PACKET_MAX_SIZE];
} LoRaReceived_t;

class LoRaTxRx {
    public:
        /**
         * @brief	Initializes the lora hardware.
         * @param   pinMOSI SPI data input pin.
         * @param   pinMISO SPI data output pin.
         * @param   pinSCLK SPI clock pin.
         * @param   pinCS Chip select pin.
         * @param   pinReset Reset pin.
         * @param   pinDIO0 Digital IO 0 pin.
         * @param   carrierFrequency LoRa carrier frequency, see comment about 'usable radio frequencies' comment on the top of the LoRaTxRx class.
         * @return  True if hardware found and initialized, false otherwise.
         * @note	This function must be called prior to any other LoRaTxRx functions.
         *          Also calls \ref 'setSpreadingFactor', \ref 'setTxPower' and \ref 'setBandwidth' with default values, 'LORA_INIT_DEFAULT_x'.
         *          If different configuration is required, call their respective function. 
         */
        bool init(
            uint8_t pinMOSI, uint8_t pinMISO, uint8_t pinSCLK, uint8_t pinCS,
            uint8_t pinReset, uint8_t pinDIO0,
            uint16_t carrierFrequency
        );

        /**
         * @brief   Control how long each symbol is transmitted.
         * @param   sf Spreading factor range: [6,12], if outside will adjust to nearest value.
         * @note    Lower Spreading Factor:  shorter range, higher data rate.
         *          Higher Spreading Factor: longer range, lower data rate.
         */
        void setSpreadingFactor(uint8_t sf);

        /**
         * @brief   Control how loud to transmit.
         * @param   power Power in dBm, range: [0, 20].
         * @note    Lower power:  shorter range, higher battery life.
         *          Higher power: longer range, lower battery life.
         */
        void setTxPower(uint8_t power);

        /**
         * @brief   Control communication bandwidth.
         * @param   bandwidth Bandwidth selection from \ref 'ELoRaBandwidth'.
         * @note    Lower bandwidth:  longer range, lower data rate, longer airtime.
         *          Higher bandwidth: shorter range, higher data rate, shorter airtime.
         */
        void setBandwidth(ELoRaBandwidth bandwidth);

        /**
         * @brief   Get current spreading factor.
         * @return  Spreading factor value range: [6,12].
         */
        inline const uint8_t getSpreadingFactor() { return m_spreadingFactor; }

        /**
         * @brief   Get current transmit power.
         * @return  Transmit power value range: [0,20].
         */
        inline const uint8_t getTxPower() { return m_txPower; }

        /**
         * @brief   Get current bandwidth.
         * @return  Bandwidth in Hz.
         */
        inline const ELoRaBandwidth getBandwidth() { return m_bandwidth; }

        /**
         * @brief   Keeps the LoRa communication alive, if new message calls on receive callback.
         * @note    Call this function periodically to parse new received messages.
         */
        void maintain();

        /**
         * @brief   Idle/Standby the LoRa device.
         */
        void enable();

        /**
         * @brief   Sleep the LoRa device.
         */
        void disable();

        /**
         * @brief   Get LoRa device state.
         * @return  True if on idle/standby, false if on sleep.
         */
        inline const bool isEnabled() { return m_isActive; }

        /**
         * @brief   Send a payload over LoRa.
         * @param   payload The payload to be sent.
         * @param   size Size of the payload, max is \ref 'LORA_RECEIVED_PACKET_MAX_SIZE' and if above then rest is dropped and not send.
         */
        void send(const uint8_t* payload, size_t size);
        
        /**
         * @brief   Registers a callback function to be called when a message is received client.
         * @param   callback The callback function with the signature void(const uint8_t* payload, uint32_t length, int rssi, float snr) to be registered.
         */
        void onReceive(LoRaTxRxOnReceiveCallback callback);

        /**
         * @brief   Statistics: number of bytes sent.
         * @return  Number of bytes sent.
         */
        inline uint32_t statsBytesSent() { return m_statsBytesSent; }

        /**
         * @brief   Statistics: number of bytes received.
         * @return  Number of bytes received.
         */
        inline uint32_t statsBytesReceived() { return m_statsBytesReceived; }

        /**
         * @brief   Statistics: number of packets sent.
         * @return  Number of packets sent.
         */
        inline uint32_t statsPacketsSent() { return m_statsPacketsSent; }

        /**
         * @brief   Statistics: number of packets received.
         * @return  Number of packets received.
         */
        inline uint32_t statsPacketsReceived() { return m_statsPacketsReceived; }


    private:
        /**
         * @brief   Check if new packet received and place it in \ref 'm_packetReceived'.
         * @return  True if packet ready to be processed, false otherwise.
         */
        bool receive();

        /**
         * @brief   Safely call 'onReceive' callback.
         * @param   payload Pointer to received message.
         * @param   size Size of the message received.
         * @param   rssi Signal strenght.
         * @param   snr Signal to noise ratio.
         */
        void _onReceive(const uint8_t* payload, size_t size, int rssi, float snr);

        
        uint8_t m_spreadingFactor;
        uint8_t m_txPower;
        ELoRaBandwidth m_bandwidth;

        bool m_isActive = false;    // True if on idle/standby, false is on sleep.

        /** Receive payload handling variables **/
        LoRaReceived_t m_packetReceived;  // Packet received.

        /** Callbacks **/
        LoRaTxRxOnReceiveCallback m_onReceiveCallback = NULL;

        /** Statistics **/
        uint32_t m_statsBytesSent       = 0;
        uint32_t m_statsBytesReceived   = 0;
        uint32_t m_statsPacketsSent     = 0;
        uint32_t m_statsPacketsReceived = 0;
};

#endif // _PINICORE_STORAGE_H_