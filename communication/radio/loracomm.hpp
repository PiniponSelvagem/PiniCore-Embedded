/**
* @file     loracomm.hpp
* @brief    LoRa communication layer.
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

#ifndef _PINICORE_LORACOMM_H_
#define _PINICORE_LORACOMM_H_

#include "drivers/communication/lora.hpp"

#define LORACOMM_INVALID_TAGID      UINT8_MAX   // Reserved tagId that is used to identify that a 'onReceive' callback is not set for that index.

#define LORACOMM_ONRECEIVE_SIZE_MAX 16  // Maximum number of tagIds that can be "subscribed". I want to avoid using 'malloc'.

#define LORACOMM_SEND_PAYLOAD_MAX   (LORA_PACKET_MAX_SIZE-sizeof(LoRaHeader_t))   // Maximum number of bytes that can be sent, excluding header.
#define LORACOMM_SEND_QUEUE_MAX     16  // Maximum number of payloads that can be on the send queue at one time.
#define LORACOMM_SEND_RETRY_MAX     3   // Maximum number of retries before dropping if no ACK reply, when required.

#define LORACOMM_DELAY_MAX_MS           3000    // Max calculated value by the signal delay calculation function
#define LORACOMM_RETRY_IN_MS            LORACOMM_DELAY_MAX_MS   // Time between retries
#define LORACOMM_TIMEOUT_MS             ((LORACOMM_DELAY_MAX_MS+LORACOMM_RETRY_IN_MS)*LORACOMM_SEND_RETRY_MAX)  // Total time after which Terminal can be considered not reachable, new payload must be created

typedef uint64_t radioid_t;

//user callbacks
typedef std::function<void(radioid_t radioId, const uint8_t* payload, size_t size, int rssi, float snr)> LoRaOnReceiveCallback;  // payload -> content only, size -> content only

typedef struct {
    uint32_t bytesSent;
    uint32_t bytesReceived;
    uint32_t packetsSent;
    uint32_t packetsReceived;
} LoRaStatistics_t;

typedef struct {
    uint8_t tagId = LORACOMM_INVALID_TAGID;
    LoRaOnReceiveCallback callback;
} LoRaOnReceiveCallback_t;

#define LORACOMM_FLAG_IDX_IS_TERMINAL 0
#define LORACOMM_FLAG_IDX_REQUIRE_ACK 1
#define LORACOMM_FLAG_IDX_IS_ACK      2
typedef struct {
                /**
                 * Payload checksum, excluding the header.
                 * Excluding the header, allows for new flags and the usage of teh reserded bytes.
                 * This way, controllers in older firmware can be supported for longer.
                 * 
                 * These bytes are also used to ACK a payload received, sued like a payload unique identifier.
                 * The way that is used is:
                 * 1 - controller 1 sends a payload with checksum 'xpto' and says that its payload requires ACK.
                 * 2 - controller 2 receives the payload, validates the checksum.
                 * 3 - controller 2 sends a reply ACK payload where its header is the usual and the content is only the 'xpto' checksum.
                 * 4 - controller 1 receives, validates the payload checksum and then gets the 'xpto' cheksum and marks the payload transaction as completed.
                 */
    uint32_t    checksum;
                /**
                 * bit 0 -> sender is Terminal (not Gateway)
                 * bit 1 -> requires ACK
                 * bit 2 -> is ACK of a received payload from a particular 'radioId' of a particular 'checksum' 
                 * bit [3..7] -> reserved
                 * 
                 * Example:
                 * 0b0000_0101
                 * - payload is from a Terminal
                 * - this payload does NOT require an ACK reply/payload
                 * - is an ACK reply/payload to the 'radioId' found in this header and to the 'checksum' found in the content section
                 */
    uint8_t     flags;
    uint8_t     tagId;
    uint16_t    reserved = 0;   // Reserved: 2 bytes padding.
    radioid_t   radioId;
} LoRaHeader_t;

typedef struct {
    uint8_t     retryCount;
    uint64_t    nextRetryAt;
    size_t      payloadSize;    // if == 0, then assume this element in the 'm_sendQueue' is empty
    uint8_t     payload[LORA_PACKET_MAX_SIZE]; // Includes header, which can be accessed by using by casting this pointer to 'LoRaHeader_t*'.
} LoRaSend_t;

#define LORACOMM_SIGNAL_QUALITY_COUNT_MAX 64
typedef struct {
    uint64_t lastUpdateAt;
    radioid_t radioId;
    int rssi;
    float snr;
} LoRaSignalQuality_t;

typedef struct {
    bool isTerminal;
    bool requireAck;
    bool isAck;
} LoRaFlags_t;


class LoRaComm {
    public:
        /**
         * @brief   Initialize the LoRa communication layer.
         * @param   pinMOSI SPI data input pin.
         * @param   pinMISO SPI data output pin.
         * @param   pinSCLK SPI clock pin.
         * @param   pinCS Chip select pin.
         * @param   pinReset Reset pin.
         * @param   pinDIO0 Digital IO 0 pin.
         * @param   carrierFrequency LoRa carrier frequency, see comment about 'usable radio frequencies' comment on the top of the LoRaTxRx class.
         * @return  True if hardware found and initialized, false otherwise.
         * @note	This function must be called prior to any other LoraComm functions.
         */
        bool init(
            uint8_t pinMOSI, uint8_t pinMISO, uint8_t pinSCLK, uint8_t pinCS,
            uint8_t pinReset, uint8_t pinDIO0,
            uint16_t carrierFrequency
        );

        /**
         * @brief   Set the role in the LoRa communication.
         * @param   isTerminal True when controller is a 'Terminal', false when is a 'Gateway'; same analogy as a cellular network.
         * @param   terminalRadioId RadioId, only used if 'isTerminal' == true, ignored if Gateway. Used to discard payloads that do not belong to this Terminal.
         * @note    If not called, the role will be default to Gateway, 'isTerminal==false'.
         */
        void setRole(bool isTerminal, radioid_t terminalRadioId);

        /**
         * @brief   Value used by checksum calculation.
         * @param   phrase A value known by both parties to further improve data validation, if '0' then phrase is not added to checksum calculation.
         */
        void setCryptoPhrase(uint8_t phrase);

        /**
         * @brief   Control how long each symbol is transmitted.
         * @param   sf Spreading factor range: [6,12], if outside will adjust to nearest value.
         * @note    Lower Spreading Factor:  shorter range, higher data rate.
         *          Higher Spreading Factor: longer range, lower data rate.
         */
        void setSpreadingFactor(uint8_t sf) { m_lora.setSpreadingFactor(sf); }

        /**
         * @brief   Control how loud to transmit.
         * @param   power Power in dBm, range: [0, 20].
         * @note    Lower power:  shorter range, higher battery life.
         *          Higher power: longer range, lower battery life.
         */
        void setTxPower(uint8_t power) { m_lora.setTxPower(power); }

        /**
         * @brief   Control communication bandwidth.
         * @param   bandwidth Bandwidth selection from \ref 'ELoRaBandwidth'.
         * @note    Lower bandwidth:  longer range, lower data rate, longer airtime.
         *          Higher bandwidth: shorter range, higher data rate, shorter airtime.
         */
        void setBandwidth(ELoRaBandwidth bandwidth) { m_lora.setBandwidth(bandwidth); }

        /**
         * @brief   Get current spreading factor.
         * @return  Spreading factor value range: [6,12].
         */
        inline const uint8_t getSpreadingFactor() { return m_lora.getSpreadingFactor(); }

        /**
         * @brief   Get current transmit power.
         * @return  Transmit power value range: [0,20].
         */
        inline const uint8_t getTxPower() { return m_lora.getTxPower(); }

        /**
         * @brief   Get current bandwidth.
         * @return  Bandwidth in Hz.
         */
        inline const ELoRaBandwidth getBandwidth() { return m_lora.getBandwidth(); }

        /**
         * @brief   Keeps the LoRa communication alive, if new payload, then calls the appropriate user callback for it.
         * @note    Call this function periodically to parse new received messages.
         */
        void maintain();

        /**
         * @brief   Idle/Standby the LoRa communication and device.
         */
        void enable();

        /**
         * @brief   Sleep the LoRa communication and device.
         */
        void disable();

        /**
         * @brief   Get LoRa communication and device state.
         * @return  True if on idle/standby, false if on sleep.
         */
        inline const bool isEnabled() { return m_lora.isEnabled(); }

        /**
         * @brief   Registers a callback function to be called when the LoRa communication receives a sepecific tagId.
         * @param   tagId Identifies the type of the payload.
         * @param   callback The callback function with the signature void(const uint8_t* payload, size_t size, int rssi, float snr) to be registered.
         * @return  True if there was space in the internal 'm_onReceiveCallbacks' array to add the tagId, false if full and could not be added.
         * @note    Calling this function for same tagId will replace old callback.
         */
        bool onReceive(uint8_t tagId, LoRaOnReceiveCallback callback);

        /**
         * @brief   Unregisters a callback function from being called when the LoRa communication receives a sepecific tagId.
         * @param   tagId The tagId.
         */
        void removeOnReceive(uint8_t tagId);

        /**
         * @brief   Queues a payload to be sent over LoRa.
         * @param   radioId Radio identifier, also known as controller 'serial'.
         * @param   tagId Identifies the type of the payload.
         * @param   requireAck True if should be acknowledged and retry if necessary, false send blindly once.
         * @param   payload Payload to be sent.
         * @param   size Size of the payload, up to \ref 'LORACOMM_SEND_PAYLOAD_MAX', if above will truncate.
         * @return  True if payload was queued for send, false if unable because send queue is full.
         */
        bool send(radioid_t radioId, uint8_t tagId, bool requireAck, const uint8_t* payload, size_t size);

        /**
         * @brief   Get current LoRa hardware and communication statistics.
         * @param   stats Pointer to struct that will place the statistics into.
         */
        void getStatistics(LoRaStatistics_t* stats);


    private:
        /**
         * @brief   Validate, identify the tagId, and then safely call 'onReceive' callback.
         * @param   payload Pointer to payload received.
         * @param   rssi Signal strenght.
         * @param   snr Signal to noise ratio.
         */
        void _onReceive(const uint8_t* payload, size_t size, int rssi, float snr);
        
        /**
         * @brief   Queues a payload to be sent over LoRa.
         * @param   radioId Radio identifier, also known as controller 'serial'.
         * @param   tagId Identifies the type of the payload.
         * @param   requireAck True if should be acknowledged and retry if necessary, false send blindly once.
         * @param   isAck True if this is an acknowledging payload, false otherwise.
         * @param   payload Payload to be sent.
         * @param   size Size of the payload, up to \ref 'LORACOMM_SEND_PAYLOAD_MAX', if above will truncate.
         * @return  True if payload was queued for send, false if unable because send queue is full.
         */
        bool _send(radioid_t radioId, uint8_t tagId, bool requireAck, bool isAck, const uint8_t* payload, size_t size);

        /**
         * @brief   Queues a acknowledge payload to be sent over LoRa.
         * @param   radioId Radio identifier, also known as controller 'serial'.
         * @param   tagId Identifies the type of the payload.
         * @param   checksumOfReceived Checksum of the payload content received that requires acknowledge.
         * @return  True if payload was queued for send, false if unable because send queue is full.
         */
        bool sendAck(radioid_t radioId, uint8_t tagId, uint32_t checksumOfReceived);

        /**
         * @brief   Updates the signal quality data structure with latest data.
         * @param   radioId RadioId of the controller that sent the payload.
         * @param   rssi Signal strenght.
         */
        void updateSignalQuality(radioid_t radioId, int rssi);

        /**
         * @brief   Calculate the amount of ms that should be used to delay a paylaod send, based on signal quality.
         * @param   radioId RadioId of the controller that want to send the payload to.
         * @return  Amount of ms that should be delayed before sending the payload.
         * @note    Allows for better payload / air management.
         */
        uint32_t calculateSendDelayFromSignalQuality(radioid_t radioId);

        /**
         * @brief   Add to the send queue a payload to be sent.
         * @param   payload The entire payload, including header which can be accessed by casting this pointer to 'LoRaSend_t*'.
         * @param   payloadSize Size in bytes of the entire payload, including header.
         * @param   delay How long in millis to delay the send of this payload.
         * @return  True if there was space and was added, false if queue is full of payload size above 'LORA_PACKET_MAX_SIZE'.
         */
        bool queueSendAdd(uint8_t* payload, size_t payloadSize, uint32_t delay);

        /**
         * @brief   Remove a payload from the send queue.
         * @param   sendElement Pointer to LoRaSend_t.
         */
        void queueSendRemove(LoRaSend_t* sendElement);

        /**
         * @brief   Get the next payload ready to be sent, meaning that the appropriate time was reached.
         * @return  Pointer to LoRaSend_t, NULL if send queue is empty.
         */
        LoRaSend_t* queueSendGetReady();

        /**
         * @brief   Search for payload in the send queue that is for 'radioId' and with 'checksum'.
         * @param   radioId To who the payload is for, radio identifier, also known as controller 'serial'.
         * @param   checksum Checksum of the payload content.
         * @return  Pointer to LoRaSend_t, NULL if not found in the send queue.
         */
        LoRaSend_t* queueSendFind(radioid_t radioId, uint32_t checksum);

        /**
         * @brief   Manages the send queue by sending to hardware the next ready payload, and managing ACKs.
         */
        void manageQueueSend();

        /**
         * @brief   Decode header flags and place then in a struct for easy usage.
         * @param   encodedFlags Byte with encoded header flags.
         * @param   flags Pointer to struct where to place the decoded flags.
         */
        void flagsDecode(uint8_t encodedFlags, LoRaFlags_t* flags);

        /**
         * @brief   Encode flags to a byte to be sent in the header.
         * @param   flags Pointer to struct where to get the flags to encode.
         * @return  Byte that has the encoded header flags.
         */
        uint8_t flagsEncode(LoRaFlags_t* flags);


        LoRaTxRx m_lora;             // Hardware used for lora commuincation.
        uint8_t m_cryptoPhrase;      // Value used to add to the checksum calculation, if '0' then it will not be used and normal checksum will be calculated.
        bool m_isTerminal;           // True when controller is a 'Terminal', false when is a 'Gateway'. Same analogy as a cellular network. 
        radioid_t m_terminalRadioId; // If isTerminal, then filter payloads only directed to my radioId.

        /**
         * @brief   Contains signal quality of previously received payloads for specific 'radioId's.
         *          This helps to decide how long should the controller wait for a ACK.
         *          Only used in Gateway mode, this is so that 2 Gateways do not colide with each other when sending payloads.
         *          The one that received the weakest signal should wait longer, giving time to the stronger to send, and if the Terminal
         *          replies with ACK to that in time, both will mark as sent successfully (even thought that the weakest one never sent the payload).
         * @warning The logic that uses this data structure, intentionally does not support 'removing' elements (aka setting 'lastUpdateAt' = 0).
         *          If that is done, duplicated elements can start appearing in this data structure.
         */
        LoRaSignalQuality_t m_signalQuality[LORACOMM_SIGNAL_QUALITY_COUNT_MAX] = {};

        /**
         * @brief   Queue that contains payloads to be sent.
         *          If 'payloadSize' of an element is == 0, then assume that payload was already handled and that element being
         *          free for next usage.
         */
        LoRaSend_t m_sendQueue[LORACOMM_SEND_QUEUE_MAX] = {};   // Queue that contains the payloads to be sent.

        /** Queue with sents that require ACK **/
        /**
         * TODO:
         * - this queue should have the RSSI and SNR of the received payload that is going to reply with ACK
         * - if is a Terminal, it should just ACK
         * - if it is a Gateway, it should calculate how long it should wait until it sends a reply ACK
         *      - stronger RSSI and stronger SNR should reply faster
         *      - weaker RSSI and weaker SNR should reply slower
         *      - I think that SNR should have higher bias because the payload was received clearer compared to a weaker SNR
         *      - TIMEOUT should take this time into account, have a margin, and be aware of the payload air time
         *      - It should listen to other Gateway ACK replies, and if is ACK for same Terminal, then drop it and mark is as ACKed
         * - ACK is made based on both radioId and checksum
         */

        /** Callbacks **/
        LoRaOnReceiveCallback_t m_onReceiveCallbacks[LORACOMM_ONRECEIVE_SIZE_MAX] = {};
};

#endif // _PINICORE_LORACOMM_H_