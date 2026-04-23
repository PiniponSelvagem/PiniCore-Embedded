/**
* @file		iota.hpp
* @brief	OTA interface API, implement it to support the desired web service.
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

#ifndef PINICORE_COMM_IOTA_H
#define PINICORE_COMM_IOTA_H

#include <stdint.h>
#include <functional>
#include <Client.h>

namespace pinicore {

enum EOTAUpdateStatus : uint8_t {
    OTA_FAILED,
    OTA_UPDATE_NOT_AVAILABLE,
    OTA_INSTALLED,
    OTA_DL_COMPLETE,
    OTA_DL_INCOMPLETE,
    OTA_CHECKSUM_MISMATCH,
    OTA_MEMORY_ERROR,
    OTA_CLIENT_ERROR
};

#define OTA_ONPROGRESS_SIGNATURE std::function<void(uint32_t downloadedBytes, uint32_t totalBytes)> // Called during update download.

#define OTA_SHA256_MAX_SIZE         32
#define OTA_SHA256_MAX_SIZE_CHAR    (OTA_SHA256_MAX_SIZE*2)

class IOTA {
    public:
        /**
         * @brief	OTA constructor.
         * @param	client Pointer to network client to be used for requests.
         * @param   currFirmware Current firmware version.
         * @param   serial Controller serial / uniqueID, optional for APIs that do not require it.
         */
        IOTA(Client* client, int currFirmware, const char* serial = NULL);

        /**
         * @brief   Set the firmware download progress callback.
         * @param   onProgress Function to be called during firmware download. Signature: 'onProgress(uint32_t downloadedBytes, uint32_t totalBytes)'.
         * @note    Configuring this callback is optional.
         */
        void setProgressCallback(OTA_ONPROGRESS_SIGNATURE onProgress);

        /**
         * @brief   Check for updates based on the current firmware version.
         * @return  Available update status.
         */
        virtual bool checkUpdate() = 0;

        /**
         * @brief   Download and install the available update.
         * @return  Update status code, if 'OTA_INSTALLED' then next reboot will execute new update.
         * @warning \ref 'checkUpdate' must be called before calling this function, otherwise 'OTA_UPDATE_NOT_AVAILABLE' will be the return value.
         */
        virtual EOTAUpdateStatus update() = 0;

        /**
         * @brief   Get the firmware version number available.
         * @return  Available firmware version number.
         * @note    This is only valid after calling \ref 'checkUpdate', if still -1, then no update available.
         */
        inline int getVersionAvailable() { return p_versionAvailable; }


    protected:
        /**
         * @brief   Callback called during firmware download.
         * @param   downloadedBytes Bytes downloaded so far.
         * @param   totalBytes Total size in bytes of the download.
         */
        void onProgress(uint32_t downloadedBytes, uint32_t totalBytes);

        /**
         * @brief   Get the Client network object.
         * @return  Pointer to Client object.
         */
        inline Client* getClient() { return m_client; };

        /**
         * @brief   Get the current firmware version.
         * @return  Currently installed firmware version.
         */
        inline int getCurrentFirmware() { return m_currFirmware; }

        /**
         * @brief   Get the system unique identifier.
         * @return  Pointer to char array with unique ID string.
         */
        inline const char* getUniqueID() { return m_serial; }

        /**
         * @brief   Start download process.
         * @param   client The communication client in use.
         * @param   updateMD5 MD5 found in the header of the reply.
         * @param   totalSize Expected firmware size found in the header of the reply.
         * @param   calculatedSHA256 At the end of the 
         */
        EOTAUpdateStatus download(
            Client* client, const char* updateMD5, uint32_t totalSize,
            char calculatedSHA256[OTA_SHA256_MAX_SIZE_CHAR]
        );

        EOTAUpdateStatus install(
            const char* updateMD5, const char* updateSHA256,
            char calculatedSHA256[OTA_SHA256_MAX_SIZE_CHAR]
        );


        int p_versionAvailable = -1;    // Firmware version available. Call 'checkUpdate' first, if still -1, then no update available. 


    private:
        /**
         * @brief   Abort update and free resources.
         */
        void abortAndFree();    

        Client *m_client;

        int m_currFirmware;
        const char* m_serial;

        OTA_ONPROGRESS_SIGNATURE m_onProgress;  // The callback called during download progress. Use \ref 'onProgress' since that will check if this function pointer is valid.
};

} // pinicore

#endif // PINICORE_COMM_IOTA_H