#include "iota.hpp"
#include "utils/time.hpp"
#include "utils/log.hpp"
#include "utils/memory.hpp"

#include <Update.h>
#include <mbedtls/md.h>

#define PINICORE_TAG_OTA    "pcore_iota"

/**
 * @brief   Maximum bytes to read per chunk during update before calling 'Update.write'.
 * @note    Somehow, lowering the buffer size seems to fix the SSL over GSM problem:
 *          [ 48389][E][ssl_lib_client.cpp:35] _handle_error(): [data_to_read():386]: (-29056) SSL - Verification of the message MAC failed
 * 
 *          Could be the size of a ESP32 flash page (usually 4096 bytes), but the SSL seems to like a smaller value.
 */
#define OTA_UPDATE_BUFFER_SIZE  256

IOTA::IOTA(Client* client, int currFirmware, const char* serial) {
    m_client       = client;
    m_currFirmware = currFirmware;
    m_serial       = serial;
}

void IOTA::setProgressCallback(OTA_ONPROGRESS_SIGNATURE onProgress) {
    m_onProgress = onProgress;
}

void IOTA::onProgress(uint32_t downloadedBytes, uint32_t totalBytes) {
    if (m_onProgress != NULL)
        m_onProgress(downloadedBytes, totalBytes);
}

EOTAUpdateStatus IOTA::download(
    Client* client, const char* updateMD5, uint32_t totalSize,
    char calculatedSHA256[OTA_SHA256_MAX_SIZE_CHAR]
) {
    uint8_t* updateBuffer;
    size_t updateBufferSize = mallocTarget((void**)&updateBuffer, OTA_UPDATE_BUFFER_SIZE, 2);

    // Prepare
    if (!Update.begin(totalSize)) {
        LOG_E(PINICORE_TAG_OTA, "Firmware size (%d) too large for partition size, aborting update", totalSize);
        return EOTAUpdateStatus::OTA_MEMORY_ERROR;
    }
    Update.setMD5(updateMD5);

    mbedtls_md_context_t ctx;
    mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
    const mbedtls_md_info_t *md_info = mbedtls_md_info_from_type(md_type);
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, md_info, 0);	
    mbedtls_md_starts(&ctx);

    // Read body
    char c;
    size_t bufferPos = 0;
    uint32_t downloadedSize = 0;
    uint64_t timeout = getMillis();
    LOG_I(PINICORE_TAG_OTA, "Firmware download started...");
    while (downloadedSize < totalSize && client->connected() && getMillis()-timeout < (30*1000)) {
        while (client->available()) {
            size_t availableLen = client->available();
            size_t toRead = min(OTA_UPDATE_BUFFER_SIZE-bufferPos, availableLen);
            int bytesRead = client->readBytes(updateBuffer + bufferPos, toRead);

            if (bytesRead < 0) {
                LOG_E(PINICORE_TAG_OTA, "Download failed during read");
                abortAndFree();
                return EOTAUpdateStatus::OTA_CLIENT_ERROR;
            }

            timeout = getMillis();
            if (bytesRead == 0) {
                continue;   // Data not received, wait a bit more
            }
        
            bufferPos += bytesRead;
            downloadedSize += bytesRead;
        
            if (bufferPos >= OTA_UPDATE_BUFFER_SIZE || downloadedSize == totalSize) {
                mbedtls_md_update(&ctx, (const unsigned char*)updateBuffer, bufferPos);
        
                int written = Update.write(updateBuffer, bufferPos);
                if (written != (int)bufferPos) {
                    LOG_E(PINICORE_TAG_OTA, "Unable to write to flash during download, total bytes written = %d", written);
                    abortAndFree();
                    return EOTAUpdateStatus::OTA_DL_INCOMPLETE;
                }
        
                bufferPos = 0;
                onProgress(downloadedSize, totalSize);
            }
        }
    }
    free(updateBuffer);
    LOG_I(PINICORE_TAG_OTA, "Firmware download complete");
    
    byte calcSHA256[OTA_SHA256_MAX_SIZE];
    mbedtls_md_finish(&ctx, calcSHA256);
    mbedtls_md_free(&ctx);
    // Convert raw SHA-256 (32 bytes) to hex string (64 chars + null)
    for (int i=0; i<OTA_SHA256_MAX_SIZE; ++i) {
        sprintf(&calculatedSHA256[i*2], "%02x", calcSHA256[i]);
    }
    calculatedSHA256[OTA_SHA256_MAX_SIZE_CHAR]=0;

    LOG_D(PINICORE_TAG_OTA, "Expected size in bytes: %d", totalSize);
    LOG_D(PINICORE_TAG_OTA, "Actual   size in bytes: %d", downloadedSize);
    if (downloadedSize != totalSize) {
        LOG_E(PINICORE_TAG_OTA, "Incomplete download");
        abortAndFree();
        return EOTAUpdateStatus::OTA_DL_INCOMPLETE;
    }

    return EOTAUpdateStatus::OTA_DL_COMPLETE;
}

EOTAUpdateStatus IOTA::install(
    const char* updateMD5, const char* updateSHA256,
    char calculatedSHA256[OTA_SHA256_MAX_SIZE_CHAR]
) {
    LOG_D(PINICORE_TAG_OTA, "Expected SHA-256: %s", updateSHA256);
    LOG_D(PINICORE_TAG_OTA, "Actual   SHA-256: %s", calculatedSHA256);

    if (strcmp(calculatedSHA256, updateSHA256)) {
        LOG_E(PINICORE_TAG_OTA, "SHA-256 do not match");
        abortAndFree();
        return EOTAUpdateStatus::OTA_CHECKSUM_MISMATCH;
    }

    // Apply update
    if(Update.end()) {
        LOG_I(PINICORE_TAG_OTA, "Update installed with success, reboot required");
        return EOTAUpdateStatus::OTA_INSTALLED;
    }

    LOG_E(PINICORE_TAG_OTA, "Update failed");
    return EOTAUpdateStatus::OTA_FAILED;
}

void IOTA::abortAndFree() {
    LOG_W(PINICORE_TAG_OTA, "Update aborted");
    Update.abort();
    Update.end();
}
