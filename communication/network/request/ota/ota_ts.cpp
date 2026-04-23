#include "ota_ts.hpp"
#include "utils/log.hpp"

#include <ArduinoHttpClient.h>
#include <SSLClientESP32.h>
#include <ArduinoJson.h>

#include "utils/memory.hpp"

namespace pinicore {

#define PINICORE_TAG_OTA_TS "pcore_ota_ts"

#ifndef OTA_TS_HOST
    #define OTA_TS_HOST "ota.piniturbosun.cc"
#endif
#define OTA_TS_PORT 443

#define URI "/ota/check/?machineKey=%s&version=%d"

#define OTA_TS_CHECKUPDATE_BUFFER_MAX_SIZE  128
#define OTA_TS_CHECKUPDATE_URI_MAX_SIZE     128

void OTATS::setCredentials(const char* username, const char* password) {
    m_username = username;
    m_password = password;
}

void OTATS::setCertificate(const char* certificate) {
    m_certificate = certificate;
}

bool OTATS::checkUpdate() {
    SSLClientESP32 ssl_client(getClient());
    ssl_client.setCACert(m_certificate);

    char uri[OTA_TS_CHECKUPDATE_URI_MAX_SIZE];
    if (snprintf(uri, sizeof(uri), URI, getUniqueID(), getCurrentFirmware()) >= sizeof(uri)) {
        LOG_E(PINICORE_TAG_OTA_TS, "URI buffer too small, aborting check update");
        return false;
    }

    // Request
    HttpClient http(ssl_client, OTA_TS_HOST);
    http.beginRequest();
    http.get(uri);
    http.sendBasicAuth(m_username, m_password);
    http.endRequest();

    // Response
    int status = http.responseStatusCode();
    http.skipResponseHeaders();

    char body[OTA_TS_CHECKUPDATE_BUFFER_MAX_SIZE];
    size_t bodySize = http.read((uint8_t*)body, OTA_TS_CHECKUPDATE_BUFFER_MAX_SIZE);
    body[bodySize] = '\0'; // ensure null termination

    LOG_D(PINICORE_TAG_OTA_TS, "Received: [status: %d] [body (%d): %s]", status, bodySize, body);
    if (status == 404) {
        return false;
    }
    
    // JSON parsing
    JsonDocument json;
    DeserializationError jsonError = deserializeJson(json, body);
    if (jsonError) {
        const char* errorMsg = jsonError.c_str();
        LOG_D(PINICORE_TAG_OTA_TS, "JSON deserialization error: '%s'", errorMsg);
        return false;
    }
    
    p_versionAvailable = -1; // Invalidate previous update version check

    // Get update information
    if (json["success"].as<bool>()) {
        // available update version
        p_versionAvailable = json["firmware"]["version"];

        // available update URI path
        strncpy(m_uriAvailableUpdate, json["firmware"]["url"], OTA_TS_AVAILABLEUPDATE_URI_MAX_SIZE);

        LOG_I(PINICORE_TAG_OTA_TS, "Found update: [version: %d] [uri: '%s']", p_versionAvailable, m_uriAvailableUpdate);
        return true;
    }
    return false;
}

EOTAUpdateStatus OTATS::update() {
    if (p_versionAvailable == -1) {
        return EOTAUpdateStatus::OTA_UPDATE_NOT_AVAILABLE;
    }

    SSLClientESP32 ssl_client(getClient());
    ssl_client.setCACert(m_certificate);

    // Request
    HttpClient http(ssl_client, OTA_TS_HOST);
    http.connectionKeepAlive();
    http.beginRequest();
    http.get(m_uriAvailableUpdate);
    http.sendBasicAuth(m_username, m_password);
    http.endRequest();

    // Response
    int status = http.responseStatusCode();
    if (status != 200) {
        return EOTAUpdateStatus::OTA_CLIENT_ERROR;
    }

    // Get firmware file information from headers
    char updateMD5[128];
    char updateSHA256[128];
    int  updateSize;
    while (http.headerAvailable()) {
        /**
         * I would like to avoid 'String', but since the library 'ArduinoHttpClient'
         * is using String, lets just YOLO until I find a better replacement.
         */
        String hName  = http.readHeaderName();
        String hValue = http.readHeaderValue();
        //LOG_T(PINICORE_TAG_OTA, "%s: %s", hName.c_str(), hValue.c_str());
        
        hName.toLowerCase();
        if (hName == "x-esp32-sketch-md5")
            strncpy(updateMD5, hValue.c_str(), sizeof(updateMD5));
        else if (hName == "x-esp32-sketch-sha256")
            strncpy(updateSHA256, hValue.c_str(), sizeof(updateMD5));
        else if (hName == "x-esp32-sketch-size")
            updateSize = hValue.toInt();

        if (http.endOfHeadersReached())
            break;
    }

    LOG_D(PINICORE_TAG_OTA_TS, "Update found: [size: %d] [MD5: %s] [SHA256: %s]", updateSize, updateMD5, updateSHA256);

    // Download
    char calculatedSHA256[32];
    EOTAUpdateStatus statusDL = download(&http, updateMD5, updateSize, calculatedSHA256);
    if (statusDL != EOTAUpdateStatus::OTA_DL_COMPLETE) {
        return statusDL;
    }

    // Install
    EOTAUpdateStatus statusINSTALL = install(updateMD5, updateSHA256, calculatedSHA256);
    return statusINSTALL;
}

} // pinicore
