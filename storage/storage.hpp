/**
* @file     storage.hpp
* @brief    Persistence data storage interface.
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

#ifndef PINICORE_STORAGE_STORAGE_H
#define PINICORE_STORAGE_STORAGE_H

#include <LittleFS.h>

namespace pinicore {

#define STORAGE_DIR_SYSTEM               "/system"                     // Folder related to PiniCore system files.
#define STORAGE_FILE_IDENTIFICATION      STORAGE_DIR_SYSTEM "/id.sys"  // Path to file that is used to identiofy the storage system.
#define STORAGE_IDENTIFICATION_MAX_BYTES 32

class Storage {
    public:
        /**
         * @brief   Initializes and mounts the file system.
         * @param   storageId Storage version identification, example: "PINICORE_TEST", "PINI_HOME_SENSOR", etc.
         * @param   size 'storageId' char* size, max = 'STORAGE_IDENTIFICATION_MAX_BYTES' including null termination.
         * @param   formatOnMismatch True if should format file system on 'storageId' mismatch.
         * @return  True if file system mounted successfully, false otherwise.
         */
        bool init(const char* storageId, const size_t size, const bool formatOnMismatch = false);

        /**
         * @brief   Format the storage file system and then set a 'storageId' to identify it.
         * @param   storageId Storage version identification, example: "PINICORE_TEST", "PINI_HOME_SENSOR", etc.
         * @param   size 'storageId' char* size, max = 'STORAGE_IDENTIFICATION_MAX_BYTES' including null termination.
         * @return  True if formated with success, false otherwise.
         * @note    The 'storageId' will be placed in a file that is used internally to identify the storage during \ref 'init'.
         */
        bool format(const char* storageId, const size_t size);

        /**
         * @brief   Flush if anything in cache and then unmount it.
         * @note    To use the storage again, call \ref 'init' again.
         */
        void close();

        /**
         * @brief   Open a file.
         * @param   path Path to the file.
         * @param   mode How to open the file: 'FILE_READ', 'FILE_WRITE', 'FILE_APPEND'.
         * @param   create True to create the file on that path.
         */
        File open(const char* path, const char* mode = FILE_READ, const bool create = false);

        /**
         * @brief   Check if a file exits on the specified path.
         * @param   path Path to the file.
         * @return  True if file exists, false otherwise.
         */
        bool exists(const char* path);

        /**
         * @brief   Remove a file on the specified path.
         * @param   path Path to the file.
         * @return  True is file was removed, false otherwise.
         */
        bool remove(const char* path);

        /**
         * @brief   Rename/Move a file.
         * @param   pathFrom Current file path.
         * @param   pathTo New file path.
         * @return  True is file was renamed/moved, false otherwise.
         */
        bool rename(const char* pathFrom, const char* pathTo);

        /**
         * @brief   Create a directory.
         * @param   path Path to the directory.
         * @return  True if created with success, false otherwise.
         */
        bool mkdir(const char *path);

        /**
         * @brief   Remove a directory if exists and it is empty.
         * @param   path Path to the directory.
         * @return  True if removed directory with success, false if does not exist or not empty.
         */
        bool rmdir(const char *path);

        /**
         * @brief   Get the total size in bytes of the file system.
         * @return  Total bytes of the file system.
         */
        size_t bytesTotal();

        /**
         * @brief   Get the amount of free bytes in the file system.
         * @return  Free bytes of the file system.
         */
        size_t bytesFree();

        /**
         * @brief   Get the amount of used bytes in the file system.
         * @return  Used bytes of the file system.
         */
        size_t bytesUsage();

    
    private:
        /**
         * @brief   Mount the LittleFS
         * @return  Status of the success of the mounting
         * @note    If unable to mount, will format to try to recover it
         */
        bool mount();

        /**
         * @brief   Create storage identification file, replace if exists.
         * @param   storageId Storage version identification, example: "PINICORE_TEST", "PINI_HOME_SENSOR", etc.
         * @param   size 'storageId' char* size, max = 'STORAGE_IDENTIFICATION_MAX_BYTES' including null termination.
         * @return  True if system files created with success, false otherwise.
         */
        bool createSystemFiles(const char* storageId, const size_t size);

        /**
         * @brief   Check if storage ID in file system matches the expected 'storageId'.
         * @param   storageId Storage version identification, example: "PINICORE_TEST", "PINI_HOME_SENSOR", etc.
         * @param   size 'storageId' char* size, max = 'STORAGE_IDENTIFICATION_MAX_BYTES' including null termination.
         */
        bool checkSystemFiles(const char* storageId, const size_t size);

        /**
         * @brief   Dumps the LittleFS file structure to Serial.
         * @param   path Directory to start from.
         * @param   levels How many levels deep to go.
         * @param   indent Used in recurssive calls to indent that level.
         * @note    Only use this for debugging LittleFS file structure.
         */
        void dumpTree(const char* path, uint8_t levels, String indent = "");
};

} // pinicore

#endif // PINICORE_STORAGE_STORAGE_H