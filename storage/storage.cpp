#include "storage.hpp"
#include "utils/log.hpp"
#include <stdint.h>
#include <LittleFS.h>

namespace pinicore {

#define PINICORE_TAG_STORAGE "pcore_storage"

#if PLOG_LEVEL >= PLOG_LEVEL_DEBUG
    #define PINICORE_DEBUG_FILE_STRUCTURE
#endif

bool Storage::init(const char* storageId, const size_t size, const bool formatOnMismatch) {
    if (size > STORAGE_IDENTIFICATION_MAX_BYTES) {
        LOG_W(PINICORE_TAG_STORAGE, 
            "Storage ID size above '%d', will truncate", STORAGE_IDENTIFICATION_MAX_BYTES 
        );
    }

    if (!mount()) {
        LOG_W(PINICORE_TAG_STORAGE, "Error while mounting... trying to recover by formating");
        bool isMounted = format(storageId, size);
        if (!isMounted) {
            return false;
        }
    }

    // Check system files, if ID mismatch then format it
    if (!checkSystemFiles(storageId, size)) {
        LOG_I(PINICORE_TAG_STORAGE, "Storage ID mismatch, formating...");
        bool isMounted = format(storageId, size);
        if (!isMounted) {
            return false;
        }
    }

#ifdef PINICORE_DEBUG_FILE_STRUCTURE
    // Dump filesystem structure
    LOG_D(PINICORE_TAG_STORAGE, "--------- Filesystem storage ---------");
    float percentFree = (bytesFree() * 100.0f) / bytesTotal();
    LOG_D(PINICORE_TAG_STORAGE, "Total = %u bytes", bytesTotal());
    LOG_D(PINICORE_TAG_STORAGE, "Usage = %u bytes", bytesUsage());
    LOG_D(PINICORE_TAG_STORAGE, "Free  = %u bytes", bytesFree());
    LOG_D(PINICORE_TAG_STORAGE, "Using %u bytes of %u bytes (%.02f %%)", bytesUsage(), bytesTotal(), percentFree);
    LOG_D(PINICORE_TAG_STORAGE, "--------------------------------------");
    LOG_D(PINICORE_TAG_STORAGE, "-------- Filesystem structure --------");
    dumpTree("/", 10);
    LOG_D(PINICORE_TAG_STORAGE, "--------------------------------------");
#endif
    return true;
}

bool Storage::format(const char* storageId, const size_t size) {
    if (!LittleFS.format()) {
        LOG_E(PINICORE_TAG_STORAGE, "Failed to format file system");
        return false;
    }

    if (!LittleFS.begin()) {
        LOG_E(PINICORE_TAG_STORAGE, "Unable to mount file system after format");
        return false;
    }
    
    return createSystemFiles(storageId, size);
}

void Storage::close() {
    LittleFS.end();
    LOG_I(PINICORE_TAG_STORAGE, "Filesystem unmounted");
}

File Storage::open(const char* path, const char* mode, const bool create) {
    return LittleFS.open(path, mode, create);
}

bool Storage::exists(const char* path) {
    return LittleFS.exists(path);
}

bool Storage::remove(const char* path) {
    return LittleFS.remove(path);
}

bool Storage::rename(const char* pathFrom, const char* pathTo) {
    return LittleFS.rename(pathFrom, pathTo);
}

bool Storage::mkdir(const char *path) {
    return LittleFS.mkdir(path);
}

bool Storage::rmdir(const char *path) {
    return LittleFS.rmdir(path);
}

size_t Storage::bytesTotal() { return LittleFS.totalBytes();     }
size_t Storage::bytesUsage() { return LittleFS.usedBytes();      }
size_t Storage::bytesFree()  { return bytesTotal()-bytesUsage(); }


bool Storage::mount() {
    if (!LittleFS.begin()) {
        LOG_E(PINICORE_TAG_STORAGE, "Unable to mount file system");
        return false;
    }
    return true;
}

bool Storage::createSystemFiles(const char* storageId, const size_t size) {
    size_t _size = (size<=STORAGE_IDENTIFICATION_MAX_BYTES) ? size : STORAGE_IDENTIFICATION_MAX_BYTES;
    char _storageId[STORAGE_IDENTIFICATION_MAX_BYTES];
    strncpy(_storageId, storageId, _size);

    mkdir(STORAGE_DIR_SYSTEM);
    File file = open(STORAGE_FILE_IDENTIFICATION, FILE_WRITE, true);
    if (!file) {
        LOG_E(PINICORE_TAG_STORAGE, "Failed to create version file");
        return false;
    }
    file.print(_storageId);
    file.close();

    LOG_I(PINICORE_TAG_STORAGE, "Added: '%s' -> '%s'", STORAGE_FILE_IDENTIFICATION, _storageId);
    return true;
}

bool Storage::checkSystemFiles(const char* storageId, const size_t size) {
    char filesystemId[STORAGE_IDENTIFICATION_MAX_BYTES];
    /**
     * This LittleFS.open can show this message:
     * [  5184][E][vfs_api.cpp:105] open(): /littlefs/system/id.sys does not exist, no permits for creation
     * It is safe to ignore because I am handling it with the 'if' statement.
     * It seems it is the same issue with 'LittleFS.exists' that prints out a similar msg when returning false.
     * The way the API should work, in my idea, is return TRUE or FALSE and shut the fuck up and let user handle the situation.
     */
    File file = LittleFS.open(STORAGE_FILE_IDENTIFICATION, FILE_READ);
    if (file) {
        file.readBytes(filesystemId, sizeof(filesystemId));
        file.close();
        return strncmp(filesystemId, storageId, size);
    }
    return false;
}

void Storage::dumpTree(const char* path, uint8_t levels, String indent) {
    File root = LittleFS.open(path);
    if (!root || !root.isDirectory()) {
        return; // skip non-folders
    }

    File file = root.openNextFile();
    while (file) {
        bool isDir = file.isDirectory();
        String path = file.path();  // full path

        // Extract just the filename (without parent path)
        String name = path.substring(path.lastIndexOf("/") + 1);

        if (isDir) {
            LOG_D(PINICORE_TAG_STORAGE, "%s📁 %s", indent, name.c_str());
            if (levels) {
                dumpTree(path.c_str(), levels - 1, indent + "    ");
            }
        } else {
            LOG_D(PINICORE_TAG_STORAGE, "%s📄 %s (%u bytes)", indent, name.c_str(), file.size());
        }

        file.close();
        file = root.openNextFile();
    }
    root.close();
}

} // pinicore
