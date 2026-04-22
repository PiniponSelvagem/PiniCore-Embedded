#include "crypto.h"
#include <CRC32.h>

uint32_t calculateChecksum(const uint8_t* data, size_t size, const uint8_t phrase) {
    CRC32 crc;
    if (phrase != 0)
        crc.add(phrase);
    crc.add(data, size);
    return crc.calc();
}
