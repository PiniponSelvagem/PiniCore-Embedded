#include "convert.hpp"

namespace pinicore {

bool stringHex2Array(
    const char* hexStr, uint32_t hexLen,
    uint32_t section,   uint32_t sectionSize,
    uint32_t* array,    size_t arraySize
) {
    uint32_t base = section * sectionSize;
    uint32_t relayBit = 0;

    for (int32_t i = (int32_t)hexLen - 1; i >= 0; --i) {
        char c = hexStr[i];

        uint8_t nibble;

        if (c >= '0' && c <= '9')      nibble = c - '0';
        else if (c >= 'a' && c <= 'f') nibble = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') nibble = c - 'A' + 10;
        else return false;

        while (nibble) {
            uint8_t bitPos = __builtin_ctz(nibble);
            nibble &= (nibble - 1);

            uint32_t globalIndex = base + relayBit + bitPos;

            if (relayBit + bitPos >= sectionSize)
                return false;

            uint32_t wordIndex = globalIndex >> 5;
            uint32_t bitIndex  = globalIndex & 31;

            array[wordIndex] |= (1u << bitIndex);
        }

        relayBit += 4;
    }

    return true;
}

} // pinicore
