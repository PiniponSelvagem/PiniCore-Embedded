#include "array.hpp"

namespace pinicore {

bool calculateBitIndex(
    size_t arraySize,
    uint32_t sectionSize,
    uint32_t section, uint32_t bit,
    uint32_t* wordIndex, uint32_t* bitIndex
) {
    uint32_t globalIndex = section * sectionSize + bit;
    uint32_t maxBits = arraySize * 32;
    
    if (bit >= sectionSize || globalIndex >= maxBits)
        return false;

    *wordIndex = globalIndex >> 5;   // /32
    *bitIndex  = globalIndex & 31;   // %32

    if (*wordIndex >= arraySize)
        return false;

    return true;
}

bool calculateSectionBit(
    size_t arraySize,
    uint32_t sectionSize,
    uint32_t wordIndex, uint32_t bitIndex,
    uint32_t* section, uint32_t* bit
) {
    if (wordIndex >= arraySize || bitIndex >= 32)
        return false;

    uint32_t globalIndex = (wordIndex << 5) + bitIndex;

    *section = globalIndex / sectionSize;
    *bit     = globalIndex % sectionSize;

    return true;
}

} // pinicore
