#include "array.hpp"

namespace pinicore {

bool calculateBitIndex(
    size_t arraySize, size_t arrayElementSize,
    uint32_t sectionSize,
    uint32_t section, uint32_t bit,
    uint32_t* wordIndex, uint32_t* bitIndex
) {
    uint32_t globalIndex = section * sectionSize + bit;

    if (
        globalIndex >= (arraySize * arrayElementSize) ||
        bit >= sectionSize
    ) {
        return false;
    }

    *wordIndex = globalIndex / arrayElementSize;
    *bitIndex  = globalIndex % arrayElementSize;

    if (*wordIndex >= arraySize)
        return false;

    return true;
}

} // pinicore
