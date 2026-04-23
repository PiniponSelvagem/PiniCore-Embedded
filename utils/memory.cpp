#include "memory.hpp"
#include "log.hpp"

#include <esp_heap_caps.h>

namespace pinicore {

#define PINICORE_TAG_MEM "pcore_memory"

size_t mavailableLargest() {
    size_t freeHeap = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    LOG_D(PINICORE_TAG_MEM, "Largest available continuous heap size for allocation: %u bytes", freeHeap);
    return freeHeap;
}

size_t mallocTarget(void** pMemory, size_t targetBytes, uint32_t dividerCap) {
    size_t freeHeap = mavailableLargest();

    size_t cappedBytes = freeHeap / dividerCap;

    if (cappedBytes > targetBytes) {
        cappedBytes = targetBytes;
    }
    
    *pMemory = heap_caps_malloc(cappedBytes, MALLOC_CAP_8BIT);

    if (pMemory) {
        LOG_D(PINICORE_TAG_MEM, "Allocated %u bytes", cappedBytes);
        return cappedBytes;
    }

    LOG_E(PINICORE_TAG_MEM, "Unable to allocate memory for %u bytes", targetBytes);
    return 0;
}

} // pinicore
