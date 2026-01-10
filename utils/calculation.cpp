#include "calculation.hpp"
#ifdef ESP_PLATFORM
    #include <esp_random.h>
#else
    #include <stdlib.h>
#endif

uint32_t randomGenerator() {
#ifdef ESP_PLATFORM
    return esp_random();
#else
    return rand();
#endif
}
