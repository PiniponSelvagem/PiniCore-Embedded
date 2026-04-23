#include "time.hpp"
#include <esp_timer.h>

namespace pinicore {

uint64_t getMillis() {
    return static_cast<uint64_t>(esp_timer_get_time()) / 1000LL;
}

} // pinicore
