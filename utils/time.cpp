#include "time.hpp"
#include <esp_timer.h>

namespace pinicore {

uint64_t getMillis() {
    return static_cast<uint64_t>(esp_timer_get_time()) / 1000LL;
}

void timestamp2DateTime(time_t timestamp, struct tm* dateTime) {
    if (dateTime == NULL)  return;
    localtime_r(&timestamp, dateTime);
}

} // pinicore
