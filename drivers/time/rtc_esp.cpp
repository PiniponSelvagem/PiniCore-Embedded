#include "rtc_esp.hpp"
#include <sys/time.h>
#include "utils/log.hpp"

namespace pinicore {

#define PINICORE_TAG_RTC_ESP "pcore_rtc_esp"

void RTCesp::set(time_t timestamp) {
    struct timeval tv;
    tv.tv_sec = timestamp;
    tv.tv_usec = 0;
    settimeofday(&tv, nullptr);
    LOG_D(PINICORE_TAG_RTC_ESP, "RTC ESP set %s to '%li' and now are '%li'", "(A)", timestamp, get());
}

void RTCesp::set(int hour, int min, int sec, int day, int month, int year) {
    struct tm tm = {};
    tm.tm_hour = hour;
    tm.tm_min = min;
    tm.tm_sec = sec;
    tm.tm_mday = day;
    tm.tm_mon = month;          // 0 = January
    tm.tm_year = year - 1900;   // Years since 1900
    tm.tm_isdst = -1;           // Let libc determine DST

    time_t timestamp = mktime(&tm);
    set(timestamp);
    LOG_D(PINICORE_TAG_RTC_ESP, "RTC ESP set %s to '%li' and now are '%li'", "(B)", timestamp, get());
}

time_t RTCesp::get() {
    time_t now;
    time(&now);
    return now;
}

} // pinicore