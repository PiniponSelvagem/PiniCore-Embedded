#include "log.hpp"
#include <cstdio>
#include <stdarg.h>
#include "time.hpp"

namespace pinicore {

// Internal function to log the message
void _plog_impl(const char* level, const char* klass, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);

    printf("%llu | %s [%s] ", getMillis(), level, klass);
    vprintf(fmt, args);
    printf("\n");

    va_end(args);
}

} // pinicore
