#include "print.hpp"

namespace pinicore {

void convertToBinaryString(char *buffer, uint32_t num, int bits) {
    for (int i = bits - 1; i >= 0; i--) {
        buffer[bits - 1 - i] = ((num >> i) & 1) ? '1' : '0';
    }
    buffer[bits] = '\0'; // Null-terminate the string
}

} // pinicore
