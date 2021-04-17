/*
 * util.hpp
 *
 * Wrapper for the common util portions of the Nordic examples.
 *
 * Copyright (c) 2021 TrueTouch
 * Distributed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT)
 */

#pragma once

#include <cstdint>

namespace util {

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
void sleep_mode_enter();

// fix endianness (network to host)
inline std::uint32_t byte_swap32(std::uint32_t input) {
    return __builtin_bswap32(input);
}

// check if bit n is set in the mask
inline bool is_set(std::uint32_t mask, unsigned int n) {
    if (n > 31) {
        return false;
    }
    return mask & (1UL << n);
}

// return the bit position of the highest set bit (-1 if no bits are set)
inline int get_highest_bit(std::uint32_t mask) {
    for (int i = 31; i >= 0; --i) {
        if (mask & (1UL << i)) {
            return i;
        }
    }
    return -1;
}

// clear the given bit
inline void clear_bit(std::uint32_t &mask, int bit_pos) {
    if (bit_pos >= 0) {
        mask &= ~(1UL << bit_pos);
    }
}

// clears the bit position of the highest set bit
inline void clear_highest_bit(std::uint32_t &mask) {
    clear_bit(mask, get_highest_bit(mask));
}

}  // namespace util