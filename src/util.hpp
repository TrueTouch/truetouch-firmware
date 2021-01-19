/*
 * util.hpp
 *
 * Wrapper for the common util portions of the Nordic examples.
 *
 * Copyright (c) 2021 TrueTouch
 * Distributed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT)
 */

#pragma once

namespace util {

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
void sleep_mode_enter();

}  // namespace util