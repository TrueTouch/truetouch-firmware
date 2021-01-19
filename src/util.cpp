/*
 * util.cpp
 *
 * Wrapper for the common util portions of the Nordic examples.
 *
 * Copyright (c) 2021 TrueTouch
 * Distributed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT)
 */

#include "util.hpp"

#include <bsp.h>
#include <bsp_btn_ble.h>

#include <cstdint>

namespace util {

void sleep_mode_enter()
{
    std::uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

}  // namespace util