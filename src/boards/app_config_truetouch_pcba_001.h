/*
 * app_config_truetouch_pcba_001.h - Custom PCBA-specific application configuration.
 *
 * Copyright (c) 2021 TrueTouch
 * Distributed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT)
 */

#pragma once

#include "boards_inc.h"

#include <nrf_sdm.h>

////////////////////////////////////////////////////////////////////////////////////////////////////
// SDK Config Definitions
////////////////////////////////////////////////////////////////////////////////////////////////////

/**< Logger backend config: use RTT */

#define NRF_LOG_BACKEND_RTT_ENABLED 1
#define NRF_LOG_BACKEND_UART_ENABLED 0

/**
 * This board uses the BL652 module, which doesn't have an external xtal.
 * Configure the clocks based on Laird's appnote (pg. 2-3).
 * https://assets.lairdtech.com/home/brandworld/files/Using%20the%20DVK-BL652%20and%20Nordic%20SDK%20v14.0.0%20with%20Eclipse%20and%20GCC.pdf
 */

/**< Internal RC as clock source. */
#define NRF_SDH_CLOCK_LF_SRC NRF_CLOCK_LF_SRC_RC
/**< SoftDevice calibration timer interval. */
#define NRF_SDH_CLOCK_LF_RC_CTIV 16
/**< How often to calibrate the RC oscillator. */
#define NRF_SDH_CLOCK_LF_RC_TEMP_CTIV 2
/**< Clock accuracy. */
#define NRF_SDH_CLOCK_LF_ACCURACY NRF_CLOCK_LF_ACCURACY_500_PPM
