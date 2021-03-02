/*
 * app_config_adafruit_feather.h - Feather-specific application configuration.
 *
 * Copyright (c) 2021 TrueTouch
 * Distributed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT)
 */

#pragma once

#include "boards_inc.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// SDK Config Definitions
////////////////////////////////////////////////////////////////////////////////////////////////////

/**< Logger backend config: use UART */

#define NRF_LOG_BACKEND_RTT_ENABLED 0
#define NRF_LOG_BACKEND_UART_ENABLED 1
