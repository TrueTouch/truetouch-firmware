/*
 * app_config.h - Application configuration
 *
 * Copyright (c) 2021 TrueTouch
 * Distributed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT)
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "boards_inc.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// SDK Config Definitions
////////////////////////////////////////////////////////////////////////////////////////////////////

/**< Which pin the UART logger should transmit on - use board-specific definition */
#ifdef NRF_LOG_BACKEND_UART_TX_PIN
#   undef NRF_LOG_BACKEND_UART_TX_PIN
#endif
#define NRF_LOG_BACKEND_UART_TX_PIN TX_PIN_NUMBER

#endif // APP_CONFIG_H
