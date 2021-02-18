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

#if defined(TRUETOUCH_USE_SERIAL) && TRUETOUCH_USE_SERIAL
#   define NRF_LOG_BACKEND_RTT_ENABLED 0
#   define NRF_LOG_BACKEND_UART_ENABLED 1
#else // use RTT logger backend
#   define NRF_LOG_BACKEND_RTT_ENABLED 1
#   define NRF_LOG_BACKEND_UART_ENABLED 0
#endif

#endif // APP_CONFIG_H
