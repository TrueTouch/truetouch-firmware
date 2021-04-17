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

/**< Include a board-specific app config header if desired. */
#if defined(USE_BOARD_APP_CONFIG) && defined(CUSTOM_BOARD_INC)
#   include STRINGIFY(CONCAT_2(app_config_, CUSTOM_BOARD_INC.h))
#endif

#endif // APP_CONFIG_H
