/*
 * boards_gpio.h - hack to handle issues with nRF SDK includes.
 *
 * Copyright (c) 2020 Cameron Kluza
 * Distributed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT)
 */

#ifndef BOARDS_GPIO_H
#define BOARDS_GPIO_H

#include <nrf.h>

/* HACK CMK: 
 * Board definitions use some nrf_gpio.h definitions.
 * Board definitions are included in app_config.h to define some
 *      config options in terms of the board (e.g. UART TX pin for logger).
 * app_config.h is included at the top of sdk_config.h (so it can pre-define config options),
 *      and through some confusing include shenanigans, the build breaks if nrf_gpio.h is included
 *      above sdk_config.h (through app_config.h).
 * So, definitions needed from nrf_gpio.h are duplicated here without the extra headers 
 *      nrf_gpio.h includes. Naming is adjusted to prevent conflicts.
 */

/** @brief Macro for mapping port and pin numbers to values understandable for nrf_gpio functions. */
#define _nRF_GPIO_PIN_MAP(port, pin) (((port) << 5) | ((pin) & 0x1F))

/**
 * @brief Enumerator used for selecting the pin to be pulled down or up at the time of pin
 * configuration.
 */
enum
{
    _nRF_GPIO_PIN_NOPULL   = GPIO_PIN_CNF_PULL_Disabled, ///<  Pin pull-up resistor disabled.
    _nRF_GPIO_PIN_PULLDOWN = GPIO_PIN_CNF_PULL_Pulldown, ///<  Pin pull-down resistor enabled.
    _nRF_GPIO_PIN_PULLUP   = GPIO_PIN_CNF_PULL_Pullup,   ///<  Pin pull-up resistor enabled.
};

#endif // BOARDS_GPIO_H
