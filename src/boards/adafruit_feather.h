/*
 * adafruit_feather.h - board definitions for Adafruit nRF52840 Feather.
 *
 * Copyright (c) 2021 TrueTouch
 * Distributed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT)
 */

#pragma once

#include "boards_gpio.h"

#define UNUSED_PIN              0

////////////////////////////////////////////////////////////////////////////////////////////////////
// Common BSP Definitions
////////////////////////////////////////////////////////////////////////////////////////////////////
#define LEDS_NUMBER             2

#define LED_1                   _nRF_GPIO_PIN_MAP(1, 15)
#define LED_2                   _nRF_GPIO_PIN_MAP(1, 10)
#define LED_START               LED_1
#define LED_STOP                LED_2

#define LEDS_ACTIVE_STATE       1

#define LEDS_LIST               { LED_1, LED_2 }

#define LEDS_INV_MASK           LEDS_MASK

#define BSP_LED_0               LED_1
#define BSP_LED_1               LED_2

#define BUTTONS_NUMBER          1

#define BUTTON_1                _nRF_GPIO_PIN_MAP(1, 2)
#define BUTTON_PULL             _nRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE    0

#define BUTTONS_LIST            { BUTTON_1 }

#define BSP_BUTTON_0            BUTTON_1

#define RX_PIN_NUMBER           _nRF_GPIO_PIN_MAP(0, 24)
#define TX_PIN_NUMBER           _nRF_GPIO_PIN_MAP(0, 25)
#define CTS_PIN_NUMBER          UNUSED_PIN
#define RTS_PIN_NUMBER          UNUSED_PIN
#define HWFC                    false

#define BSP_QSPI_SCK_PIN        _nRF_GPIO_PIN_MAP(0, 19)
#define BSP_QSPI_CSN_PIN        _nRF_GPIO_PIN_MAP(0, 20)
#define BSP_QSPI_IO0_PIN        _nRF_GPIO_PIN_MAP(0, 17)
#define BSP_QSPI_IO1_PIN        _nRF_GPIO_PIN_MAP(0, 22)
#define BSP_QSPI_IO2_PIN        _nRF_GPIO_PIN_MAP(0, 23)
#define BSP_QSPI_IO3_PIN        _nRF_GPIO_PIN_MAP(0, 21)

////////////////////////////////////////////////////////////////////////////////////////////////////
// Adafruit Feather Definitions
////////////////////////////////////////////////////////////////////////////////////////////////////

/* I2C Pin Definitions */
#define FEATHER_SCL_PIN         _nRF_GPIO_PIN_MAP(0, 11)
#define FEATHER_SDA_PIN         _nRF_GPIO_PIN_MAP(0, 12)

/* SPI Pin Definitions */
#define FEATHER_SCK_PIN         _nRF_GPIO_PIN_MAP(0, 14)
#define FEATHER_MOSI_PIN        _nRF_GPIO_PIN_MAP(0, 13)
#define FEATHER_MISO_PIN        _nRF_GPIO_PIN_MAP(0, 15)

/* UART Pin Definitions */
#define FEATHER_RX_PIN          RX_PIN_NUMBER
#define FEATHER_TX_PIN          TX_PIN_NUMBER

/* Digital Pins */
#define FEATHER_D2_PIN          _nRF_GPIO_PIN_MAP(0, 10)
#define FEATHER_D5_PIN          _nRF_GPIO_PIN_MAP(1, 8)
#define FEATHER_D6_PIN          _nRF_GPIO_PIN_MAP(0, 7)
#define FEATHER_D9_PIN          _nRF_GPIO_PIN_MAP(0, 26)
#define FEATHER_D10_PIN         _nRF_GPIO_PIN_MAP(0, 27)
#define FEATHER_D11_PIN         _nRF_GPIO_PIN_MAP(0, 6)
#define FEATHER_D12_PIN         _nRF_GPIO_PIN_MAP(0, 8)
#define FEATHER_D13_PIN         _nRF_GPIO_PIN_MAP(0, 9)

/* Analog Pins */
#define FEATHER_A0_PIN          _nRF_GPIO_PIN_MAP(0, 4)
#define FEATHER_A1_PIN          _nRF_GPIO_PIN_MAP(0, 5)
#define FEATHER_A2_PIN          _nRF_GPIO_PIN_MAP(0, 30)
#define FEATHER_A3_PIN          _nRF_GPIO_PIN_MAP(0, 28)
#define FEATHER_A4_PIN          _nRF_GPIO_PIN_MAP(0, 2)
#define FEATHER_A5_PIN          _nRF_GPIO_PIN_MAP(0, 3)

/* Misc Pins */
#define FEATHER_AREF_PIN        _nRF_GPIO_PIN_MAP(0, 31)
#define NEOPIXEL_DI_PIN         _nRF_GPIO_PIN_MAP(0, 16)
#define FEATHER_VDIV_PIN        _nRF_GPIO_PIN_MAP(0, 29) // Vbat divider

/* LEDs */
#define FEATHER_LED_D3_PIN      LED_1
#define FEATHER_LED_CONN_PIN    LED_2

/* Buttons */
#define FEATHER_USER_BTN_PIN    BUTTON_1

////////////////////////////////////////////////////////////////////////////////////////////////////
// TrueTouch Definitions
////////////////////////////////////////////////////////////////////////////////////////////////////

/* ERM PWM pins (1 per finger + palm) */
#define ERM_THUMB               FEATHER_D5_PIN
#define ERM_INDEX               FEATHER_D6_PIN
#define ERM_MIDDLE              FEATHER_D9_PIN
#define ERM_RING                FEATHER_D10_PIN
#define ERM_PINKY               FEATHER_D11_PIN
#define ERM_PALM                FEATHER_D12_PIN

/* Solenoid GPIO pins (1 per finger) */
#define SOLENOID_THUMB          FEATHER_A0_PIN
#define SOLENOID_INDEX          FEATHER_A1_PIN
#define SOLENOID_MIDDLE         FEATHER_A2_PIN
#define SOLENOID_RING           FEATHER_A3_PIN
#define SOLENOID_PINKY          FEATHER_A4_PIN
