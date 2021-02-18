/*
 * truetouch_pcba_001.h - board definitions for first revision TrueTouch PCBA.
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
#define LEDS_NUMBER             1

#define LED_1                   _nRF_GPIO_PIN_MAP(0, 29)    /* BL652: 34 */
#define LED_START               LED_1
#define LED_STOP                LED_1

#define LEDS_ACTIVE_STATE       1

#define LEDS_LIST               { LED_1 }

#define LEDS_INV_MASK           LEDS_MASK

#define BSP_LED_0               LED_1

#define BUTTONS_NUMBER          0

#define BUTTON_PULL             _nRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE    0

#define BUTTONS_LIST            { }

#define RX_PIN_NUMBER           _nRF_GPIO_PIN_MAP(0, 13)    /* BL652: 28 */
#define TX_PIN_NUMBER           _nRF_GPIO_PIN_MAP(0, 15)    /* BL652: 29 */
#define CTS_PIN_NUMBER          UNUSED_PIN
#define RTS_PIN_NUMBER          UNUSED_PIN
#define HWFC                    false

////////////////////////////////////////////////////////////////////////////////////////////////////
// TrueTouch Definitions
////////////////////////////////////////////////////////////////////////////////////////////////////

/* BL652 ERM PWM pin mapping */
#define M1_PIN                  _nRF_GPIO_PIN_MAP(0, 16)    /* M1, BL652: 10 */
#define M2_PIN                  _nRF_GPIO_PIN_MAP(0, 19)    /* M2, BL652: 31 */
#define M3_PIN                  _nRF_GPIO_PIN_MAP(0, 22)    /* M3, BL652: 4  */
#define M4_PIN                  _nRF_GPIO_PIN_MAP(0, 25)    /* M4, BL652: 38 */
#define M5_PIN                  _nRF_GPIO_PIN_MAP(0, 27)    /* M5, BL652: 36 */
#define M6_PIN                  _nRF_GPIO_PIN_MAP(0, 30)    /* M6, BL652: 33 */

/* Board-to-finger ERM PWM pin mapping */
#define ERM_THUMB               M1_PIN
#define ERM_INDEX               M3_PIN
#define ERM_MIDDLE              M2_PIN
#define ERM_RING                M6_PIN
#define ERM_PINKY               M5_PIN
#define ERM_PALM                M4_PIN

/* BL652 solenoid pin mapping */
#define S1_PIN                  _nRF_GPIO_PIN_MAP(0, 0)     /* S1, BL652: 25 */
#define S2_PIN                  _nRF_GPIO_PIN_MAP(0, 3)     /* S2, BL652: 22 */
#define S3_PIN                  _nRF_GPIO_PIN_MAP(0, 6)     /* S3, BL652: 19 */
#define S4_PIN                  _nRF_GPIO_PIN_MAP(0, 8)     /* S4, BL652: 17 */
#define S5_PIN                  _nRF_GPIO_PIN_MAP(0, 12)    /* S5, BL652: 12 */

/* Board-to-finger solenoid pin mapping */
#define SOLENOID_THUMB          S5_PIN
#define SOLENOID_INDEX          S4_PIN
#define SOLENOID_MIDDLE         S3_PIN
#define SOLENOID_RING           S2_PIN
#define SOLENOID_PINKY          S1_PIN

/* Set true so PWM is inverted (i.e. runs high until
   it hits the top value then goes low) - so, a duty cycle of 255 is always on,
   0 is always off. */
#define TRUETOUCH_INVERT_PWM    true

/* Set true so pulsing happens in parallel (all pins on then all pins off) instead of
   sequentially. */
#define TRUETOUCH_PULSE_PARALLEL    false

/* Set true to use the UART logger backend, otherwise RTT will be used. */
#define TRUETOUCH_USE_SERIAL        false
