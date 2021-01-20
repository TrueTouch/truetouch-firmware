/*
 * nordic_pwm.hpp
 *
 * Wrapper for the PWM portions of the Nordic PWM library example.
 *
 * Copyright (c) 2021 TrueTouch
 * Distributed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT)
 */

#pragma once

#include "boards_inc.h"

#include <nrf_drv_pwm.h>

#include <cstdint>

namespace pwm {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////////////////////////////////////////////

/** Pins used in the first PWM module. */
constexpr std::uint8_t PWM0_CHANNEL0 { ERM_THUMB };
constexpr std::uint8_t PWM0_CHANNEL1 { ERM_INDEX };
constexpr std::uint8_t PWM0_CHANNEL2 { ERM_MIDDLE };
constexpr std::uint8_t PWM0_CHANNEL3 { ERM_RING };
constexpr std::uint8_t PWM0_PINS[NRF_PWM_CHANNEL_COUNT] {
    PWM0_CHANNEL0,
    PWM0_CHANNEL1,
    PWM0_CHANNEL2,
    PWM0_CHANNEL3,
};

/** Pins used in the second PWM channel. */
constexpr std::uint8_t PWM1_CHANNEL0 { ERM_PINKY };
constexpr std::uint8_t PWM1_CHANNEL1 { ERM_PALM };
constexpr std::uint8_t PWM1_CHANNEL2 { NRF_DRV_PWM_PIN_NOT_USED };
constexpr std::uint8_t PWM1_CHANNEL3 { NRF_DRV_PWM_PIN_NOT_USED };
constexpr std::uint8_t PWM1_PINS[NRF_PWM_CHANNEL_COUNT] {
    PWM1_CHANNEL0,
    PWM1_CHANNEL1,
    PWM1_CHANNEL2,
    PWM1_CHANNEL3,
};

/** Initialize the PWM library. Blocks until the PWM is ready. */
void init();

/** Set the duty cycle of a given pin. The pin must be a PWM pin. */
void set_duty_cycle(std::uint8_t pin, std::uint16_t duty_cycle);

/** Sets the duty cycle of the given ERM pin. */


}  // namespace pwm