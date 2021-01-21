/*
 * nordic_pwm.cpp
 *
 * Wrapper for the PWM portions of the Nordic PWM library example.
 *
 * Copyright (c) 2021 TrueTouch
 * Distributed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT)
 */

#include "nordic_pwm.hpp"

#include <app_error.h>
#include <nrf_drv_pwm.h>
#include <nrf_log.h>

namespace pwm {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private data
////////////////////////////////////////////////////////////////////////////////////////////////////
/** Two PWM instances (need 6 outputs, each channel can output 4 different compares). */
static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static nrf_drv_pwm_t m_pwm1 = NRF_DRV_PWM_INSTANCE(1);

/** Duty cycles for the PWM0 instance. */
static nrf_pwm_values_individual_t m_pwm0_seq_values {};
static nrf_pwm_sequence_t const    m_pwm0_seq =
{
    .values = {
        .p_individual = &m_pwm0_seq_values,
    },
    .length              = NRF_PWM_VALUES_LENGTH(m_pwm0_seq_values),
    .repeats             = 0,
    .end_delay           = 0
};

/** Duty cycles for the PWM1 instance. */
static nrf_pwm_values_individual_t m_pwm1_seq_values {};
static nrf_pwm_sequence_t const    m_pwm1_seq =
{
    .values = {
        .p_individual = &m_pwm1_seq_values,
    },
    .length              = NRF_PWM_VALUES_LENGTH(m_pwm1_seq_values),
    .repeats             = 0,
    .end_delay           = 0
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private function prototypes
////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * For a given pin, gets a pointer to its duty cycle value. Writing to this pointer will change
 * the duty cycle output on the PWM pin.
 *
 * @param pin the pin number to get duty cycle for.
 * @return    the corresponding duty cycle pointer if the pin is valid, else nullptr.
 */
static std::uint16_t *get_duty_cycle(std::uint8_t pin);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private handler/callback prototypes
////////////////////////////////////////////////////////////////////////////////////////////////////

/** Handler for PWM0 events. */
static void pwm0_handler(nrf_drv_pwm_evt_type_t event_type);

/** Handler for PWM1 events. */
static void pwm1_handler(nrf_drv_pwm_evt_type_t event_type);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public implementations
////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * This initializes the two PWM instances, and starts continuous playback of a "sequence", which
 * just a collection of 4 duty cycle values used to control the 4 outputs per PWM instance. The
 * duty sequence playback reads from RAM the value to set as the duty cycle, so updating the
 * "m_pwm0_seq_values" and "m_pwm1_seq_values" variables will automatically update the duty cycles.
 */
void init()
{
    nrf_drv_pwm_config_t config0 {
        .output_pins  = {
            PWM0_CHANNEL0,
            PWM0_CHANNEL1,
            PWM0_CHANNEL2,
            PWM0_CHANNEL3,
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 10000, // TODO: what top value to use?
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };

    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, pwm0_handler));
    nrf_drv_pwm_simple_playback(&m_pwm0, &m_pwm0_seq, 1, NRF_DRV_PWM_FLAG_LOOP);

    nrf_drv_pwm_config_t config1 {
        .output_pins  = {
            PWM1_CHANNEL0,
            PWM1_CHANNEL1,
            PWM1_CHANNEL2,
            PWM1_CHANNEL3,
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 10000, // TODO: what top value to use?
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };

    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm1, &config1, pwm1_handler));
    nrf_drv_pwm_simple_playback(&m_pwm1, &m_pwm1_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}

void set_duty_cycle(std::uint8_t pin, std::uint16_t duty_cycle)
{
    std::uint16_t *p_duty_cycle = get_duty_cycle(pin);
    APP_ERROR_CHECK_BOOL(nullptr != p_duty_cycle);

    *p_duty_cycle = duty_cycle;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private function implementations
////////////////////////////////////////////////////////////////////////////////////////////////////
static std::uint16_t *get_duty_cycle(std::uint8_t pin)
{
    switch (pin) {
        case PWM0_CHANNEL0:
            return &m_pwm0_seq_values.channel_0;

        case PWM0_CHANNEL1:
            return &m_pwm0_seq_values.channel_1;

        case PWM0_CHANNEL2:
            return &m_pwm0_seq_values.channel_2;

        case PWM0_CHANNEL3:
            return &m_pwm0_seq_values.channel_3;

        case PWM1_CHANNEL0:
            return &m_pwm1_seq_values.channel_0;

        case PWM1_CHANNEL1:
            return &m_pwm1_seq_values.channel_1;

        case PWM1_CHANNEL2:
            return &m_pwm1_seq_values.channel_2;

        case PWM1_CHANNEL3:
            return &m_pwm1_seq_values.channel_3;

        default:
           return nullptr;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private handler/callback implementations
////////////////////////////////////////////////////////////////////////////////////////////////////

static void pwm0_handler(nrf_drv_pwm_evt_type_t event_type)
{
    // TODO: do we care about any of this?
}

static void pwm1_handler(nrf_drv_pwm_evt_type_t event_type)
{
    // TODO: do we care about any of this?
}

}  // namespace pwm