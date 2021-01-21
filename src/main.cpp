/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

////////////////////////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "nordic_ble.hpp"
#include "nordic_pwm.hpp"
#include "util.hpp"

#include <app_timer.h>
#include <app_util_platform.h>
#include <bsp_btn_ble.h>
#include <nordic_common.h>
#include <nrf.h>
#include <nrf_gpio.h>
#include <nrf_pwr_mgmt.h>

#include <nrf_log.h>
#include <nrf_log_ctrl.h>
#include <nrf_log_default_backends.h>

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////////////////////////////////////////////
/** Value used as error code on stack dump, can be used to identify stack location on stack
    unwind. */
constexpr std::uint32_t DEAD_BEEF { 0xDEADBEEF };

/**< UART TX buffer size. */
constexpr std::uint32_t UART_TX_BUF_SIZE { 256 };

/**< UART RX buffer size. */
constexpr std::uint32_t UART_RX_BUF_SIZE { 256 };

constexpr std::uint32_t SOLENOID_PIN_CNT { 5 };
constexpr std::uint8_t SOLENOID_PINS[SOLENOID_PIN_CNT] {
    SOLENOID_THUMB,
    SOLENOID_INDEX,
    SOLENOID_MIDDLE,
    SOLENOID_RING,
    SOLENOID_PINKY,
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private data
////////////////////////////////////////////////////////////////////////////////////////////////////

/** App timer instance to update PWMs. */
APP_TIMER_DEF(m_pwm_update_timer);

/** Duty cycle for LEDs. */
static constexpr int ADJUST { 250 };
static int m_conn_duty_cycle { 10000 };
static int m_conn_adjust { -ADJUST };
static int m_d3_duty_cycle { 0 };
static int m_d3_adjust { ADJUST };

////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void pwm_timer_callback(void *p_context)
{
    /* Adjust duty cycles */
    m_conn_duty_cycle += m_conn_adjust;
    m_d3_duty_cycle += m_d3_adjust;

    if (m_conn_duty_cycle <= 0) {
        m_conn_duty_cycle = 0;
        m_conn_adjust = +ADJUST;
    } else if (m_conn_duty_cycle >= 10000) {
        m_conn_duty_cycle = 10000;
        m_conn_adjust = -ADJUST;
    }

    if (m_d3_duty_cycle <= 0) {
        m_d3_duty_cycle = 0;
        m_d3_adjust = +ADJUST;
    } else if (m_d3_duty_cycle >= 10000) {
        m_d3_duty_cycle = 10000;
        m_d3_adjust = -ADJUST;
    }

    NRF_LOG_INFO("Update PWM: conn=%d, d3=%d", m_conn_duty_cycle, m_d3_duty_cycle);

    pwm::set_duty_cycle(FEATHER_LED_CONN_PIN, m_conn_duty_cycle);
    pwm::set_duty_cycle(FEATHER_LED_D3_PIN, m_d3_duty_cycle);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init()
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_pwm_update_timer, APP_TIMER_MODE_REPEATED, pwm_timer_callback);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            util::sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            ble::disconnect();
            break;

        case BSP_EVENT_WHITELIST_OFF:
            ble::advertise_no_whitelist();
            break;

        default:
            break;
    }
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init()
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init()
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle()
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Initialize GPIO outputs to solenoids.
 */
static void init_solenoid_gpio()
{
    for (std::size_t i = 0; i < SOLENOID_PIN_CNT; ++i) {
        nrf_gpio_cfg_output(SOLENOID_PINS[i]);
    }
}

/* NOTE: following functions available for manipulating GPIO:
 * nrf_gpio_pin_set(uint32_t pin_number);
 * nrf_gpio_pin_clear(uint32_t pin_number);
 * nrf_gpio_pin_toggle(uint32_t pin_number);
 * nrf_gpio_pin_write(uint32_t pin_number, uint32_t value);
 */

/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;

    // Initialize.
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    init_solenoid_gpio();
    power_management_init();
    ble::init();
    pwm::init();

    // Start execution.
    printf("\r\nUART started.\r\n");
    NRF_LOG_INFO("Debug logging over UART started.");
    ble::advertise();

    // Start timer
    APP_ERROR_CHECK(app_timer_start(m_pwm_update_timer, 10000, nullptr));

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
