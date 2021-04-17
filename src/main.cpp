/*
 * main.cpp
 *
 * Main program for the TrueTouch device.
 *
 * Copyright (c) 2021 TrueTouch
 * Distributed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT)
 */

////////////////////////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "boards_inc.h"
#include "nordic_ble.hpp"
#include "truetouch.hpp"
#include "util.hpp"

#include <app_timer.h>
#include <app_util_platform.h>
#include <nordic_common.h>
#include <nrf.h>
#include <nrf_gpio.h>
#include <nrf_pwr_mgmt.h>

#include <nrf_log.h>
#include <nrf_log_ctrl.h>
#include <nrf_log_default_backends.h>

#include <cstddef>

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////////////////////////////////////////////
/** Value used as error code on stack dump, can be used to identify stack location on stack
    unwind. */
static constexpr std::uint32_t DEAD_BEEF { 0xDEADBEEF };

/** Period (ms) for blinking status LED. */
static constexpr std::uint32_t STATUS_LED_PERIOD_MS { 500 };

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private data
////////////////////////////////////////////////////////////////////////////////////////////////////
/**< A timer to drive the status LED. */
APP_TIMER_DEF(g_status_led_timer);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private Function Prototypes
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

/**@brief Function for initializing the timer module.
 */
static void timers_init();

/**@brief Function for initializing the nrf log module.
 */
static void log_init();

/**@brief Function for initializing power management.
 */
static void power_management_init();

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle();

/**
 * BLE event callback for updating the status LED.
 */
static void ble_event_callback(ble_evt_t const *ble_evt);

/**
 * Callback for toggling the status LED.
 */
static void led_timer_handler(void *p_context);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////////////////////////

/**@brief Application main function.
 */
int main()
{
    // Initialize.
    log_init();
    timers_init();
    power_management_init();

#ifdef BENCHMARK_TIMING
      nrf_gpio_cfg_output(FEATHER_SDA_PIN);
#endif

    /* Config status LED as output. */
    nrf_gpio_cfg_output(STATUS_LED_PIN);
    nrf_gpio_pin_clear(STATUS_LED_PIN);

    ble::init(ble_event_callback);

    /* CTOR registers BLE callback and configures solenoid/ERM pins */
    TrueTouch truetouch {};

    // Start execution.
    NRF_LOG_INFO("Debug logging started.");
    ble::advertise();

    /* Start status LED toggle timer (toggle while advertising). */
    APP_ERROR_CHECK(
        app_timer_start(g_status_led_timer, APP_TIMER_TICKS(STATUS_LED_PERIOD_MS), nullptr));

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
        truetouch.service();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private Function Implementations
////////////////////////////////////////////////////////////////////////////////////////////////////

static void timers_init() {
    ret_code_t err_code = app_timer_init();

    err_code = app_timer_create(&g_status_led_timer,
                            APP_TIMER_MODE_REPEATED,
                            led_timer_handler);

    APP_ERROR_CHECK(err_code);
}

static void log_init() {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void power_management_init() {
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

static void idle_state_handle() {
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static void ble_event_callback(ble_evt_t const *ble_evt) {
    switch (ble_evt->header.evt_id)
    {
        /** Connected: solid status LED */
        case BLE_GAP_EVT_CONNECTED:
            APP_ERROR_CHECK(app_timer_stop(g_status_led_timer));
            nrf_gpio_pin_set(STATUS_LED_PIN);
            break;

        /** Disconnected (device will restart advertising): toggling status LED. */
        case BLE_GAP_EVT_DISCONNECTED:
            APP_ERROR_CHECK(app_timer_start(
                g_status_led_timer, APP_TIMER_TICKS(STATUS_LED_PERIOD_MS), nullptr));
            break;

        default:
            // No implementation needed.
            break;
    }
}

static void led_timer_handler(void *p_context) {
    nrf_gpio_pin_toggle(STATUS_LED_PIN);
}
