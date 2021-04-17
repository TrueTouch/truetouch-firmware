/*
 * nordic_ble.hpp
 *
 * Wrapper for the BLE portions of the Nordic UART Service example.
 *
 * Copyright (c) 2021 TrueTouch
 * Distributed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT)
 */

#pragma once

#include <app_timer.h>
#include <app_util.h>
#include <ble.h>
#include <ble_types.h>

#include <cstdint>

namespace ble {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////////////////////////////////////////////
/** A tag identifying the SoftDevice BLE configuration. */
constexpr int CONN_CFG_TAG { 1 };

/** Name of device. Will be included in the advertising data. */
constexpr const char *DEVICE_NAME { "TrueTouch" };

/** UUID type for the Nordic UART Service (vendor specific). */
constexpr std::uint32_t NUS_SERVICE_UUID_TYPE { BLE_UUID_TYPE_VENDOR_BEGIN };

/** The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
constexpr std::uint32_t ADV_INTERVAL { 64 };

/** The advertising duration (180 seconds) in units of 10 milliseconds. */
constexpr std::uint32_t ADV_DURATION { 0 };

/** Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
constexpr std::uint32_t MIN_CONN_INTERVAL { MSEC_TO_UNITS(20, UNIT_1_25_MS) };

/** Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
constexpr std::uint32_t MAX_CONN_INTERVAL { MSEC_TO_UNITS(75, UNIT_1_25_MS) };

/** Slave latency. */
constexpr std::uint32_t SLAVE_LATENCY { 0 };

/** Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
constexpr std::uint32_t CONN_SUP_TIMEOUT { MSEC_TO_UNITS(4000, UNIT_10_MS) };

/** Time from initiating event (connect or start of notification) to first time
    sd_ble_gap_conn_param_update is called (5 seconds). */
constexpr std::uint32_t FIRST_CONN_PARAMS_UPDATE_DELAY { APP_TIMER_TICKS(5000) };

/** Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
constexpr std::uint32_t NEXT_CONN_PARAMS_UPDATE_DELAY  { APP_TIMER_TICKS(30000) };

/** Number of attempts before giving up the connection parameter negotiation. */
constexpr std::uint32_t MAX_CONN_PARAMS_UPDATE_COUNT { 3 };

/** Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_OBSERVER_PRIO 3

/** Maximum number of callbacks that can be registered. */
constexpr std::uint32_t CALLBACK_MAX { 4 };

////////////////////////////////////////////////////////////////////////////////////////////////////
// Types
////////////////////////////////////////////////////////////////////////////////////////////////////

using EventCallback = void (*)(ble_evt_t const *ble_evt);

using UartCallback = void (*)(void *context, const std::uint8_t *data, std::uint16_t length);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the Nordic BLE stack. Any errors during BLE stack initialization are handled by
 * using APP_ERROR_CHECK.
 *
 * @param[in] callback a function to call when BLE events happen.
 */
void init(EventCallback callback = nullptr);

/**
 * Disconnects from the current connection.
 */
void disconnect();

/**
 * Starts advertising.
 */
void advertise();

/**
 * Starts advertising without a whitelist.
 */
void advertise_no_whitelist();

/**
 * Returns the negotiated max data length.
 */
std::uint16_t max_data_length();

/**
 * Sends data over the NUS service.
 */
void send(std::uint8_t *data, std::uint16_t length);

/**
 * Register callback for when data is written to this device. Callbacks should copy the data
 * somewhere else for processing.
 */
void register_callback(void *context, UartCallback callback);

}  // namespace ble
