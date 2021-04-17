/**
 * truetouch.cpp - a simple protocol written on top of a BLE UART for controlling pins on
 *                 the TrueTouch device.
 *
 * Copyright (c) 2021 TrueTouch
 * Distributed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT)
 */

#include "truetouch.hpp"

#include "boards_inc.h"
#include "nordic_pwm.hpp"
#include "util.hpp"

#include <nrf_log.h>
#include <nrf_gpio.h>

#include <cstring>

APP_TIMER_DEF(MY_TEST_TIMER);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

TrueTouch::TrueTouch() : _buffer {}, _buffer_cnt {}, _pulse_pin_bitset {}, _pulse_dur_ms {},
                         _current_pulse_bit {NO_ACTIVE_BIT} {
    ble::register_callback(this, ble_uart_callback);
    APP_ERROR_CHECK(
        app_timer_create(&_timer, APP_TIMER_MODE_SINGLE_SHOT, timer_timeout_callback));

    for (std::size_t i = 0; i < SOLENOID_COUNT; ++i) {
        nrf_gpio_cfg_output(finger_to_solenoid_pin(i));
    }

    pwm::init();
}

TrueTouch::~TrueTouch() {

}

void TrueTouch::service() {
    /* Do nothing if there's no data */
    if (_buffer_cnt == 0) {
        return;
    }

    /* First byte is command, peek that. */
    const Command command = static_cast<Command>(_buffer[0]);

    /* Perform command-specific processing (break if all data bytes aren't recieved yet) */
    switch (command) {
        case Command::SOLENOID_WRITE: {
            handle_solenoid_write();
        } break;

        case Command::SOLENOID_PULSE: {
            handle_solenoid_pulse();
        } break;

        case Command::ERM_SET: {
            handle_erm_set();
        } break;
    }

    /* For debugging/benchmarking: send an ACK to get a rough RTT measurement. */
#ifdef BENCHMARK_TIMING
    std::uint8_t ack_msg[] = {
        static_cast<std::uint8_t>(command),
        's', 'u', 'c', 'c', 'e', 's', 's'
    };
    ack(ack_msg, sizeof(ack_msg));

    /* Set pin high from when we get data to when we finish handling it. */
    nrf_gpio_pin_clear(FEATHER_SDA_PIN);
#endif
}

int TrueTouch::finger_to_solenoid_pin(Finger finger) {
    switch (finger) {
        case Finger::THUMB:
            return SOLENOID_THUMB;

        case Finger::INDEX:
            return SOLENOID_INDEX;

        case Finger::MIDDLE:
            return SOLENOID_MIDDLE;

        case Finger::RING:
            return SOLENOID_RING;

        case Finger::PINKY:
            return SOLENOID_PINKY;

        case Finger::PALM:
        default:
            return -1;
    }
}

int TrueTouch::finger_to_erm_pin(Finger finger) {
    switch (finger) {
        case Finger::THUMB:
            return ERM_THUMB;

        case Finger::INDEX:
            return ERM_INDEX;

        case Finger::MIDDLE:
            return ERM_MIDDLE;

        case Finger::RING:
            return ERM_RING;

        case Finger::PINKY:
            return ERM_PINKY;

        case Finger::PALM:
            return ERM_PALM;

        default:
            return -1;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Internal Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

void TrueTouch::handle_solenoid_write() {
    if (_buffer_cnt < sizeof(SolenoidWrite)) {
        return; // missing bytes
    }

    /* Parse byte buffer into the struct */
    auto params = parse_bytes<SolenoidWrite>();

    /* Fix endianness and mask any extraneous values */
    params.finger_bitset = util::byte_swap32(params.finger_bitset) & SOLENOID_MASK;

    NRF_LOG_DEBUG("%s: finger bitset=0x%x value=%s",
        __func__, params.finger_bitset,
        params.output == GpioOutput::OUT_HIGH ? "high" : "low");

    if (params.finger_bitset == 0) {
        NRF_LOG_WARNING("%s: no fingers set", __func__);
        return;
    }

    /* Go through each bit and set appropriate pins */
    for (int pin_idx = 0; pin_idx < SOLENOID_COUNT; ++pin_idx) {
        if (util::is_set(params.finger_bitset, pin_idx)) {
            auto pin = finger_to_solenoid_pin(pin_idx);
            nrf_gpio_pin_write(pin, params.output == GpioOutput::OUT_HIGH);
        }
    }
}

void TrueTouch::handle_solenoid_pulse() {
    if (_buffer_cnt < sizeof(SolenoidPulse)) {
        return; // missing bytes
    }

    /* Stop the timer to avoid concurrency isses. */
    APP_ERROR_CHECK(app_timer_stop(_timer));

    /* Parse byte buffer into the struct */
    auto params = parse_bytes<SolenoidPulse>();

    /* Fix endianness and mask any extraneous values */
    params.finger_bitset = util::byte_swap32(params.finger_bitset) & SOLENOID_MASK;
    params.duration_ms = util::byte_swap32(params.duration_ms);

    NRF_LOG_DEBUG("%s (%s): finger bitset=0x%x duration=%dms",
        __func__,
        TRUETOUCH_PULSE_PARALLEL ? "parallel" : "sequential",
        params.finger_bitset, params.duration_ms);

    if (params.finger_bitset == 0) {
        NRF_LOG_WARNING("%s: no fingers set", __func__);
    }

    /* Store data for later use, without overwriting any current fingers that still
     * need to be pulsed. */
    _pulse_pin_bitset |= params.finger_bitset;
    _pulse_dur_ms = params.duration_ms;

    if constexpr (TRUETOUCH_PULSE_PARALLEL) {
        handle_solenoid_pulse_parallel();
    } else {
        handle_solenoid_pulse_sequential();
    }
}

void TrueTouch::handle_solenoid_pulse_sequential() {
    /* Either a pulse is ongoing or it isn't; if there's no active pin,
     * choose the highest one from the bitset to be active. */
    if (_current_pulse_bit == NO_ACTIVE_BIT) {
        _current_pulse_bit = util::get_highest_bit(_pulse_pin_bitset);
    }
    auto pin = finger_to_solenoid_pin(_current_pulse_bit);

    NRF_LOG_DEBUG("Pulsing pin %d for %d", pin, _pulse_dur_ms);
    nrf_gpio_pin_set(pin);

    /* (Re)start the timer. */
    APP_ERROR_CHECK(app_timer_start(_timer, APP_TIMER_TICKS(_pulse_dur_ms), this));
}

void TrueTouch::handle_solenoid_pulse_parallel() {
    /* Set all pins high and (re)start the timer. */
    for (int pin_idx = 0; pin_idx < SOLENOID_COUNT; ++pin_idx) {
        if (util::is_set(_pulse_pin_bitset, pin_idx)) {
            auto pin = finger_to_solenoid_pin(pin_idx);
            nrf_gpio_pin_set(pin);
        }
    }

    APP_ERROR_CHECK(app_timer_start(_timer, APP_TIMER_TICKS(_pulse_dur_ms), this));
}

void TrueTouch::handle_erm_set() {
    if (_buffer_cnt < sizeof(ErmSet)) {
        return; // missing bytes
    }

    /* Parse byte buffer into the struct */
    auto params = parse_bytes<ErmSet>();

    /* Fix endianness for multi-byte pieces of data */
    params.finger_bitset = util::byte_swap32(params.finger_bitset);

    NRF_LOG_DEBUG("PWM_SET: finger bitset=0x%x intensity=%d",
        params.finger_bitset, params.intensity);

    /* Go through each bit and set PWM on appropriate pins */
    for (int pin_idx = 0; pin_idx < ERM_COUNT; ++pin_idx) {
        if (util::is_set(params.finger_bitset, pin_idx)) {
            auto pin = finger_to_erm_pin(pin_idx);
            /* Protocol currently uses 1-byte (i.e. 0-255) for PWM intensity */
            pwm::set_duty_cycle(pin, pwm::map_to_duty_cycle(params.intensity, 0, 255));
        }
    }
}

#ifdef BENCHMARK_TIMING
void TrueTouch::ack(std::uint8_t *bytes, std::uint16_t len) {
    ble::send(bytes, len);
}
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
// Internal Callbacks
////////////////////////////////////////////////////////////////////////////////////////////////////

void TrueTouch::ble_uart_callback(void *context, const std::uint8_t *data, std::uint16_t length)
{
    auto *_this = reinterpret_cast<TrueTouch *>(context);

    std::size_t remaining_length = BUFFER_SIZE - _this->_buffer_cnt;
    if (length > remaining_length) { // truncate... should never happen though
       NRF_LOG_WARNING("Truncating incoming BLE data from %d to %d", length, remaining_length);
       length = remaining_length;
    }

    std::memcpy(&_this->_buffer[_this->_buffer_cnt], data, length);
    _this->_buffer_cnt += length;
}

void TrueTouch::timer_timeout_callback(void *context)
{
    auto *_this = reinterpret_cast<TrueTouch *>(context);

    if (!_this->_pulse_pin_bitset) {
        NRF_LOG_WARNING("Timer timed out with no pins to pulse!");
        return;
    }

    if constexpr (TRUETOUCH_PULSE_PARALLEL) {
        _this->timer_timeout_callback_parallel();
    } else {
        _this->timer_timeout_callback_sequential();
    }
}

void TrueTouch::timer_timeout_callback_parallel() {
    /* Turn off all the pins */
    for (int pin_idx = 0; pin_idx < SOLENOID_COUNT; ++pin_idx) {
        if (util::is_set(_pulse_pin_bitset, pin_idx)) {
            auto pin = finger_to_solenoid_pin(pin_idx);
            nrf_gpio_pin_clear(pin);
        }
    }

    /* Clear variables */
    _pulse_pin_bitset = 0;
    _pulse_dur_ms = 0;
}

void TrueTouch::timer_timeout_callback_sequential() {
    /* Turn off the current pin and shift the queue. */
    auto pin = finger_to_solenoid_pin(_current_pulse_bit);
    nrf_gpio_pin_clear(pin);
    util::clear_bit(_pulse_pin_bitset, _current_pulse_bit);

    /* Stop if no more pins to pulse */
    if (!_pulse_pin_bitset) {
        _pulse_dur_ms = 0;
        _current_pulse_bit = NO_ACTIVE_BIT;
        return;
    }

    /* Start the next pin */
    _current_pulse_bit = util::get_highest_bit(_pulse_pin_bitset);
    pin = finger_to_solenoid_pin(_current_pulse_bit);
    nrf_gpio_pin_set(pin);
    APP_ERROR_CHECK(app_timer_start(_timer, APP_TIMER_TICKS(_pulse_dur_ms), this));
}
