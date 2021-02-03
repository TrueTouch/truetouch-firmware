/**
 * truetouch.hpp - a simple protocol written on top of a BLE UART for controlling pins on
 *                 the TrueTouch device.
 *
 * Copyright (c) 2021 TrueTouch
 * Distributed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT)
 */

#pragma once

#include "nordic_ble.hpp"

#include <app_error.h>
#include <app_timer.h>

#include <cstdint>
#include <climits>
#include <cstddef>
#include <cstring>

class TrueTouch {
public:
////////////////////////////////////////////////////////////////////////////////////////////////////
// Public Types
////////////////////////////////////////////////////////////////////////////////////////////////////
    /** The type constituting a bitset. */
    using Bitset = std::uint32_t;

    /** Types of commands. */
    enum class Command : std::uint8_t {
        SOLENOID_WRITE = 0x01,  /*!< Digital write to the given fingers' solenoids. */
        SOLENOID_PULSE = 0x02,  /*!< Pulse given fingers' solenoids for so many ms. */
        ERM_SET = 0x03,         /*!< Set PWM on given fingers' ERM motors. */
    };

    /** Fingers the TrueTouch device is connected to. */
    enum class Finger : std::uint8_t {
        THUMB = 0,
        INDEX = 1,
        MIDDLE = 2,
        RING = 3,
        PINKY = 4,
        PALM = 5,
    };

    /** Solenoid write options. */
    enum class GpioOutput : std::uint8_t {
        OUT_LOW = 0,
        OUT_HIGH = 1
    };

    /** Solenoid write parameters. */
    struct __attribute__((packed)) SolenoidWrite {
        Command command;
        Bitset finger_bitset; // n-th bit set configures n-th finger in the Finger enum
        GpioOutput output;
    };

    /** Solenoid pulse parameters. */
    struct __attribute__((packed)) SolenoidPulse {
        Command command;
        Bitset finger_bitset; // n-th bit set configures n-th finger in the Finger enum
        std::uint32_t duration_ms; // duration of pulse per gpio in ms
    };

    /** ERM set parameters. */
    struct __attribute__((packed)) ErmSet {
        Command command;
        Bitset finger_bitset; // n-th bit set configures n-th finger in the Finger enum
        std::uint8_t intensity; // 0-255
    };

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public Constants
////////////////////////////////////////////////////////////////////////////////////////////////////
    /** Size of the UART buffer. */
    static constexpr std::size_t BUFFER_SIZE { 256 };

    /** Max number of bits in a bitset. */
    static constexpr std::size_t BITSET_BIT_COUNT { sizeof(Bitset) * CHAR_BIT };

    /** Number of solenoids in the system. */
    static constexpr std::size_t SOLENOID_COUNT { 5 };

    /** Number of ERM motors in the system. */
    static constexpr std::size_t ERM_COUNT { 6 };

public:
////////////////////////////////////////////////////////////////////////////////////////////////////
// Public Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Initializes hardware used for TrueTouch. BLE should be initialized before this is called.
     */
    TrueTouch();

    virtual ~TrueTouch();

    /** Services any pending data read by this device. */
    void service();

    /** Util - returns the nRF pin corresponding to the given finger's solenoid.
     *  Returns -1 for the palm. */
    static int finger_to_solenoid_pin(Finger finger);
    static int finger_to_solenoid_pin(int finger) {
        APP_ERROR_CHECK_BOOL(finger < SOLENOID_COUNT);
        return finger_to_solenoid_pin(static_cast<Finger>(finger));
    }

    /** Util - returns the nRF pin corresponding to the given finger's ERM motor. */
    static int finger_to_erm_pin(Finger finger);
    static int finger_to_erm_pin(int finger) {
        APP_ERROR_CHECK_BOOL(finger < ERM_COUNT);
        return finger_to_erm_pin(static_cast<Finger>(finger));
    }

private:
////////////////////////////////////////////////////////////////////////////////////////////////////
// Internal Data
////////////////////////////////////////////////////////////////////////////////////////////////////

    /** Buffer used to read data recieved from BLE UART. */
    std::uint8_t _buffer[BUFFER_SIZE];
    volatile std::size_t _buffer_cnt;

    /** An app timer control block and instance pointer.
      * NOTE: this is defined without the macro so that it can be per class instance. This could
      *       introduce issues in later SDK versions, keep an eye on changes.
      */
    app_timer_t _timer_cb = {
        .active = false,
    };
    const app_timer_id_t _timer = &_timer_cb;

    /** Bitset of pins to be pulsed. */
    Bitset _pulse_pin_bitset;
    /** Duration of each pulse in ms. */
    std::uint32_t _pulse_dur_ms;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Internal Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

    /** Functions to handle commands */
    void handle_solenoid_write();
    void handle_solenoid_pulse();
    void handle_erm_set();

    /** Copy bytes from the BLE UART byte buffer into the type T */
    template <typename T>
    T parse_bytes() {
        if (_buffer_cnt < sizeof(T)) {
            /* Uh-oh... */
            APP_ERROR_HANDLER(0);
        } else {
            /* Grab the structure from the byte buffer. */
            T ret;
            std::memcpy(&ret, _buffer, sizeof(T));

            /* Done with these bytes, "erase" them. */
            std::size_t remaining = _buffer_cnt - sizeof(T);
            std::memmove(&_buffer[0], &_buffer[sizeof(T)], remaining);
            _buffer_cnt -= sizeof(T);

            return ret;
        }

        // subdue the return type warning (APP_ERROR_HANDLER will not return)
        return T{};
    }

    /** Callback to handle incoming UART data. */
    static void ble_uart_callback(void *context, const std::uint8_t *data, std::uint16_t length);

    /** Callback called when the timer expires. */
    static void timer_timeout_callback(void *context);
};
