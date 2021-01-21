/* bleuart_pin_ctrl.cpp
 * A simple pin control class that utilizes BLE UART to control pins on this device.
 * Functionality:
 *      - Control GPIO (set, clear, toggle)
 *      - Control PWM (0-255)
 *      - Get status
 */

#include "bleuart_pin_ctrl.h"

#include "nordic_pwm.hpp"

#include <nrf_log.h>
#include <nrf_gpio.h>

#include <cstring>

namespace ble_uart_pin_ctrl {

void PinCtrl::service() {
    /* Always service pin pulsing if it's ongoing */
    service_gpio_pulse();

    /* Do nothing if there's no data */
    if (_buffer_cnt == 0) {
        return;
    }

    /* First byte is command, peek that */
    const Command command = static_cast<Command>(_buffer[0]);

    /* Perform command-specific processing (break if all data bytes aren't recieved yet) */
    switch (command) {
        case Command::GPIO_CONFIGURE: {
            handle_gpio_configure();
        } break;

        case Command::GPIO_WRITE: {
            handle_gpio_write();
        } break;

        case Command::GPIO_PULSE: {
            handle_gpio_pulse();
        } break;

        case Command::GPIO_QUERY: {
            handle_gpio_query();
            // TODO CMK (11/14/20): implement
        } break;

        case Command::PWM_SET: {
            handle_pwm_set();
        } break;

        case Command::QUERY_STATE: {
            handle_query_state();
            // TODO CMK (11/14/20): implement
        } break;
    }
}

void PinCtrl::handle_gpio_configure() {
    if (_buffer_cnt < sizeof(GpioConfigure)) {
        return; // missing bytes
    }

    /* Parse byte buffer into the struct */
    auto params = parse_bytes<GpioConfigure>();
    // fix endianness for multi-byte pieces of data
    params.gpio_port = byte_swap32(params.gpio_port);
    params.gpio_bitset = byte_swap32(params.gpio_bitset);

    NRF_LOG_DEBUG("GPIO_CONFIGURE: %x %d", params.gpio_bitset, params.gpio_direction);

    /* Go through each bit and configure appropriate pins */
    for (int pin_idx = 0; pin_idx < 32; ++pin_idx) {
        if (is_set(params.gpio_bitset, pin_idx)) {
            auto pin = NRF_GPIO_PIN_MAP(params.gpio_port, pin_idx);
            if (params.gpio_direction == GpioDirection::DIR_INPUT) {
                nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_NOPULL);
            } else {
                nrf_gpio_cfg_output(pin);
                nrf_gpio_pin_clear(pin);
            }
        }
    }
}

void PinCtrl::handle_gpio_write() {
    if (_buffer_cnt < sizeof(GpioWrite)) {
        return; // missing bytes
    }

    /* Parse byte buffer into the struct */
    auto params = parse_bytes<GpioWrite>();
    // fix endianness for multi-byte pieces of data
    params.gpio_port = byte_swap32(params.gpio_port);
    params.gpio_bitset = byte_swap32(params.gpio_bitset);

    NRF_LOG_DEBUG("GPIO_WRITE: %x", params.gpio_bitset);

    /* Go through each bit and set appropriate pins */
    for (int pin_idx = 0; pin_idx < 32; ++pin_idx) {
        if (is_set(params.gpio_bitset, pin_idx)) {
            auto pin = NRF_GPIO_PIN_MAP(params.gpio_port, pin_idx);
            nrf_gpio_pin_write(pin, params.output == GpioOutput::OUT_HIGH);
        }
    }
}

#if 0 // TODO CMK (1/21/21): reimplement w/ timers?
void PinCtrl::handle_gpio_pulse() {
    if (_buffer_cnt < sizeof(GpioPulse)) {
        return; // missing bytes
    }

    /* Parse byte buffer into the struct */
    auto params = parse_bytes<GpioPulse>();
    // fix endianness for multi-byte pieces of data
    params.gpio_port = byte_swap32(params.gpio_port);
    params.gpio_bitset = byte_swap32(params.gpio_bitset);
    params.duration_ms = byte_swap32(params.duration_ms);

    NRF_LOG_DEBUG("GPIO_PULSE: %x %d", params.gpio_bitset, params.duration_ms);

    /* Store data for use by pin pulsing routine */
    _pins_to_pulse = params.gpio_bitset;
    _pulse_dur_ms = params.duration_ms;

    if (!_pins_to_pulse) { // nothing to do
        return;
    }

    /* Start the first pulse (set the pin high and record the start time) */
    int pin = PinCtrl::get_highest_bit(_pins_to_pulse);
    if (pin < 0) { // note - should never happen
        NRF_LOG_WARN("Pulse starting with no pin set");
        return;
    }

    NRF_LOG_DEBUG("Pulsing pin %d for %d", pin, _pulse_dur_ms);

    digitalWrite(pin, HIGH);
    _pulse_start_ms = millis();
}
#else
void PinCtrl::handle_gpio_pulse() {}
#endif

void PinCtrl::handle_gpio_query() {
    NRF_LOG_WARNING("GPIO QUERY: TODO");
}

void PinCtrl::handle_pwm_set() {
    if (_buffer_cnt < sizeof(PwmSet)) {
        return; // missing bytes
    }

    /* Parse byte buffer into the struct */
    auto params = parse_bytes<PwmSet>();
    // fix endianness for multi-byte pieces of data
    params.gpio_port = byte_swap32(params.gpio_port);
    params.gpio_bitset = byte_swap32(params.gpio_bitset);

    NRF_LOG_DEBUG("PWM_SET: %x %d", params.gpio_bitset, params.intensity);

    /* Go through each bit and set PWM on appropriate pins */
    for (int pin_idx = 0; pin_idx < 32; ++pin_idx) {
        if (is_set(params.gpio_bitset, pin_idx)) {
            auto pin = NRF_GPIO_PIN_MAP(params.gpio_port, pin_idx);
            // TODO CMK (1/21/21): this will be the wrong duty cycle... need to adjust/map some things
            pwm::set_duty_cycle(pin, params.intensity);
        }
    }
}

void PinCtrl::handle_query_state() {
    NRF_LOG_WARNING("QUERY STATE: TODO");
}

#if 0 // TODO CMK (1/21/21): implement with timers?
void PinCtrl::service_gpio_pulse() {
    /* Do nothing if no pulsing is ongoing */
    if (!_pins_to_pulse) {
        return;
    }

    /* Do nothing if pulse time hasn't elapsed yet */
    if (millis() - _pulse_start_ms < _pulse_dur_ms) {
        return;
    }

    /* Set the pin low */
    int pin = PinCtrl::get_highest_bit(_pins_to_pulse);
    if (pin < 0) { // note - should never happen
        Serial.println("!!! Pulse active with no pin set");
        return;
    }

    digitalWrite(pin, LOW);
    PinCtrl::clear_highest_bit(_pins_to_pulse);

    /* If there's nothing left to do, stop */
    if (!_pins_to_pulse) {
        DBG_LOG("Done with pulsing");
        _pulse_start_ms = 0;
        return;
    }

    /* Start next pulse */
    pin = PinCtrl::get_highest_bit(_pins_to_pulse);
    if (pin < 0) { // note - should never happen
        Serial.println("!!! Next pulse starting with no pin set");
        return;
    }

    DBG_LOG("Pulsing pin ");
    DBG_LOG(pin);
    DBG_LOG(" for ");
    DBG_LOG_LINE(_pulse_dur_ms);

    digitalWrite(pin, HIGH);
    _pulse_start_ms = millis();
}
#else
void PinCtrl::service_gpio_pulse() {}
#endif

void PinCtrl::callback(const std::uint8_t *data, std::uint16_t length)
{
    std::size_t remaining_length = BUFFER_SIZE - _buffer_cnt;
    if (length > remaining_length) { // truncate... should never happen though
       NRF_LOG_WARNING("Truncating incoming BLE data from %d to %d", length, remaining_length);
       length = remaining_length;
    }

    std::memcpy(&_buffer[_buffer_cnt], data, length);
    _buffer_cnt += length;
}

}  // namespace ble_uart_pin_ctrl
