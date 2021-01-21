/* bleuart_pin_ctrl.cpp
 * A simple pin control class that utilizes BLE UART to control pins on this device.
 * Functionality:
 *      - Control GPIO (set, clear, toggle)
 *      - Control PWM (0-255)
 *      - Get status
 */

#include "bleuart_pin_ctrl.hpp"

#include "nordic_pwm.hpp"
#include "util.hpp"

#include <nrf_log.h>
#include <nrf_gpio.h>

#include <cstring>

namespace ble_uart_pin_ctrl {

APP_TIMER_DEF(MY_TEST_TIMER);

void PinCtrl::service() {
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
    params.gpio_port = util::byte_swap32(params.gpio_port);
    params.gpio_bitset = util::byte_swap32(params.gpio_bitset);

    NRF_LOG_DEBUG("GPIO_CONFIGURE: %x %d", params.gpio_bitset, params.gpio_direction);

    /* Go through each bit and configure appropriate pins */
    for (int pin_idx = 0; pin_idx < 32; ++pin_idx) {
        if (util::is_set(params.gpio_bitset, pin_idx)) {
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
    params.gpio_port = util::byte_swap32(params.gpio_port);
    params.gpio_bitset = util::byte_swap32(params.gpio_bitset);

    NRF_LOG_DEBUG("GPIO_WRITE: %x", params.gpio_bitset);

    /* Go through each bit and set appropriate pins */
    for (int pin_idx = 0; pin_idx < 32; ++pin_idx) {
        if (util::is_set(params.gpio_bitset, pin_idx)) {
            auto pin = NRF_GPIO_PIN_MAP(params.gpio_port, pin_idx);
            nrf_gpio_pin_write(pin, params.output == GpioOutput::OUT_HIGH);
        }
    }
}

// APP_TIMER_TICKS
void PinCtrl::handle_gpio_pulse() {
    if (_buffer_cnt < sizeof(GpioPulse)) {
        return; // missing bytes
    }

    /* Parse byte buffer into the struct */
    auto params = parse_bytes<GpioPulse>();
    // fix endianness for multi-byte pieces of data
    params.gpio_port = util::byte_swap32(params.gpio_port);
    params.gpio_bitset = util::byte_swap32(params.gpio_bitset);
    params.duration_ms = util::byte_swap32(params.duration_ms);

    NRF_LOG_DEBUG("GPIO_PULSE: %x %d", params.gpio_bitset, params.duration_ms);

    /* Store which pins to pulse */
    for (int pin_idx = 0; pin_idx < 32 && _pulse_cnt < SOLENOID_COUNT; ++pin_idx) {
        if (util::is_set(params.gpio_bitset, pin_idx)) {
            auto pin = NRF_GPIO_PIN_MAP(params.gpio_port, pin_idx);
            _pulse_pins[_pulse_cnt] = pin;
            ++_pulse_cnt;
        }
    }

    /* Store data for use by pin pulsing routine */
    _pulse_dur_ms = params.duration_ms;

    /* Set the first pin high and start the timer. */
    auto pin = _pulse_pins[0];
    NRF_LOG_DEBUG("Pulsing pin %d for %d", pin, _pulse_dur_ms);
    nrf_gpio_pin_set(pin);
    APP_ERROR_CHECK(app_timer_start(_timer, APP_TIMER_TICKS(_pulse_dur_ms), this));
}

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
    params.gpio_port = util::byte_swap32(params.gpio_port);
    params.gpio_bitset = util::byte_swap32(params.gpio_bitset);

    NRF_LOG_DEBUG("PWM_SET: %x %d", params.gpio_bitset, params.intensity);

    /* Go through each bit and set PWM on appropriate pins */
    for (int pin_idx = 0; pin_idx < 32; ++pin_idx) {
        if (util::is_set(params.gpio_bitset, pin_idx)) {
            auto pin = NRF_GPIO_PIN_MAP(params.gpio_port, pin_idx);
            // TODO CMK (1/21/21): this will be the wrong duty cycle... need to adjust/map some things
            pwm::set_duty_cycle(pin, params.intensity);
        }
    }
}

void PinCtrl::handle_query_state() {
    NRF_LOG_WARNING("QUERY STATE: TODO");
}

void PinCtrl::ble_uart_callback(void *context, const std::uint8_t *data, std::uint16_t length)
{
    auto *_this = reinterpret_cast<PinCtrl *>(context);

    std::size_t remaining_length = BUFFER_SIZE - _this->_buffer_cnt;
    if (length > remaining_length) { // truncate... should never happen though
       NRF_LOG_WARNING("Truncating incoming BLE data from %d to %d", length, remaining_length);
       length = remaining_length;
    }

    std::memcpy(&_this->_buffer[_this->_buffer_cnt], data, length);
    _this->_buffer_cnt += length;
}

void PinCtrl::timer_timeout_callback(void *context)
{
    auto *_this = reinterpret_cast<PinCtrl *>(context);

    if (_this->_pulse_cnt <= 0) {
        NRF_LOG_WARNING("Timer timed out with no pins to pulse!");
        return;
    }

    /* Turn off the current pin and shift the queue. */
    nrf_gpio_pin_clear(_this->_pulse_pins[0]);
    std::memmove(&_this->_pulse_pins[0], &_this->_pulse_pins[1],
        sizeof(_this->_pulse_pins) - sizeof(_this->_pulse_pins[0]));
    --_this->_pulse_cnt;

    /* Stop if no more pins to pulse */
    if (_this->_pulse_cnt <= 0) {
        return;
    }

    /* Start the next pin */
    nrf_gpio_pin_set(_this->_pulse_pins[0]);
    APP_ERROR_CHECK(app_timer_start(_this->_timer, APP_TIMER_TICKS(_this->_pulse_dur_ms), _this));
}

}  // namespace ble_uart_pin_ctrl
