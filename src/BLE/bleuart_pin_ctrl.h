/* bleuart_pin_ctrl.h
 * A simple pin control class that utilizes BLE UART to control pins on this device.
 * Functionality:
 *      - Control GPIO (set, clear, toggle, pulse)
 *      - Control PWM (0-255)
 *      - Get status
 */

#pragma once

#include <app_error.h>

#include <cstdint>
#include <cstddef>

namespace ble_uart_pin_ctrl {

/**
 * Messages shall be 1 command byte followed by data byte(s).
 * Data bytes will be organized based on structs below.
 */

/** Types of commands */
enum class Command : std::uint8_t {
    /** GPIO commands */
    GPIO_CONFIGURE = 0x01,  /*!< Central -> this: configure GPIO pin(s) */
    GPIO_WRITE = 0x02,      /*!< Central -> this: digital write on output GPIO pin(s) */
    GPIO_PULSE = 0x03,      /*!< Central -> this: pulse GPIO pin(s) for so many ms */
    // TODO CMK (11/14/20): implement or delete
    GPIO_QUERY = 0x04,      /*!< this -> Central: get GPIO status info */

    /** PWM commands */
    PWM_SET = 0x05,         /*!< Central -> this: set PWM output on pin(s) */

    /** Query commands */
    // TODO CMK (11/14/20): implement or delete
    QUERY_STATE = 0x06,     /*!< this -> Central: get info about device state */
};

/** GPIO direction options */
enum class GpioDirection : std::uint8_t {
    DIR_INPUT = 0,
    DIR_OUTPUT
};

/** GPIO digital write options */
enum class GpioOutput : std::uint8_t {
    OUT_LOW = 0,
    OUT_HIGH = 1
};

/** GPIO configure parameter */
/* Note: when GPIO are configured as output, they are set to low as well */
struct __attribute__((packed)) GpioConfigure {
    Command command;
    std::uint32_t gpio_port; // unused on Arduino
    std::uint32_t gpio_bitset; // if the n-th bit is 1, GPIO n is being configured
    GpioDirection gpio_direction;
};

/** GPIO set parameters */
struct __attribute__((packed)) GpioWrite {
    Command command;
    std::uint32_t gpio_port; // unused on Arduino
    std::uint32_t gpio_bitset; // if the n-th bit is 1, GPIO n is being configured
    GpioOutput output;
};

/** GPIO toggle parameters */
struct __attribute__((packed)) GpioToggle {
    Command command;
    std::uint32_t gpio_port; // unused on Arduino
    std::uint32_t gpio_bitset; // if the n-th bit is 1, GPIO n is being configured
};

/** GPIO pulse parameters */
struct __attribute__((packed)) GpioPulse {
    Command command;
    std::uint32_t gpio_port; // unused on Arduino
    std::uint32_t gpio_bitset; // if the n-th bit is 1, GPIO n is being configured
    std::uint32_t duration_ms; // duration of pulse per gpio in ms
};

/** GPIO query */
struct __attribute__((packed)) GpioQuery {
    // TODO
};

/** PWM set parameters */
struct __attribute__((packed)) PwmSet {
    Command command;
    std::uint32_t gpio_port; // unused on Arduino
    std::uint32_t gpio_bitset; // if the n-th bit is 1, GPIO n is being configured
    std::uint8_t intensity; // 0-255
};

/** Query state parameters */
struct __attribute__((packed)) QueryState {
    // TODO
};

class PinCtrl {
public:
    /** Size of the UART buffer. */
    static constexpr std::size_t BUFFER_SIZE { 256 };

private:
    /** Buffer used to read data recieved from BLE UART. */
    std::uint8_t _buffer[BUFFER_SIZE];
    volatile std::size_t _buffer_cnt;

    /** Functions to handle commands */
    void handle_gpio_configure();
    void handle_gpio_write();
    void handle_gpio_pulse();
    void handle_gpio_query();
    void handle_pwm_set();
    void handle_query_state();

    /** Service any ongoing pulse command */
    void service_gpio_pulse();

    /** Copy bytes from the BLE UART byte buffer into the type T */
    template <typename T>
    T parse_bytes() {
        if (_buffer_cnt < sizeof(T)) {
            /** Uh-oh... */
            APP_ERROR_HANDLER(0);
        } else {
            T ret;
            memcpy(&ret, _buffer, sizeof(T));
            return ret;
        }
    }

public:
    PinCtrl() {}
    virtual ~PinCtrl() {}

    /** Services any pending data read by the  */
    void service();

    /** Callback to handle incoming UART data. */
    // TODO: add contexts to callbacks
    static void callback(const std::uint8_t *data, std::uint16_t length);

    /* Misc utils/helpers */

    // fix endianness (network to host)
    static std::uint32_t byte_swap32(std::uint32_t input) {
        return __builtin_bswap32(input);
    }

    // check if bit n is set in the mask
    static bool is_set(std::uint32_t mask, unsigned int n) {
        if (n > 31) {
            return false;
        }
        return mask & (1UL << n);
    }

    // return the bit position of the highest set bit (-1 if no bits are set)
    static int get_highest_bit(std::uint32_t mask) {
        for (int i = 31; i >= 0; --i) {
            if (mask & (1UL << i)) {
                return i;
            }
        }
        return -1;
    }

    // clears the bit position of the highest set bit
    static void clear_highest_bit(std::uint32_t &mask) {
        int n = get_highest_bit(mask);
        if (n >= 0) {
            mask &= ~(1UL << n);
        }
    }
};

}  // namespace ble_uart_pin_ctrl
