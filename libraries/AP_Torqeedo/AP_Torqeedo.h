/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_TORQEEDO_ENABLED
#define HAL_TORQEEDO_ENABLED (BOARD_FLASH_SIZE > 1024)
#endif

#if HAL_TORQEEDO_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_HAL/Semaphores.h>

#define TORQEEDO_MESSAGE_LEN_MAX    30  // messages are no more than 30 bytes

class AP_Torqeedo {
public:
    AP_Torqeedo();

    CLASS_NO_COPY(AP_Torqeedo);

    static AP_Torqeedo* get_singleton();

    // initialise driver
    void init();

    // consume incoming messages from motor, reply with latest motor speed
    // runs in background thread
    void thread_main();

    // returns true if communicating with the motor
    bool healthy();

    // run pre-arm check.  returns false on failure and fills in failure_msg
    // any failure_msg returned will not include a prefix
    bool pre_arm_checks(char *failure_msg, uint8_t failure_msg_len);

    static const struct AP_Param::GroupInfo var_info[];

private:

    // message ids
    enum class MsgId : uint8_t {
        SET_MOTOR_SPEED = 0x0,
        UNKNOWN_0x01 = 0x1,
        REQUEST_MOTOR_SPEED = 0x14,
        UNKNOWN_0x20 = 0x20,
        UNKNOWN_0x30 = 0x30,
    };

    enum class ParseState {
        WAITING_FOR_HEADER = 0,
        WAITING_FOR_FOOTER,
    };

    // TYPE parameter values
    enum class ConnectionType : uint8_t {
        TYPE_DISABLED = 0,
        TYPE_TILLER = 1,
        TYPE_MOTOR = 2
    };

    // OPTIONS parameter values
    enum options {
        LOG             = 1<<0,
        DEBUG_TO_GCS    = 1<<1,
    };

    // initialise serial port and gpio pins (run from background thread)
    // returns true on success
    bool init_internals();

    // returns true if the driver is enabled
    bool enabled() const;

    // process a single byte received on serial port
    // return true if a complete message has been received (the message will be held in _received_buff)
    bool parse_byte(uint8_t b);

    // returns true if it is safe to send a message
    bool safe_to_send() const { return (_send_delay_us == 0); }

    // set pin to enable sending commands to motor
    void send_start();

    // check for timeout after sending and unset pin if required
    void check_for_send_end();

    // calculate delay require to allow bytes to be sent
    uint32_t calc_send_delay_us(uint8_t num_bytes);

    // send a motor speed command as a value from -1000 to +1000
    // value is taken directly from SRV_Channel
    void send_motor_speed_cmd();

    // output logging and debug messages (if required)
    // force_logging should be true if caller wants to ensure the latest status is logged
    void log_and_debug(bool force_logging);

    // parameters
    AP_Enum<ConnectionType> _type;      // connector type used (0:disabled, 1:tiller connector, 2: motor connector)
    AP_Int8 _pin_onoff;     // Pin number connected to Torqeedo's on/off pin. -1 to disable turning motor on/off from autopilot
    AP_Int8 _pin_de;        // Pin number connected to RS485 to Serial converter's DE pin. -1 to disable sending commands to motor
    AP_Int16 _options;      // options bitmask

    // members
    AP_HAL::UARTDriver *_uart;      // serial port to communicate with motor
    bool _initialised;              // true once driver has been initialised
    bool _send_motor_speed;         // true if motor speed should be sent at next opportunity
    int16_t _motor_speed;           // desired motor speed (set from within update method)
    uint32_t _last_send_motor_ms;   // system time (in millis) last motor speed command was sent (used for health reporting)
    uint32_t _last_send_motor_us;   // system time (in micros) last motor speed command was sent (used for timing to unset DE pin)
    uint32_t _send_delay_us;        // delay (in micros) to allow bytes to be sent after which pin can be unset.  0 if not delaying

    // health reporting
    HAL_Semaphore _last_healthy_sem;// semaphore protecting reading and updating of _last_send_motor_ms and _last_received_ms
    uint32_t _last_debug_ms;        // system time (in millis) that debug was last output

    // message parsing members
    ParseState _parse_state;        // current state of parsing
    uint32_t _parse_error_count;    // total number of parsing errors (for reporting)
    uint32_t _parse_success_count;  // number of messages successfully parsed (for reporting)
    uint8_t _received_buff[TORQEEDO_MESSAGE_LEN_MAX];   // characters received
    uint8_t _received_buff_len;     // number of characters received
    uint32_t _last_received_ms;     // system time (in millis) that a message was successfully parsed (for health reporting)

    static AP_Torqeedo *_singleton;
};

namespace AP {
    AP_Torqeedo *torqeedo();
};

#endif // HAL_TORQEEDO_ENABLED
