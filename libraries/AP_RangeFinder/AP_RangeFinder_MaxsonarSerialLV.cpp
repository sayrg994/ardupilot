/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder_MaxsonarSerialLV.h"

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>

#define MAXSONAR_SERIAL_LV_BAUD_RATE 9600

extern const AP_HAL::HAL& hal;

AP_RangeFinder_MaxsonarSerialLV::AP_RangeFinder_MaxsonarSerialLV(
    RangeFinder::RangeFinder_State &_state,
    AP_RangeFinder_Params &_params):
    AP_RangeFinder_Backend_Serial(_state, _params)
{
    if (_backend_type == RangeFinder::Type::MBSER) {
        unit_conversion = 0.0254f;
    } else if (_backend_type == RangeFinder::Type::MBSER_CM) {
        unit_conversion = 0.01f;
    } else if (_backend_type == RangeFinder::Type::MBSER_MM) {
        unit_conversion = 0.001f;
    }
}

// read - return last value measured by sensor
bool AP_RangeFinder_MaxsonarSerialLV::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    int32_t sum = 0;
    int16_t nbytes = uart->available();
    uint16_t count = 0;

    while (nbytes-- > 0) {
        char c = uart->read();
        if (c == '\r') {
            linebuf[linebuf_len] = 0;
            sum += (int)atoi(linebuf);
            count++;
            linebuf_len = 0;
        } else if (isdigit(c)) {
            linebuf[linebuf_len++] = c;
            if (linebuf_len == sizeof(linebuf)) {
                // too long, discard the line
                linebuf_len = 0;
            }
        }
    }

    if (count == 0) {
        return false;
    }

    // Maxbotix serial sonar may give the metrics in inches, cm or mm, so we have to transform this to meters
    reading_m = unit_conversion * (float(sum) / count);

    return true;
}
