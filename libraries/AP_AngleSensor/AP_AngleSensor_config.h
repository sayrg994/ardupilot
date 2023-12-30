#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_ANGLESENSOR_ENABLED
#define AP_ANGLESENSOR_ENABLED 0 //DISABLED by default
#endif

#define ANGLE_SENSOR_MAX_INSTANCES      2
#define ANGLE_SENSOR_BUS_DEFAULT        0
#define ANGLE_SENSOR_ADDR_DEFAULT       0x40
#define ANGLE_SENSOR_DEFAULT_OFFSET    0
#define ANGLE_SENSOR_DIRECTION_FORWARDS 1
