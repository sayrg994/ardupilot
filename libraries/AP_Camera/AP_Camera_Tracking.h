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

/*
  Camera Tracking Library
 */
#pragma once

#include <GCS_MAVLink/GCS.h>
#include "AP_Camera_shareddefs.h"
// #include "AP_Camera_Backend.h"
#include "AP_Camera_config.h"

#if AP_CAMERA_TRACKING_ENABLED

// #include "AP_Camera.h"

class AP_Camera_Tracking
{
public:

    // Constructor
    AP_Camera_Tracking() {
        
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Camera_Tracking);

    // initialize the camera tracking library
    void init();

    // set tracking to non      nj klbhnjk nm,e, point or rectangle (see TrackingType enum)
    // if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
    // p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
    bool set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2, uint8_t tracking_device_sysid, uint8_t tracking_device_compid, mavlink_camera_information_t _cam_info);

    // handle MAVLink messages from the camera
    void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg);

//     // configure camera
//     void configure(float shooting_mode, float shutter_speed, float aperture, float ISO, int32_t exposure_type, int32_t cmd_id, float engine_cutoff_time) override;

//     // send camera settings message to GCS
//     void send_camera_settings(mavlink_channel_t chan) const override;

//     // send camera information message to GCS
//     void send_camera_information(mavlink_channel_t chan) const override;
};

#endif // AP_CAMERA_TRACKING_ENABLED
