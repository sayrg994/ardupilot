#include "AP_Camera_Tracking.h"

#if AP_CAMERA_TRACKING_ENABLED

extern const AP_HAL::HAL& hal;

// set tracking to none, point or rectangle (see TrackingType enum)
// if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
// p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
bool AP_Camera_Tracking::set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2, uint8_t tracking_device_sysid, uint8_t tracking_device_compid, mavlink_camera_information_t _cam_info)
{
    // if we don't support the required tracking then return
    switch (tracking_type) {
        case TrackingType::TRK_NONE:
            break;
        case TrackingType::TRK_POINT:
            if (!(_cam_info.flags & CAMERA_CAP_FLAGS_HAS_TRACKING_POINT)) {
               return false;
            }
            break;
        case TrackingType::TRK_RECTANGLE:
            if (!(_cam_info.flags & CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE)) {
               return false;
            }
            break;
        default:
            return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO,"Tracking: New Tracking request");
    uint8_t proxy_device_compid=0, proxy_device_sysid=0;
    auto _link = GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_ONBOARD_CONTROLLER, proxy_device_compid, proxy_device_sysid);
    if (_link == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING,"AP_Camera: Could Not find any onboard controller registered");
    }
    // prepare and send message
    mavlink_command_long_t pkt {};

    pkt.confirmation=0;
    pkt.target_component=tracking_device_compid;
    pkt.target_system = tracking_device_sysid;
    pkt.param1 = 0;
    pkt.param2 = 0;
    pkt.param3 = 0;
    pkt.param4 = 0;
    pkt.param5 = 0;
    pkt.param6 = 0;
    pkt.param7 = 0;

    if (tracking_type == TrackingType::TRK_POINT) {
        pkt.command = MAV_CMD_CAMERA_TRACK_POINT;
        pkt.param1 = p1.x;
        pkt.param2 = p1.y;
    } else if (tracking_type == TrackingType::TRK_RECTANGLE) {
        pkt.command = MAV_CMD_CAMERA_TRACK_RECTANGLE;
        pkt.param1 = p1.x;
        pkt.param2 = p1.y;
        pkt.param3 = p2.x;
        pkt.param4 = p2.y;
    } else if (tracking_type == TrackingType::TRK_NONE) {
        pkt.command = MAV_CMD_CAMERA_STOP_TRACKING;
    }

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);
    gcs().send_text(MAV_SEVERITY_WARNING,"proxy device sysid %d and comp %d",proxy_device_sysid,proxy_device_compid);
    gcs().send_text(MAV_SEVERITY_WARNING,"sent message to device sysid %d and comp %d",tracking_device_sysid,tracking_device_compid);
    return true;
}

#endif // AP_CAMERA_TRACKING_ENABLED