/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// counter to verify landings
static uint16_t land_detector = LAND_DETECTOR_TRIGGER;  // we assume we are landed

// land_complete_maybe - return true if we may have landed (used to reset loiter targets during landing)
static bool land_complete_maybe()
{
    return (ap.land_complete || ap.land_complete_maybe);
}

// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// called at 50hz
static void update_land_detector()
{
    bool motor_at_lower_limit = motors.limit.throttle_lower;
    Vector3f accel_ef = ahrs.get_accel_ef_blended();
    accel_ef.z += GRAVITY_MSS;
    bool accel_stationary = (accel_ef.length() < 1.0f);

    if ( motor_at_lower_limit && accel_stationary) {
        if (!ap.land_complete) {
            // increase counter until we hit the trigger then set land complete flag
            if( land_detector < LAND_DETECTOR_TRIGGER) {
                land_detector++;
            }else{
                set_land_complete(true);
                land_detector = LAND_DETECTOR_TRIGGER;
            }
        }
    } else {
        // we've sensed movement up or down so reset land_detector
        land_detector = 0;
        // if throttle output is high then clear landing flag
        if (motors.get_throttle_out() > get_non_takeoff_throttle()) {
            set_land_complete(false);
        }
    }

    // set land maybe flag
    set_land_complete_maybe(land_detector >= LAND_DETECTOR_MAYBE_TRIGGER);
}

// update_throttle_low_comp - sets motors throttle_low_comp value depending upon vehicle state
//  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
//  has no effect when throttle is above hover throttle
static void update_throttle_low_comp()
{
    // manual throttle
    if (mode_has_manual_throttle(control_mode)) {
        if(!motors.armed() || g.rc_3.control_in <= 0) {
            motors.set_throttle_low_comp(0.1f);
        } else {
            motors.set_throttle_low_comp(0.5f);
        }
    } else {
        // autopilot controlled throttle
        const Vector3f angle_target = attitude_control.angle_ef_targets();
        if (pythagorous2(angle_target.x, angle_target.y) > 1500.0f) {
            // if target lean angle is over 15 degrees set high
            motors.set_throttle_low_comp(0.9f);
        } else {
            motors.set_throttle_low_comp(0.1f);
        }
    }
}
