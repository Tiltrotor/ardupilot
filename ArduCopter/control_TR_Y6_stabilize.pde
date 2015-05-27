/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#if FRAME_CONFIG == TILTROTOR_Y6_FRAME
/*
 * control_TR_Y6_stabilize.pde - For a tiltrotor Y6 only- init and run calls for stabilize flight mode. Copy of control_stabilize.pde
 */

// stabilize_TR_Y6_init - initialize tiltrotor Y6 stabilize controller
static bool stabilize_TR_Y6_init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    // To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
    pos_control.set_alt_target(0);

    // stabilize should never be made to fail
    return true;
}

// stabilize_TR_Y6_run - runs the main tiltrotor stabilize controller
// should be called at 100hz or more

static void stabilize_TR_Y6_run()
{

    float target_roll, target_pitch;
    float target_yaw_rate, req_tvec_angle;
    int16_t pilot_throttle_scaled;

////TREVOR ADDED "set_throttle_out_unstabilized()" 3.3 REBASE Throttle changes- NEEDS REVIEW!!
    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || g.rc_3.control_in <= 0) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();

    //attitude_control.set_throttle_out(0, false) TREVOR REMOVED 3.3 REBASE Throttle Changes-NEEDS REVIEW!!

        //we're not armed, so pass through tvec with no limits on angle
        req_tvec_angle = tvec_pwm_to_angle();
        aparm.tvec_angle_deg = req_tvec_angle;
        aparm.tvec_angle_pwm = tvec_angle_to_pwm(req_tvec_angle);
        aparm.gear_pos_pwm = g.rc_7.radio_in;
        return;

        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);


    //read in requested tvec angle, then determine and set the final angle

    req_tvec_angle = tvec_pwm_to_angle(); //see what tvec angle is being requested

    set_tvec_angle(req_tvec_angle);//limits the acutal tvec angle

    //read in the gear position from the user's rc_7 in
    aparm.gear_pos_pwm = g.rc_7.radio_in;//ch 7 in is currently our gear up/down switch

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}

//convert user's tvec pwm into the equiv. angle
static float tvec_pwm_to_angle()
{

float angle_deg = 0.09f*float(g.rc_8.radio_in) - 90.0f;

angle_deg = constrain_float(angle_deg,0.0f,90.0f);//for now, limit request to 0-90 deg of tvec

return angle_deg;//send back the requested tvec angle in degrees
}


static void set_tvec_angle(float req_angle)
{

if(!motors.armed())
{
aparm.tvec_angle_deg = req_angle;
aparm.tvec_angle_pwm = tvec_angle_to_pwm(req_angle);
return;
}


if(airspeed.get_airspeed() < 5.0f);
{

req_angle = constrain_float(req_angle,0.0f,90.0f);

#endif  //TILTROTOR_Y6_FRAME
