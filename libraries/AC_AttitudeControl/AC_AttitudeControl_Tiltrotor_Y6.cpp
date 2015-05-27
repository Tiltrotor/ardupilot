// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl_Tiltrotor_Y6.h"
#include <AP_HAL.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Tiltrotor_Y6::var_info[] PROGMEM = {
//TREVOR REMOVED FOR 3.3
    // @Param: RATE_RP_MAX
    // @DisplayName: Angle Rate Roll-Pitch max
    // @Description: maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
    // @Units: Centi-Degrees/Sec
    // @Range: 9000 36000
    // @Increment: 500
    // @User: Advanced
    //AP_GROUPINFO("RATE_RP_MAX", 0, AC_AttitudeControl_Tiltrotor_Y6, _angle_rate_rp_max, AC_ATTITUDE_CONTROL_RATE_RP_MAX_DEFAULT),

    // @Param: RATE_Y_MAX
    // @DisplayName: Angle Rate Yaw max
    // @Description: maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
    // @Units: Centi-Degrees/Sec
    // @Range: 4500 18000
    // @Increment: 500
    // @User: Advanced
    //AP_GROUPINFO("RATE_Y_MAX",  1, AC_AttitudeControl_Tiltrotor_Y6, _angle_rate_y_max, AC_ATTITUDE_CONTROL_RATE_Y_MAX_DEFAULT),

    // @Param: SLEW_YAW
    // @DisplayName: Yaw target slew rate
    // @Description: Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
    // @Units: Centi-Degrees/Sec
    // @Range: 500 18000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("SLEW_YAW",    2, AC_AttitudeControl_Tiltrotor_Y6, _slew_yaw, AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT),

//TREVOR REMOVED FOR 3.3
    // @Param: ACCEL_RP_MAX
    // @DisplayName: Acceleration Max for Roll/Pitch
    // @Description: Maximum acceleration in roll/pitch axis
    // @Units: Centi-Degrees/Sec/Sec
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    //AP_GROUPINFO("ACCEL_RP_MAX", 3, AC_AttitudeControl_Tiltrotor_Y6, _accel_rp_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT),



    // @Param: ACCEL_Y_MAX
    // @DisplayName: Acceleration Max for Yaw
    // @Description: Maximum acceleration in yaw axis
    // @Units: Centi-Degrees/Sec/Sec
    // @Range: 0 72000
    // @Values: 0:Disabled, 18000:Slow, 36000:Medium, 54000:Fast
    // @Increment: 1000
    // @User: Advanced
    AP_GROUPINFO("ACCEL_Y_MAX",  4, AC_AttitudeControl_Tiltrotor_Y6, _accel_yaw_max, AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT),

    // @Param: RATE_FF_ENAB
    // @DisplayName: Rate Feedforward Enable
    // @Description: Controls whether body-frame rate feedfoward is enabled or disabled
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("RATE_FF_ENAB", 5, AC_AttitudeControl_Tiltrotor_Y6, _rate_bf_ff_enabled, AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT),

    // @Param: TCONST
    // @DisplayName: Roll Time Constant
    // @Description: This controls the time constant in seconds from demanded to achieved bank angle. A value of 0.5 is a good default and will work with nearly all models. Advanced users may want to reduce this time to obtain a faster response but there is no point setting a time less than the aircraft can achieve.
    // @Range: 0.4 1.0
    // @Units: seconds
    // @Increment: 0.1
    // @User: Advanced
    // @Param: ACCEL_R_MAX

    // @DisplayName: Acceleration Max for Roll
    // @Description: Maximum acceleration in roll axis
    // @Units: Centi-Degrees/Sec/Sec
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("ACCEL_R_MAX", 6, AC_AttitudeControl_Tiltrotor_Y6, _accel_roll_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT),
    // @Param: ACCEL_P_MAX
    // @DisplayName: Acceleration Max for Pitch
    // @Description: Maximum acceleration in pitch axis
    // @Units: Centi-Degrees/Sec/Sec
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("ACCEL_P_MAX", 7, AC_AttitudeControl_Tiltrotor_Y6, _accel_pitch_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT),


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///Aerodynamic Roll PID

    AP_GROUPINFO("RLL_TAU",      8, AC_AttitudeControl_Tiltrotor_Y6, _roll_TC,       0.5f),

      // @Param: P
      // @DisplayName: Proportional Gain
      // @Description: This is the gain from bank angle to aileron. This gain works the same way as the P term in the old PID (RLL2SRV_P) and can be set to the same value.
      // @Range: 0.1 2.0
      // @Increment: 0.1
      // @User: User
      AP_GROUPINFO("RLL_P",        9, AC_AttitudeControl_Tiltrotor_Y6, _roll_P,        0.4f),

      // @Param: D
      // @DisplayName: Damping Gain
      // @Description: This is the gain from roll rate to aileron. This adjusts the damping of the roll control loop. It has the same effect as RLL2SRV_D in the old PID controller but without the spikes in servo demands. This gain helps to reduce rolling in turbulence. It should be increased in 0.01 increments as too high a value can lead to a high frequency roll oscillation that could overstress the airframe.
      // @Range: 0 0.1
      // @Increment: 0.01
      // @User: User
      AP_GROUPINFO("RLL_D",        10, AC_AttitudeControl_Tiltrotor_Y6, _roll_D,        0.02f),

      // @Param: I
      // @DisplayName: Integrator Gain
      // @Description: This is the gain from the integral of bank angle to aileron. It has the same effect as RLL2SRV_I in the old PID controller. Increasing this gain causes the controller to trim out steady offsets due to an out of trim aircraft.
      // @Range: 0 1.0
      // @Increment: 0.05
      // @User: User
      AP_GROUPINFO("RLL_I",        11, AC_AttitudeControl_Tiltrotor_Y6, _roll_I,        0.0f),

      // @Param: RMAX
      // @DisplayName: Maximum Roll Rate
      // @Description: This sets the maximum roll rate that the controller will demand (degrees/sec). Setting it to zero disables the limit. If this value is set too low, then the roll can't keep up with the navigation demands and the plane will start weaving. If it is set too high (or disabled by setting to zero) then ailerons will get large inputs at the start of turns. A limit of 60 degrees/sec is a good default.
      // @Range: 0 180
      // @Units: degrees/second
      // @Increment: 1
      // @User: Advanced
      AP_GROUPINFO("RLL_RMAX",   12, AC_AttitudeControl_Tiltrotor_Y6, _roll_rmax,       0),

      // @Param: IMAX
      // @DisplayName: Integrator limit
      // @Description: This limits the number of degrees of aileron in centi-degrees over which the integrator will operate. At the default setting of 1500 centi-degrees, the integrator will be limited to +- 15 degrees of servo travel. The maximum servo deflection is +- 45 centi-degrees, so the default value represents a 1/3rd of the total control throw which is adequate unless the aircraft is severely out of trim.
      // @Range: 0 4500
      // @Increment: 1
      // @User: Advanced
      AP_GROUPINFO("RLL_IMAX",      13, AC_AttitudeControl_Tiltrotor_Y6, _roll_imax,        1500),

///Aerodymanic PITCH AXIS PID
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// @Param: TCONST
// @DisplayName: Roll Time Constant
// @Description: This controls the time constant in seconds from demanded to achieved bank angle. A value of 0.5 is a good default and will work with nearly all models. Advanced users may want to reduce this time to obtain a faster response but there is no point setting a time less than the aircraft can achieve.
// @Range: 0.4 1.0
// @Units: seconds
// @Increment: 0.1
// @User: Advanced
AP_GROUPINFO("PIT_TAU",      14, AC_AttitudeControl_Tiltrotor_Y6, _pitch_TC,       0.5f),

  // @Param: P
  // @DisplayName: Proportional Gain
  // @Description: This is the gain from bank angle to aileron. This gain works the same way as the P term in the old PID (RLL2SRV_P) and can be set to the same value.
  // @Range: 0.1 2.0
  // @Increment: 0.1
  // @User: User
  AP_GROUPINFO("PIT_P",        15, AC_AttitudeControl_Tiltrotor_Y6, _pitch_P,        0.4f),

  // @Param: D
  // @DisplayName: Damping Gain
  // @Description: This is the gain from roll rate to aileron. This adjusts the damping of the roll control loop. It has the same effect as RLL2SRV_D in the old PID controller but without the spikes in servo demands. This gain helps to reduce rolling in turbulence. It should be increased in 0.01 increments as too high a value can lead to a high frequency roll oscillation that could overstress the airframe.
  // @Range: 0 0.1
  // @Increment: 0.01
  // @User: User
  AP_GROUPINFO("PIT_D",        16, AC_AttitudeControl_Tiltrotor_Y6, _pitch_D,        0.02f),

  // @Param: I
  // @DisplayName: Integrator Gain
  // @Description: This is the gain from the integral of bank angle to aileron. It has the same effect as RLL2SRV_I in the old PID controller. Increasing this gain causes the controller to trim out steady offsets due to an out of trim aircraft.
  // @Range: 0 1.0
  // @Increment: 0.05
  // @User: User
  AP_GROUPINFO("PIT_I",        17, AC_AttitudeControl_Tiltrotor_Y6, _pitch_I,        0.0f),

  // @Param: RMAX
  // @DisplayName: Maximum Roll Rate
  // @Description: This sets the maximum roll rate that the controller will demand (degrees/sec). Setting it to zero disables the limit. If this value is set too low, then the roll can't keep up with the navigation demands and the plane will start weaving. If it is set too high (or disabled by setting to zero) then ailerons will get large inputs at the start of turns. A limit of 60 degrees/sec is a good default.
  // @Range: 0 180
  // @Units: degrees/second
  // @Increment: 1
  // @User: Advanced
  AP_GROUPINFO("PIT_RMAX",   18, AC_AttitudeControl_Tiltrotor_Y6, _pitch_rmax,       0),

  // @Param: IMAX
  // @DisplayName: Integrator limit
  // @Description: This limits the number of degrees of aileron in centi-degrees over which the integrator will operate. At the default setting of 1500 centi-degrees, the integrator will be limited to +- 15 degrees of servo travel. The maximum servo deflection is +- 45 centi-degrees, so the default value represents a 1/3rd of the total control throw which is adequate unless the aircraft is severely out of trim.
  // @Range: 0 4500
  // @Increment: 1
  // @User: Advanced
  AP_GROUPINFO("PIT_IMAX",      19, AC_AttitudeControl_Tiltrotor_Y6, _pitch_imax,        1500),
    AP_GROUPEND
};

// passthrough_bf_roll_pitch_rate_yaw - passthrough the pilots roll and pitch inputs directly to swashplate for flybar acro mode
void AC_AttitudeControl_Tiltrotor_Y6::passthrough_bf_roll_pitch_rate_yaw_m(float roll_passthrough, float pitch_passthrough, float yaw_rate_bf)
{

}
//
// rate controller (body-frame) methods
//

// rate_controller_run - run lowest level rate controller and send outputs to the motors
// should be called at 100hz or more

void AC_AttitudeControl_Tiltrotor_Y6::rate_controller_run()
{

	_tvec_rads = ToRad(_aparm.tvec_angle_deg);
	_tvec_rads = constrain_float(_tvec_rads,0.0f,M_PI_2);


// Disabled until development of 3.3- Needs review after rebase
	float Xp_target = 0.0f;
	float Zp_target = 0.0f;

//Future development- Inwork after 3.3 rebase review
    _motors.set_roll(rate_bf_to_motor_roll(0));  //
    _motors.set_pitch(rate_bf_to_motor_pitch(0)); //
    _motors.set_yaw(rate_bf_to_motor_yaw(0));//

//new functions specifically for a tiltrotor_Y6 frame
    ((AP_MotorsTiltrotor_Y6&)_motors).set_roll_aero(rate_bf_to_motor_roll_aero(_rate_bf_target.x));
    ((AP_MotorsTiltrotor_Y6&)_motors).set_pitch_aero(rate_bf_to_motor_pitch_aero(_rate_bf_target.y));
    ((AP_MotorsTiltrotor_Y6&)_motors).set_yaw_mot(rate_bf_to_motor_yaw_mot(_rate_bf_target.z));
    ((AP_MotorsTiltrotor_Y6&)_motors).set_tvec_pwm(_aparm.tvec_angle_pwm);
    ((AP_MotorsTiltrotor_Y6&)_motors).set_gear_pwm(_aparm.gear_pos_pwm);
}

//
// private methods
//

float AC_AttitudeControl_Tiltrotor_Y6::rate_bf_to_motor_roll(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?

    //mike: Future development- REVEIW!! after 3.3 REBASE

    current_rate =  0.0f;

    // calculate error and call pid controller
    rate_error = rate_target_cds - current_rate;
    _pid_rate_roll.set_input_filter_d(rate_error); // TREVOR ADDED entire line for 3.3

    //get p value
    p = _pid_rate_roll.get_p(); // TREVOR removed (rate_error) for 3.3 REBASE NEEDS REVIEW!!

    // get i term
    i = _pid_rate_roll.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = _pid_rate_roll.get_i();  //TREVOR removed  (rate_error, _dt) for 3.3
    }

    // get d term
    d = _pid_rate_roll.get_d(); //TREVOR removed (rate_error, _dt) for 3.3

    // constrain output and return
    return constrain_float((p+i+d), -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);

    // To-Do: allow logging of PIDs?
}

// rate_bf_to_motor_yaw - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float AC_AttitudeControl_Tiltrotor_Y6::rate_bf_to_motor_yaw(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?

    current_rate = (_ahrs.get_gyro().x * AC_ATTITUDE_CONTROL_DEGX100);

    // calculate error and call pid controller
    rate_error  = rate_target_cds - current_rate;
    _pid_rate_yaw.set_input_filter_all(rate_error); //TREVOR ADDED entire line for 3.3 REBASE - NEEDS REVIEW


    p = _pid_rate_yaw.get_p(); //(rate_error) removed by TREVOR for 3.3

    // separately calculate p, i, d values for logging
    //p = _pid_rate_yaw.get_p(); // (rate_error) removed by TREVOR for 3.3 // Entire line removed per 3.3??

    // get i term
    i = _pid_rate_yaw.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_motors.limit.yaw || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = _pid_rate_yaw.get_i(); // (rate_error, _dt) removed by TREVOR for 3.3
    }

    // get d value
    d = _pid_rate_yaw.get_d(); //(rate_error, _dt) removed by TREVOR for 3.3

    // constrain output and return
    return constrain_float((p+i+d), -AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX);

    // To-Do: allow logging of PIDs?
}


//
// body-frame rate controller
//

float AC_AttitudeControl_Tiltrotor_Y6::rate_bf_to_motor_roll_aero(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

        // get current rate
    // To-Do: make getting gyro rates more efficient?


    current_rate = (_ahrs.get_gyro().x * AC_ATTITUDE_CONTROL_DEGX100);


    // calculate error and call pid controller
    rate_error = rate_target_cds - current_rate;
    _pid_rate_roll.set_input_filter_d(rate_error);// TREVOR ADDED entire line for 3.3 REBASE REVIEW

    p = _pid_rate_roll_aero.get_p();//(rate_error) removed by TREVOR for 3.3

    // get i term
    i = _pid_rate_roll_aero.get_integrator();         //get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = _pid_rate_roll_aero.get_i();// get_i(rate_error, _dt); //(rate_error, _dt) removed by TREVOR for 3.3
    }

    // get d term
    d = _pid_rate_roll_aero.get_d();// TREVOR removed for 3.3 get_d(rate_error, _dt);

    // constrain output and return
    return constrain_float((p+i+d), -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);

    // To-Do: allow logging of PIDs?
}

float AC_AttitudeControl_Tiltrotor_Y6::rate_bf_to_motor_pitch_aero(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

        // get current rate
    // To-Do: make getting gyro rates more efficient?

    current_rate = (_ahrs.get_gyro().y * AC_ATTITUDE_CONTROL_DEGX100);

    // calculate error and call pid controller
    rate_error = rate_target_cds - current_rate;
    _pid_rate_pitch.set_input_filter_d(rate_error); //TREVOR ADDED entire line for 3.3

    p = _pid_rate_pit_aero.get_p() ;// TREVOR REMOVED get_p(rate_error) for 3.3

    // get i term
    i = _pid_rate_pit_aero.get_integrator() ;//get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = _pid_rate_pit_aero.get_i(); //TREVOR REMOVED rate_error, _dt from () for 3.3
    }

    // get d term
    d = _pid_rate_pit_aero.get_d();//TREVOR REMOVED (rate_error, _dt) for 3.3

    // constrain output and return
    return constrain_float((p+i+d), -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);

        // To-Do: allow logging of PIDs?
}

float AC_AttitudeControl_Tiltrotor_Y6::rate_bf_to_motor_yaw_mot(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?


    current_rate = (_ahrs.get_gyro().z * AC_ATTITUDE_CONTROL_DEGX100);


    // calculate error and call pid controller
    rate_error = rate_target_cds - current_rate;
    _pid_rate_yaw.set_input_filter_all(rate_error); //TREVOR ADDED entire line for 3.3

    // TREOVR ADDED for 3.3 -send input to PID controller THEN REMOVED FOR COMPILE
    //_pid_rate_yaw.set_input_filter_all(); //TREVOR REMOVED rate_error for 3.3

    p = _pid_rate_yaw_mot.get_p()   ;//TREVOR removed get_p(rate_error) for 3.3
    //b = _pid_rate_roll_aero->get_p(rate_error); trever test scenario

    // get i term
    i = _pid_rate_yaw_mot.get_integrator()   ;//get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = _pid_rate_yaw_mot.get_i()  ;//TREVOR REMOVED get_i(rate_error, _dt);
    }

    // get d term
    d = _pid_rate_yaw_mot.get_d()   ;// TREVOR REMOVED get_d(rate_error, _dt);

    // constrain output and return
    return constrain_float((p+i+d), -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);

    // To-Do: allow logging of PIDs?
}

void AC_AttitudeControl_Tiltrotor_Y6::rate_bf_to_motor_roll_pitch_m(float rate_roll_target_cds, float rate_pitch_target_cds)
{

}

//
// throttle functions
//

int16_t AC_AttitudeControl_Tiltrotor_Y6::get_angle_boost(int16_t throttle_pwm)
{

	float temp = _ahrs.cos_pitch() * _ahrs.cos_roll();
	    int16_t throttle_out;

	    temp = constrain_float(temp, 0.5f, 1.0f);

	    // reduce throttle if we go inverted
	    temp = constrain_float(9000-max(labs(_ahrs.roll_sensor),labs(_ahrs.pitch_sensor)), 0, 3000) / (3000 * temp);

	    // apply scale and constrain throttle
	    // To-Do: move throttle_min and throttle_max into the AP_Vehicles class?
	    throttle_out = constrain_float((float)(throttle_pwm-_motors.throttle_min()) * temp + _motors.throttle_min(), _motors.throttle_min(), 1000);

	    // record angle boost for logging
	    _angle_boost = throttle_out - throttle_pwm;

	    return throttle_out;

}


void AC_AttitudeControl_Tiltrotor_Y6::update_feedforward_filter_rates_m(float time_step)
{

}


// angle_ef_roll_pitch_rate_ef_yaw_smooth - attempts to maintain a roll and pitch angle and yaw rate (all earth frame) while smoothing the attitude based on the feel parameter
//      smoothing_gain : a number from 1 to 50 with 1 being sluggish and 50 being very crisp
void AC_AttitudeControl_Tiltrotor_Y6::angle_ef_roll_pitch_rate_ef_yaw_smooth(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef, float smoothing_gain)
{
    Vector3f angle_ef_error;    // earth frame angle errors
    float rate_change_limit;

    // sanity check smoothing gain
    smoothing_gain = constrain_float(smoothing_gain,1.0f,50.0f);

    float linear_angle = _accel_yaw_max/(smoothing_gain*smoothing_gain); //Trevor Updated _accel_y_max to _accel_yaw_max For 3.3 REBASE
    rate_change_limit = _accel_yaw_max * _dt; //Trevor Updated _accel_y_max to _accel_yaw_max for 3.3 REBASE- NEEDS REVIEW
    float rate_ef_desired;
    float angle_to_target;

    if (_accel_roll_max > 0.0f) {

    	// calculate earth-frame feed forward roll rate using linear response when close to the target, sqrt response when we're further away
    	angle_to_target = roll_angle_ef - _angle_ef_target.x;
    	if (angle_to_target > linear_angle) {
    		rate_ef_desired = safe_sqrt(2.0f*_accel_roll_max*((float)fabs(angle_to_target)-(linear_angle/2.0f))); //TREVOR UPDATED acc roll max 3.3
    	} else if (angle_to_target < -linear_angle) {
    		rate_ef_desired = -safe_sqrt(2.0f*_accel_roll_max*((float)fabs(angle_to_target)-(linear_angle/2.0f))); //TREVOR UPDATED acc roll max 3.3
    	} else {
    		rate_ef_desired = smoothing_gain*angle_to_target;
    	}
    	_rate_ef_desired.x = constrain_float(rate_ef_desired, _rate_ef_desired.x-rate_change_limit, _rate_ef_desired.x+rate_change_limit);

    	// update earth-frame roll angle target using desired roll rate
        update_ef_roll_angle_and_error(_rate_ef_desired.x, angle_ef_error, AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX);

    	// calculate earth-frame feed forward pitch rate using linear response when close to the target, sqrt response when we're further away
    	angle_to_target = pitch_angle_ef - _angle_ef_target.y;
    	if (angle_to_target > linear_angle) {
    		rate_ef_desired = safe_sqrt(2.0f*_accel_pitch_max*((float)fabs(angle_to_target)-(linear_angle/2.0f))); //TREVOR UPDATED acc pitch max 3.3
    	} else if (angle_to_target < -linear_angle) {
    		rate_ef_desired = -safe_sqrt(2.0f*_accel_pitch_max*((float)fabs(angle_to_target)-(linear_angle/2.0f))); //TREVOR UPDATED acc pitch max 3.3
    	} else {
    		rate_ef_desired = smoothing_gain*angle_to_target;
    	}
    	_rate_ef_desired.y = constrain_float(rate_ef_desired, _rate_ef_desired.y-rate_change_limit, _rate_ef_desired.y+rate_change_limit);

    	// update earth-frame pitch angle target using desired pitch rate
        update_ef_pitch_angle_and_error(_rate_ef_desired.y, angle_ef_error, AC_ATTITUDE_RATE_STAB_PITCH_OVERSHOOT_ANGLE_MAX);
    } else {
        // target roll and pitch to desired input roll and pitch
    	_angle_ef_target.x = roll_angle_ef;
        angle_ef_error.x = wrap_180_cd_float(_angle_ef_target.x - _ahrs.roll_sensor);

    	_angle_ef_target.y = pitch_angle_ef;
        angle_ef_error.y = wrap_180_cd_float(_angle_ef_target.y - _ahrs.pitch_sensor);

        // set roll and pitch feed forward to zero
    	_rate_ef_desired.x = 0;
    	_rate_ef_desired.y = 0;
    }
    // constrain earth-frame angle targets
    _angle_ef_target.x = constrain_float(_angle_ef_target.x, -_aparm.angle_max, _aparm.angle_max);
    _angle_ef_target.y = constrain_float(_angle_ef_target.y, -_aparm.angle_max, _aparm.angle_max);

    if (_accel_yaw_max > 0.0f) {                       //TREVOR UPDATED _accel_y_max to _accel_yaw_max
    	// set earth-frame feed forward rate for yaw
        rate_change_limit = _accel_yaw_max * _dt; //TREVOR UPDATED _accel_y_max to _accel_yaw_max

        float rate_change = yaw_rate_ef - _rate_ef_desired.z;
        rate_change = constrain_float(rate_change, -rate_change_limit, rate_change_limit);
        _rate_ef_desired.z += rate_change;
        // calculate yaw target angle and angle error
        update_ef_yaw_angle_and_error(_rate_ef_desired.z, angle_ef_error, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX);
    } else {
        // set yaw feed forward to zero
    	_rate_ef_desired.z = yaw_rate_ef;
    	//TR_spd_gate_yaw_controller();
        // calculate yaw target angle and angle error
        update_ef_yaw_angle_and_error(_rate_ef_desired.z, angle_ef_error, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX);
    }

    // convert earth-frame angle errors to body-frame angle errors
    frame_conversion_ef_to_bf(angle_ef_error, _angle_bf_error);


    // convert body-frame angle errors to body-frame rate targets
    update_rate_bf_targets();

    // add body frame rate feed forward
    if (_rate_bf_ff_enabled) {
        // convert earth-frame feed forward rates to body-frame feed forward rates
        frame_conversion_ef_to_bf(_rate_ef_desired, _rate_bf_desired);
        _rate_bf_target += _rate_bf_desired;
    } else {
        // convert earth-frame feed forward rates to body-frame feed forward rates

    	if(_rate_ef_desired.z != 0)//do normal mixing of pilot input
    	{
    	frame_conversion_ef_to_bf(Vector3f(0,0,_rate_ef_desired.z), _rate_bf_desired);
        _rate_bf_target += _rate_bf_desired;
    	}
    	else
    	{


    			_rate_bf_target.z = (_ahrs.get_gyro().z * AC_ATTITUDE_CONTROL_DEGX100);


    	}

    }

    // body-frame to motor outputs should be called separately
}
