// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AC_AttitudeControl_Tiltrotor_Y6.h
/// @brief   ArduCopter attitude control library for a Tiltrotor_Y6 (copied from Trad Heli)

#ifndef AC_ATTITUDECONTROL_TILTROTOR_Y6_H
#define AC_ATTITUDECONTROL_TILTROTOR_Y6_H

#include <AC_AttitudeControl.h>
#include <AC_PID.h>
#include <Filter.h>
#include <AP_Airspeed.h>

#define AC_ATTITUDE_HELI_ROLL_FF                    0.0f
#define AC_ATTITUDE_HELI_PITCH_FF                   0.0f
#define AC_ATTITUDE_HELI_YAW_FF                     0.0f
#define AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE  0.02f
#define AC_ATTITUDE_HELI_RATE_FF_FILTER             5.0f

class AC_AttitudeControl_Tiltrotor_Y6 : public AC_AttitudeControl {
public:
    AC_AttitudeControl_Tiltrotor_Y6( AP_AHRS &ahrs,
                        const AP_Vehicle::MultiCopter &aparm,
                        AP_MotorsTiltrotor_Y6& motors,
                        AC_P& p_angle_roll, AC_P& p_angle_pitch, AC_P& p_angle_yaw,
                        AC_PID& pid_rate_roll, AC_PID& pid_rate_pitch, AC_PID& pid_rate_yaw,
                        AC_PID& pid_rate_pit_aero, AC_PID& pid_rate_roll_aero, AC_PID& pid_rate_yaw_mot,
                        AP_Airspeed& airspeed
                        ) :
        AC_AttitudeControl(ahrs, aparm, motors,
                           p_angle_roll, p_angle_pitch, p_angle_yaw,
                           pid_rate_roll, pid_rate_pitch, pid_rate_yaw),
        _passthrough_roll(0), _passthrough_pitch(0),
        _pid_rate_pit_aero(pid_rate_pit_aero),
        _pid_rate_roll_aero(pid_rate_roll_aero),
        _pid_rate_yaw_mot(pid_rate_yaw_mot),
        _airspeed(airspeed),
        _tvec_rads(M_PI_2)

		{
            AP_Param::setup_object_defaults(this, var_info);
		}

    // passthrough_bf_roll_pitch_rate_yaw - roll and pitch are passed through directly, body-frame rate target for yaw
    void passthrough_bf_roll_pitch_rate_yaw_m(float roll_passthrough, float pitch_passthrough, float yaw_rate_bf);

	// rate_controller_run - run lowest level body-frame rate controller and send outputs to the motors
	// should be called at 100hz or more
	virtual void rate_controller_run();

	// use_leaky_i - controls whether we use leaky i term for body-frame to motor output stage
	void use_leaky_i_m(bool leaky_i) {  _flags_heli.leaky_i = leaky_i; }

    // use_flybar_passthrough - controls whether we pass-through control inputs to swash-plate
	void use_flybar_passthrough_m(bool passthrough) {  _flags_heli.flybar_passthrough = passthrough; }

    void update_feedforward_filter_rates_m(float time_step);

    virtual void angle_ef_roll_pitch_rate_ef_yaw_smooth(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef, float smoothing_gain);
    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:

    // To-Do: move these limits flags into the heli motors class
    struct AttControlHeliFlags {
        uint8_t limit_roll          :   1;  // 1 if we have requested larger roll angle than swash can physically move
        uint8_t limit_pitch         :   1;  // 1 if we have requested larger pitch angle than swash can physically move
        uint8_t limit_yaw           :   1;  // 1 if we have requested larger yaw angle than tail servo can physically move
        uint8_t leaky_i             :   1;  // 1 if we should use leaky i term for body-frame rate to motor stage
        uint8_t flybar_passthrough  :   1;  // 1 if we should pass through pilots roll & pitch input directly to swash-plate
    } _flags_heli;

    //
    // body-frame rate controller
    //
	// rate_bf_to_motor_roll_pitch - ask the rate controller to calculate the motor outputs to achieve the target body-frame rate (in centi-degrees/sec) for roll, pitch and yaw
    // outputs are sent directly to motor class
    void rate_bf_to_motor_roll_pitch_m(float rate_roll_target_cds, float rate_pitch_target_cds);

    //tiltrotor PIDs
   float rate_bf_to_motor_roll_aero(float rate_target_cds);
   float rate_bf_to_motor_pitch_aero(float rate_target_cds);
   float rate_bf_to_motor_yaw_mot(float rate_target_cds);

// add roll and yaw pid functions for the thrust vector
   virtual float rate_bf_to_motor_roll(float rate_target_cds);
   virtual float rate_bf_to_motor_yaw(float rate_target_cds);

    // get_angle_boost - calculate total body frame throttle required to produce the given earth frame throttle
    virtual int16_t get_angle_boost(int16_t throttle_pwm);

    // LPF filters to act on Rate Feedforward terms to linearize output.
    // Due to complicated aerodynamic effects, feedforwards acting too fast can lead
    // to jerks on rate change requests.
    LowPassFilterFloat pitch_feedforward_filter;
    LowPassFilterFloat roll_feedforward_filter;
    LowPassFilterFloat yaw_feedforward_filter;

    ///additional tiltrotor specific PIDs
    AC_PID&             _pid_rate_pit_aero;
    AC_PID&             _pid_rate_roll_aero;
    AC_PID&             _pid_rate_yaw_mot;

    AP_Airspeed& _airspeed;

    // pass through for roll and pitch
    int16_t _passthrough_roll;
    int16_t _passthrough_pitch;

    float _tvec_rads;//tvec angle in radians, computed using info in aparms

    AP_Float _roll_TC;
    AP_Float _roll_P;
    AP_Float _roll_I;
    AP_Float _roll_D;
    AP_Int16 _roll_rmax;
    AP_Int16 _roll_imax;
    AP_Float _pitch_TC;
    AP_Float _pitch_P;
    AP_Float _pitch_I;
    AP_Float _pitch_D;
    AP_Int16 _pitch_rmax;
    AP_Int16 _pitch_imax;
};

#endif //AC_ATTITUDECONTROL_TILTROTOR_Y6_H
