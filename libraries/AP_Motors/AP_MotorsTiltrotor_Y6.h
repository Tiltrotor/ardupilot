// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsSingle.h
/// @brief	Motor and Servo control class for Singlecopters

#ifndef __AP_MOTORS_TRY6_H__
#define __AP_MOTORS_TRY6_H__

#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include "AP_Motors.h"

// feedback direction
#define AP_MOTORS_SING_POSITIVE      1
#define AP_MOTORS_SING_NEGATIVE     -1

#define AP_MOTORS_SINGLE_SPEED_DIGITAL_SERVOS 250 // update rate for digital servos
#define AP_MOTORS_SINGLE_SPEED_ANALOG_SERVOS 125  // update rate for analog servos

#define AP_MOTORS_SINGLE_SERVO_INPUT_RANGE      4500    // roll or pitch input of -4500 will cause servos to their minimum (i.e. radio_min), +4500 will move them to their maximum (i.e. radio_max)

/// @class      AP_MotorsTiltrotor_Y6
class AP_MotorsTiltrotor_Y6 : public AP_Motors {
public:

    /// Constructor
    AP_MotorsTiltrotor_Y6( RC_Channel& rc_roll, RC_Channel& rc_pitch, RC_Channel& rc_throttle, RC_Channel& rc_yaw, RC_Channel& servo1, RC_Channel& servo2, RC_Channel& servo3, RC_Channel& servo4, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_Motors(rc_roll, rc_pitch, rc_throttle, rc_yaw, speed_hz),
        _servo1(servo1),
        _servo2(servo2),
        _servo3(servo3),
        _servo4(servo4)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // init
    virtual void        Init();

    // set update rate to motors - a value in hertz
    void                set_update_rate( uint16_t speed_hz );


///tiltrotor mike
//udpate: removed const and it still works.  wtf
    //i had to add the "const" to get it to work.  need to look this up
    void set_roll_aero(int16_t roll_aero_in)  { _servo1.servo_out = roll_aero_in; };
    void set_pitch_aero(int16_t pitch_aero_in)  { _servo2.servo_out = pitch_aero_in; };
    void set_yaw_mot(int16_t yaw_mot_in) { _servo3.servo_out = yaw_mot_in; };
    void set_tvec_pwm(int16_t tvec_pwm) { _servo4.radio_out = tvec_pwm; };
    void set_gear_pwm(int16_t gear_pwm) { _servo1.radio_in = gear_pwm; };




    // enable - starts allowing signals to be sent to motors
    virtual void        enable();

    // output_test - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void        output_test(uint8_t motor_seq, int16_t pwm);

    // output_min - sends minimum values out to the motors
    virtual void        output_min();

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    virtual uint16_t    get_motor_mask();

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // output - sends commands to the motors
    // TREVOR ADDED 3.3 Rebase (removed output_armed(); and output_disarmed();)
    void output_armed_stabilizing();
    void output_armed_not_stabilizing();
    void output_disarmed();




    AP_Int8             _rev_roll;      // REV Roll feedback
    AP_Int8             _rev_pitch;     // REV pitch feedback
    AP_Int8             _rev_yaw;       // REV yaw feedback
    AP_Int16            _servo_speed;   // servo speed
    RC_Channel&         _servo1;
    RC_Channel&         _servo2;
    RC_Channel&         _servo3;
    RC_Channel&         _servo4;
};

#endif  // AP_MOTORS TILTROTOR_Y6
