#include "quadcopter_control.h"

void motor_mixer(control_signal_t throttle, control_signal_t roll_ctrl,
                control_signal_t pitch_ctrl, control_signal_t yaw_ctrl,
                MotorSpeeds& motor_speeds) {
    #pragma HLS PIPELINE off
    #pragma HLS INLINE off

    control_signal_t scaled_roll = roll_ctrl * control_signal_t(0.25);
    control_signal_t scaled_pitch = pitch_ctrl * control_signal_t(0.25);
    control_signal_t scaled_yaw = yaw_ctrl * control_signal_t(0.15);

    motor_speeds.front_left  = throttle - scaled_pitch + scaled_roll - scaled_yaw;
    motor_speeds.front_right = throttle - scaled_pitch - scaled_roll + scaled_yaw;
    motor_speeds.rear_left   = throttle + scaled_pitch + scaled_roll + scaled_yaw;
    motor_speeds.rear_right  = throttle + scaled_pitch - scaled_roll - scaled_yaw;

    if (motor_speeds.front_left < control_signal_t(0)) motor_speeds.front_left = control_signal_t(0);
    if (motor_speeds.front_left > control_signal_t(1)) motor_speeds.front_left = control_signal_t(1);
    if (motor_speeds.front_right < control_signal_t(0)) motor_speeds.front_right = control_signal_t(0);
    if (motor_speeds.front_right > control_signal_t(1)) motor_speeds.front_right = control_signal_t(1);
    if (motor_speeds.rear_left < control_signal_t(0)) motor_speeds.rear_left = control_signal_t(0);
    if (motor_speeds.rear_left > control_signal_t(1)) motor_speeds.rear_left = control_signal_t(1);
    if (motor_speeds.rear_right < control_signal_t(0)) motor_speeds.rear_right = control_signal_t(0);
    if (motor_speeds.rear_right > control_signal_t(1)) motor_speeds.rear_right = control_signal_t(1);
}
