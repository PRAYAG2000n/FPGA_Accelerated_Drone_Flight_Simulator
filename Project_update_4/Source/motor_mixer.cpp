#include "quadcopter_control.h"

void motor_mixer(control_signal_t throttle, control_signal_t roll_ctrl,
                control_signal_t pitch_ctrl, control_signal_t yaw_ctrl,
                MotorSpeeds& motor_speeds) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    // Quadcopter X-configuration mixing with proper scaling
    // Scale controls to prevent saturation
    control_signal_t scaled_roll = roll_ctrl * control_signal_t(0.3);
    control_signal_t scaled_pitch = pitch_ctrl * control_signal_t(0.3);
    control_signal_t scaled_yaw = yaw_ctrl * control_signal_t(0.2);

    // Mix motor outputs - ensure all motors get base throttle
    motor_speeds.front_left  = throttle + scaled_pitch + scaled_roll + scaled_yaw;
    motor_speeds.front_right = throttle + scaled_pitch - scaled_roll - scaled_yaw;
    motor_speeds.rear_left   = throttle - scaled_pitch + scaled_roll - scaled_yaw;
    motor_speeds.rear_right  = throttle - scaled_pitch - scaled_roll + scaled_yaw;

    // Ensure minimum motor speed for stability (except in emergency stop)
    control_signal_t min_speed = control_signal_t(15.0);
    if (motor_speeds.front_left < min_speed && throttle > control_signal_t(5.0))
        motor_speeds.front_left = min_speed;
    if (motor_speeds.front_right < min_speed && throttle > control_signal_t(5.0))
        motor_speeds.front_right = min_speed;
    if (motor_speeds.rear_left < min_speed && throttle > control_signal_t(5.0))
        motor_speeds.rear_left = min_speed;
    if (motor_speeds.rear_right < min_speed && throttle > control_signal_t(5.0))
        motor_speeds.rear_right = min_speed;

    // Constrain motor speeds to valid range [0, 100]
    if (motor_speeds.front_left < control_signal_t(0)) motor_speeds.front_left = control_signal_t(0);
    if (motor_speeds.front_left > control_signal_t(100)) motor_speeds.front_left = control_signal_t(100);
    if (motor_speeds.front_right < control_signal_t(0)) motor_speeds.front_right = control_signal_t(0);
    if (motor_speeds.front_right > control_signal_t(100)) motor_speeds.front_right = control_signal_t(100);
    if (motor_speeds.rear_left < control_signal_t(0)) motor_speeds.rear_left = control_signal_t(0);
    if (motor_speeds.rear_left > control_signal_t(100)) motor_speeds.rear_left = control_signal_t(100);
    if (motor_speeds.rear_right < control_signal_t(0)) motor_speeds.rear_right = control_signal_t(0);
    if (motor_speeds.rear_right > control_signal_t(100)) motor_speeds.rear_right = control_signal_t(100);
}
