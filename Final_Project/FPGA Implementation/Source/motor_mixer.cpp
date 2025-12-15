#include "quadcopter_control.h"

void motor_mixer(control_signal_t throttle, control_signal_t roll_ctrl,
                control_signal_t pitch_ctrl, control_signal_t yaw_ctrl,
                MotorSpeeds& motor_speeds) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    // Scale control inputs
    control_signal_t scaled_roll = roll_ctrl * control_signal_t(0.2);
    control_signal_t scaled_pitch = pitch_ctrl * control_signal_t(0.2);
    control_signal_t scaled_yaw = yaw_ctrl * control_signal_t(0.15);

    // Motor mixing
    motor_speeds.front_left  = throttle + scaled_pitch + scaled_roll + scaled_yaw;
    motor_speeds.front_right = throttle + scaled_pitch - scaled_roll - scaled_yaw;
    motor_speeds.rear_left   = throttle - scaled_pitch + scaled_roll - scaled_yaw;
    motor_speeds.rear_right  = throttle - scaled_pitch - scaled_roll + scaled_yaw;

    // Minimum speed
    control_signal_t min_speed = control_signal_t(20.0);
    if (throttle > control_signal_t(10.0)) {
        if (motor_speeds.front_left < min_speed)  motor_speeds.front_left = min_speed;
        if (motor_speeds.front_right < min_speed) motor_speeds.front_right = min_speed;
        if (motor_speeds.rear_left < min_speed)   motor_speeds.rear_left = min_speed;
        if (motor_speeds.rear_right < min_speed)  motor_speeds.rear_right = min_speed;
    }

    // Find max motor
    control_signal_t max_motor = motor_speeds.front_left;
    if (motor_speeds.front_right > max_motor) max_motor = motor_speeds.front_right;
    if (motor_speeds.rear_left > max_motor) max_motor = motor_speeds.rear_left;
    if (motor_speeds.rear_right > max_motor) max_motor = motor_speeds.rear_right;

    // Scale if over limit
    const control_signal_t max_allowed = control_signal_t(95.0);
    if (max_motor > max_allowed) {
        control_signal_t scale = max_allowed / max_motor;
        motor_speeds.front_left  *= scale;
        motor_speeds.front_right *= scale;
        motor_speeds.rear_left   *= scale;
        motor_speeds.rear_right  *= scale;
    }

    // Final constraints
    if (motor_speeds.front_left < control_signal_t(0)) motor_speeds.front_left = control_signal_t(0);
    if (motor_speeds.front_left > control_signal_t(100)) motor_speeds.front_left = control_signal_t(100);
    if (motor_speeds.front_right < control_signal_t(0)) motor_speeds.front_right = control_signal_t(0);
    if (motor_speeds.front_right > control_signal_t(100)) motor_speeds.front_right = control_signal_t(100);
    if (motor_speeds.rear_left < control_signal_t(0)) motor_speeds.rear_left = control_signal_t(0);
    if (motor_speeds.rear_left > control_signal_t(100)) motor_speeds.rear_left = control_signal_t(100);
    if (motor_speeds.rear_right < control_signal_t(0)) motor_speeds.rear_right = control_signal_t(0);
    if (motor_speeds.rear_right > control_signal_t(100)) motor_speeds.rear_right = control_signal_t(100);
}
