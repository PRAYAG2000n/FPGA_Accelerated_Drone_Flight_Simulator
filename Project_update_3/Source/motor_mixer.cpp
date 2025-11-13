#include "quadcopter_control.h"

void motor_mixer(control_signal_t throttle, control_signal_t roll_ctrl,
                control_signal_t pitch_ctrl, control_signal_t yaw_ctrl,
                MotorSpeeds& motor_speeds) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    // Quadcopter X-configuration mixing
    motor_speeds.front_left  = throttle + pitch_ctrl + roll_ctrl + yaw_ctrl;
    motor_speeds.front_right = throttle + pitch_ctrl - roll_ctrl - yaw_ctrl;
    motor_speeds.rear_left   = throttle - pitch_ctrl + roll_ctrl - yaw_ctrl;
    motor_speeds.rear_right  = throttle - pitch_ctrl - roll_ctrl + yaw_ctrl;

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
