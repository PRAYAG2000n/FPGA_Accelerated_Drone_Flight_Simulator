#include "quadcopter_control.h"

void safety_monitor(MotorSpeeds& motor_speeds, StateVector& state,
                   bool& emergency_stop, SystemParams params) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    static int error_count = 0;
    const int MAX_ERROR_COUNT = 10;

    // Check for emergency stop conditions
    bool angle_exceeded = (state.roll > 0.785 || state.roll < -0.785 ||  // > 45 degrees
                          state.pitch > 0.785 || state.pitch < -0.785);

    bool altitude_exceeded = (state.pos_z > 100.0);  // Maximum altitude limit

    bool motor_failure = (motor_speeds.front_left > 100 || motor_speeds.front_right > 100 ||
                         motor_speeds.rear_left > 100 || motor_speeds.rear_right > 100 ||
                         motor_speeds.front_left < 0 || motor_speeds.front_right < 0 ||
                         motor_speeds.rear_left < 0 || motor_speeds.rear_right < 0);

    // Check for motor speed inconsistencies
    control_signal_t avg_speed = (motor_speeds.front_left + motor_speeds.front_right +
                                 motor_speeds.rear_left + motor_speeds.rear_right) / 4;

    control_signal_t max_diff = 0;
    control_signal_t diff;

    // Manual absolute value calculations using if statements
    if (motor_speeds.front_left > avg_speed) {
        diff = motor_speeds.front_left - avg_speed;
    } else {
        diff = avg_speed - motor_speeds.front_left;
    }
    if (diff > max_diff) max_diff = diff;

    if (motor_speeds.front_right > avg_speed) {
        diff = motor_speeds.front_right - avg_speed;
    } else {
        diff = avg_speed - motor_speeds.front_right;
    }
    if (diff > max_diff) max_diff = diff;

    if (motor_speeds.rear_left > avg_speed) {
        diff = motor_speeds.rear_left - avg_speed;
    } else {
        diff = avg_speed - motor_speeds.rear_left;
    }
    if (diff > max_diff) max_diff = diff;

    if (motor_speeds.rear_right > avg_speed) {
        diff = motor_speeds.rear_right - avg_speed;
    } else {
        diff = avg_speed - motor_speeds.rear_right;
    }
    if (diff > max_diff) max_diff = diff;

    bool motor_imbalance = (max_diff > control_signal_t(30.0));

    // Emergency conditions
    if (angle_exceeded || altitude_exceeded || motor_failure || motor_imbalance || emergency_stop) {
        error_count++;

        if (error_count >= MAX_ERROR_COUNT || emergency_stop) {
            // Emergency stop - shut down all motors
            motor_speeds.front_left = control_signal_t(0);
            motor_speeds.front_right = control_signal_t(0);
            motor_speeds.rear_left = control_signal_t(0);
            motor_speeds.rear_right = control_signal_t(0);

            // Reset state for safety
            state.roll = 0;
            state.pitch = 0;
            state.vel_x = 0;
            state.vel_y = 0;
            state.vel_z = 0;
        }
    } else {
        // Reset error count if no issues
        if (error_count > 0) error_count--;
    }

    // Additional safety: Ensure motor speeds are within bounds
    if (motor_speeds.front_left < control_signal_t(0)) motor_speeds.front_left = control_signal_t(0);
    if (motor_speeds.front_left > control_signal_t(100)) motor_speeds.front_left = control_signal_t(100);

    if (motor_speeds.front_right < control_signal_t(0)) motor_speeds.front_right = control_signal_t(0);
    if (motor_speeds.front_right > control_signal_t(100)) motor_speeds.front_right = control_signal_t(100);

    if (motor_speeds.rear_left < control_signal_t(0)) motor_speeds.rear_left = control_signal_t(0);
    if (motor_speeds.rear_left > control_signal_t(100)) motor_speeds.rear_left = control_signal_t(100);

    if (motor_speeds.rear_right < control_signal_t(0)) motor_speeds.rear_right = control_signal_t(0);
    if (motor_speeds.rear_right > control_signal_t(100)) motor_speeds.rear_right = control_signal_t(100);
}
