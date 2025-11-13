#include "quadcopter_control.h"

void safety_monitor(MotorSpeeds& motor_speeds, StateVector& state,
                   bool& emergency_stop, SystemParams params) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    static int error_count = 0;
    const int MAX_ERROR_COUNT = 5; // Reduced for faster response

    // Convert angles to degrees for checking
    fp32_t roll_deg = state.roll * fp32_t(180.0 / 3.14159);
    fp32_t pitch_deg = state.pitch * fp32_t(180.0 / 3.14159);

    // Check for emergency stop conditions - MANUAL ABSOLUTE VALUE CHECKS
    bool roll_exceeded = (roll_deg > fp32_t(45.0)) || (roll_deg < fp32_t(-45.0));
    bool pitch_exceeded = (pitch_deg > fp32_t(45.0)) || (pitch_deg < fp32_t(-45.0));
    bool angle_exceeded = roll_exceeded || pitch_exceeded;

    bool altitude_exceeded = (state.pos_z > fp32_t(150.0)); // Max altitude 150m
    bool negative_altitude = (state.pos_z < fp32_t(-5.0)); // Minimum altitude

    // Motor failure detection
    bool motor_failure = (motor_speeds.front_left > control_signal_t(100) ||
                         motor_speeds.front_right > control_signal_t(100) ||
                         motor_speeds.rear_left > control_signal_t(100) ||
                         motor_speeds.rear_right > control_signal_t(100) ||
                         motor_speeds.front_left < control_signal_t(0) ||
                         motor_speeds.front_right < control_signal_t(0) ||
                         motor_speeds.rear_left < control_signal_t(0) ||
                         motor_speeds.rear_right < control_signal_t(0));

    // Motor speed inconsistency check
    control_signal_t avg_speed = (motor_speeds.front_left + motor_speeds.front_right +
                                 motor_speeds.rear_left + motor_speeds.rear_right) / control_signal_t(4.0);

    control_signal_t max_diff = control_signal_t(0);
    control_signal_t diff;

    // Manual absolute value calculations
    diff = (motor_speeds.front_left > avg_speed) ?
           (motor_speeds.front_left - avg_speed) :
           (avg_speed - motor_speeds.front_left);
    if (diff > max_diff) max_diff = diff;

    diff = (motor_speeds.front_right > avg_speed) ?
           (motor_speeds.front_right - avg_speed) :
           (avg_speed - motor_speeds.front_right);
    if (diff > max_diff) max_diff = diff;

    diff = (motor_speeds.rear_left > avg_speed) ?
           (motor_speeds.rear_left - avg_speed) :
           (avg_speed - motor_speeds.rear_left);
    if (diff > max_diff) max_diff = diff;

    diff = (motor_speeds.rear_right > avg_speed) ?
           (motor_speeds.rear_right - avg_speed) :
           (avg_speed - motor_speeds.rear_right);
    if (diff > max_diff) max_diff = diff;

    bool motor_imbalance = (max_diff > control_signal_t(40.0));

    // Emergency conditions - ANY of these should trigger emergency stop
    if (angle_exceeded || altitude_exceeded || negative_altitude ||
        motor_failure || motor_imbalance || emergency_stop) {
        error_count++;
    } else {
        // Reset error count if no issues
        if (error_count > 0) error_count--;
    }

    // Apply emergency stop if conditions are met
    if (error_count >= MAX_ERROR_COUNT || emergency_stop) {
        // FORCE ALL MOTORS TO ZERO - CRITICAL SAFETY
        motor_speeds.front_left = control_signal_t(0);
        motor_speeds.front_right = control_signal_t(0);
        motor_speeds.rear_left = control_signal_t(0);
        motor_speeds.rear_right = control_signal_t(0);
        emergency_stop = true;

        // Reset velocities for safety
        state.vel_x = fp32_t(0);
        state.vel_y = fp32_t(0);
        state.vel_z = fp32_t(0);
        state.ang_vel_x = fp32_t(0);
        state.ang_vel_y = fp32_t(0);
        state.ang_vel_z = fp32_t(0);
    }

    // Always ensure motor speeds are within valid bounds
    if (motor_speeds.front_left < control_signal_t(0)) motor_speeds.front_left = control_signal_t(0);
    if (motor_speeds.front_left > control_signal_t(100)) motor_speeds.front_left = control_signal_t(100);
    if (motor_speeds.front_right < control_signal_t(0)) motor_speeds.front_right = control_signal_t(0);
    if (motor_speeds.front_right > control_signal_t(100)) motor_speeds.front_right = control_signal_t(100);
    if (motor_speeds.rear_left < control_signal_t(0)) motor_speeds.rear_left = control_signal_t(0);
    if (motor_speeds.rear_left > control_signal_t(100)) motor_speeds.rear_left = control_signal_t(100);
    if (motor_speeds.rear_right < control_signal_t(0)) motor_speeds.rear_right = control_signal_t(0);
    if (motor_speeds.rear_right > control_signal_t(100)) motor_speeds.rear_right = control_signal_t(100);
}
