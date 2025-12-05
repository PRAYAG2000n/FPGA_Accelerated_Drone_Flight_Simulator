#include "quadcopter_control.h"

void safety_monitor(MotorSpeeds& motor_speeds, StateVector& state,
                   bool& emergency_stop, SystemParams params) {
    #pragma HLS PIPELINE off
    #pragma HLS INLINE off

    static ap_uint<4> error_count = 0;
    const ap_uint<4> MAX_ERROR_COUNT = 5;

    // Check angle limits (in radians)
    // 45 degrees = 0.785 radians
    const fp32_t max_angle = fp32_t(0.785);

    bool roll_exceeded = (state.roll > max_angle) || (state.roll < -max_angle);
    bool pitch_exceeded = (state.pitch > max_angle) || (state.pitch < -max_angle);
    bool angle_exceeded = roll_exceeded || pitch_exceeded;

    // Check altitude limits
    bool altitude_exceeded = state.pos_z > fp32_t(150.0);
    bool negative_altitude = state.pos_z < fp32_t(-1.0);

    // Combine emergency conditions
    bool any_emergency = angle_exceeded || altitude_exceeded || negative_altitude || emergency_stop;

    // Update error counter
    if (any_emergency) {
        if (error_count < MAX_ERROR_COUNT) {
            error_count = error_count + 1;
        }
    } else {
        if (error_count > 0) {
            error_count = error_count - 1;
        }
    }

    // Apply emergency stop after sustained errors
    if (error_count >= MAX_ERROR_COUNT) {
        motor_speeds.front_left = control_signal_t(0);
        motor_speeds.front_right = control_signal_t(0);
        motor_speeds.rear_left = control_signal_t(0);
        motor_speeds.rear_right = control_signal_t(0);
        emergency_stop = true;
    }

    // Constrain motor outputs to valid range [0, 1]
    if (motor_speeds.front_left < control_signal_t(0)) {
        motor_speeds.front_left = control_signal_t(0);
    }
    if (motor_speeds.front_left > control_signal_t(1)) {
        motor_speeds.front_left = control_signal_t(1);
    }

    if (motor_speeds.front_right < control_signal_t(0)) {
        motor_speeds.front_right = control_signal_t(0);
    }
    if (motor_speeds.front_right > control_signal_t(1)) {
        motor_speeds.front_right = control_signal_t(1);
    }

    if (motor_speeds.rear_left < control_signal_t(0)) {
        motor_speeds.rear_left = control_signal_t(0);
    }
    if (motor_speeds.rear_left > control_signal_t(1)) {
        motor_speeds.rear_left = control_signal_t(1);
    }

    if (motor_speeds.rear_right < control_signal_t(0)) {
        motor_speeds.rear_right = control_signal_t(0);
    }
    if (motor_speeds.rear_right > control_signal_t(1)) {
        motor_speeds.rear_right = control_signal_t(1);
    }
}
