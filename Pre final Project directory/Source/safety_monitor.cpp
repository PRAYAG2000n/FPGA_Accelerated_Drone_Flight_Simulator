#include "quadcopter_control.h"

void safety_monitor(MotorSpeeds& motor_speeds, StateVector& state,
                   bool& emergency_stop, SystemParams params) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    static ap_uint<4> error_count = 0;
    const ap_uint<4> MAX_ERROR_COUNT = 5;

    // Convert to degrees
    fp32_t roll_deg = state.roll * fp32_t(57.2958);
    fp32_t pitch_deg = state.pitch * fp32_t(57.2958);

    // Safety checks
    bool angle_exceeded = (roll_deg > fp32_t(45.0)) || (roll_deg < fp32_t(-45.0)) ||
                         (pitch_deg > fp32_t(45.0)) || (pitch_deg < fp32_t(-45.0));

    bool altitude_exceeded = (state.pos_z > fp32_t(150.0));
    bool negative_altitude = (state.pos_z < fp32_t(-5.0));

    bool motor_failure = (motor_speeds.front_left > control_signal_t(100)) ||
                        (motor_speeds.front_right > control_signal_t(100)) ||
                        (motor_speeds.rear_left > control_signal_t(100)) ||
                        (motor_speeds.rear_right > control_signal_t(100));

    // Motor imbalance
    control_signal_t max_motor = motor_speeds.front_left;
    control_signal_t min_motor = motor_speeds.front_left;

    if (motor_speeds.front_right > max_motor) max_motor = motor_speeds.front_right;
    if (motor_speeds.front_right < min_motor) min_motor = motor_speeds.front_right;
    if (motor_speeds.rear_left > max_motor) max_motor = motor_speeds.rear_left;
    if (motor_speeds.rear_left < min_motor) min_motor = motor_speeds.rear_left;
    if (motor_speeds.rear_right > max_motor) max_motor = motor_speeds.rear_right;
    if (motor_speeds.rear_right < min_motor) min_motor = motor_speeds.rear_right;

    bool motor_imbalance = ((max_motor - min_motor) > control_signal_t(40.0));

    bool any_emergency = angle_exceeded || altitude_exceeded || negative_altitude ||
                        motor_failure || motor_imbalance || emergency_stop;

    // Error counting
    if (any_emergency) {
        if (error_count < MAX_ERROR_COUNT) error_count++;
    } else {
        if (error_count > 0) error_count--;
    }

    // Emergency stop
    if (error_count >= MAX_ERROR_COUNT || emergency_stop) {
        motor_speeds.front_left = control_signal_t(0);
        motor_speeds.front_right = control_signal_t(0);
        motor_speeds.rear_left = control_signal_t(0);
        motor_speeds.rear_right = control_signal_t(0);
        emergency_stop = true;
    }

    // Final constraints
    if (motor_speeds.front_left > control_signal_t(100)) motor_speeds.front_left = control_signal_t(100);
    if (motor_speeds.front_right > control_signal_t(100)) motor_speeds.front_right = control_signal_t(100);
    if (motor_speeds.rear_left > control_signal_t(100)) motor_speeds.rear_left = control_signal_t(100);
    if (motor_speeds.rear_right > control_signal_t(100)) motor_speeds.rear_right = control_signal_t(100);
    if (motor_speeds.front_left < control_signal_t(0)) motor_speeds.front_left = control_signal_t(0);
    if (motor_speeds.front_right < control_signal_t(0)) motor_speeds.front_right = control_signal_t(0);
    if (motor_speeds.rear_left < control_signal_t(0)) motor_speeds.rear_left = control_signal_t(0);
    if (motor_speeds.rear_right < control_signal_t(0)) motor_speeds.rear_right = control_signal_t(0);
}
