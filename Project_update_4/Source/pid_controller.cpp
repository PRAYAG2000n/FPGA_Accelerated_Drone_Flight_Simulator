#include "quadcopter_control.h"

void pid_controller(control_signal_t error, control_signal_t integral,
                   control_signal_t previous_error, control_signal_t& output,
                   control_signal_t kp, control_signal_t ki, control_signal_t kd) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    // Calculate derivative term
    control_signal_t derivative = error - previous_error;

    // Update integral with anti-windup
    control_signal_t new_integral = integral + error;

    // Anti-windup limits
    const control_signal_t max_integral = control_signal_t(50.0);
    const control_signal_t min_integral = control_signal_t(-50.0);

    if (new_integral > max_integral) {
        new_integral = max_integral;
    } else if (new_integral < min_integral) {
        new_integral = min_integral;
    }

    // Calculate PID output
    control_signal_t p_term = kp * error;
    control_signal_t i_term = ki * new_integral;
    control_signal_t d_term = kd * derivative;

    output = p_term + i_term + d_term;

    // Output limits
    const control_signal_t max_output = control_signal_t(80.0);
    const control_signal_t min_output = control_signal_t(-80.0);

    if (output > max_output) {
        output = max_output;
    } else if (output < min_output) {
        output = min_output;
    }

    // Simple dead zone without complex logic
    const control_signal_t dead_zone = control_signal_t(0.5);
    if (output > -dead_zone && output < dead_zone) {
        output = control_signal_t(0);
    }
}
