#include "quadcopter_control.h"

void pid_controller(control_signal_t error, control_signal_t integral,
                   control_signal_t previous_error, control_signal_t& output,
                   control_signal_t kp, control_signal_t ki, control_signal_t kd) {
    #pragma HLS PIPELINE off
    #pragma HLS INLINE off

    // Calculate derivative term
    control_signal_t derivative = error - previous_error;

    // Calculate new integral with anti-windup
    control_signal_t new_integral = integral + error;
    const control_signal_t max_integral = control_signal_t(1.0);

    if (new_integral > max_integral) {
        new_integral = max_integral;
    } else if (new_integral < -max_integral) {
        new_integral = -max_integral;
    }

    // PID calculation
    control_signal_t p_term = kp * error;
    control_signal_t i_term = ki * new_integral;
    control_signal_t d_term = kd * derivative;

    output = p_term + i_term + d_term;

    // Output limiting
    const control_signal_t max_output = control_signal_t(1.0);

    if (output > max_output) {
        output = max_output;
    } else if (output < -max_output) {
        output = -max_output;
    }
}
