#include "quadcopter_control.h"

void pid_controller(control_signal_t error, control_signal_t integral,
                   control_signal_t previous_error, control_signal_t& output,
                   control_signal_t kp, control_signal_t ki, control_signal_t kd) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    control_signal_t derivative = error - previous_error;

    // Anti-windup
    control_signal_t new_integral = integral + error;
    const control_signal_t max_integral = control_signal_t(50.0);

    if (new_integral > max_integral) {
        new_integral = max_integral;
    } else if (new_integral < -max_integral) {
        new_integral = -max_integral;
    }

    // PID calculation
    control_signal_t proportional_term = kp * error;
    control_signal_t integral_term = ki * new_integral;
    control_signal_t derivative_term = kd * derivative;

    output = proportional_term + integral_term + derivative_term;

    // Output limiting
    const control_signal_t max_output = control_signal_t(80.0);
    const control_signal_t min_output = control_signal_t(-80.0);

    if (output > max_output) {
        output = max_output;
    } else if (output < min_output) {
        output = min_output;
    }
}
