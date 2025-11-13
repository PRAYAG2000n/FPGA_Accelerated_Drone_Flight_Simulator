#include "quadcopter_control.h"

void pid_controller(control_signal_t error, control_signal_t integral,
                   control_signal_t previous_error, control_signal_t& output,
                   control_signal_t kp, control_signal_t ki, control_signal_t kd) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    control_signal_t derivative = error - previous_error;

    // PID calculation with anti-windup
    control_signal_t new_integral = integral + error;

    // Simple anti-windup: limit integral term
    if (new_integral > control_signal_t(100.0)) {
        new_integral = control_signal_t(100.0);
    } else if (new_integral < control_signal_t(-100.0)) {
        new_integral = control_signal_t(-100.0);
    }

    output = (kp * error) + (ki * new_integral) + (kd * derivative);

    // Limit output to reasonable range
    if (output > control_signal_t(100.0)) {
        output = control_signal_t(100.0);
    } else if (output < control_signal_t(-100.0)) {
        output = control_signal_t(-100.0);
    }
}
