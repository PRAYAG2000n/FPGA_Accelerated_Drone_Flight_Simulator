#include "quadcopter_control.h"
#include "hls_math.h"

void altitude_controller(StateVector current_state, ControlCommands commands,
                        control_signal_t& throttle_output, SystemParams params) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    static control_signal_t alt_integral = control_signal_t(0);
    static control_signal_t prev_alt_error = control_signal_t(0);

    // PID Gains
    const control_signal_t kp_alt = control_signal_t(1.5);
    const control_signal_t ki_alt = control_signal_t(0.05);
    const control_signal_t kd_alt = control_signal_t(1.0);

    control_signal_t target_altitude = commands.throttle;
    control_signal_t current_altitude = control_signal_t(current_state.pos_z);

    // Error with deadband
    control_signal_t alt_error = target_altitude - current_altitude;
    control_signal_t alt_deadband = control_signal_t(0.1);

    if (alt_error < control_signal_t(0)) {
        if (-alt_error < alt_deadband) alt_error = 0;
    } else {
        if (alt_error < alt_deadband) alt_error = 0;
    }

    // PID
    control_signal_t alt_derivative = alt_error - prev_alt_error;

    control_signal_t new_integral = alt_integral + alt_error;
    const control_signal_t max_integral = control_signal_t(25.0);

    if (new_integral > max_integral) {
        alt_integral = max_integral;
    } else if (new_integral < -max_integral) {
        alt_integral = -max_integral;
    } else {
        alt_integral = new_integral;
    }

    throttle_output = (kp_alt * alt_error) + (ki_alt * alt_integral) + (kd_alt * alt_derivative);

    // Hover throttle
    control_signal_t hover_throttle = control_signal_t(48.0);
    throttle_output += hover_throttle;

    // Limits
    if (throttle_output < control_signal_t(15)) throttle_output = control_signal_t(15);
    if (throttle_output > control_signal_t(85)) throttle_output = control_signal_t(85);

    prev_alt_error = alt_error;
}
