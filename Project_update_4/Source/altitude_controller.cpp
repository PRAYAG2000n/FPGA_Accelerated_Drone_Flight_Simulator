#include "quadcopter_control.h"

void altitude_controller(StateVector current_state, ControlCommands commands,
                        control_signal_t& throttle_output, SystemParams params) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    static control_signal_t alt_integral = control_signal_t(0);
    static control_signal_t prev_alt_error = control_signal_t(0);

    // PID gains for altitude control
    const control_signal_t kp_alt = control_signal_t(2.0);
    const control_signal_t ki_alt = control_signal_t(0.1);
    const control_signal_t kd_alt = control_signal_t(0.8);

    // Target altitude from throttle command (map 0-100 throttle to 0-100m altitude)
    control_signal_t target_altitude = commands.throttle;

    // Current altitude
    control_signal_t current_altitude = control_signal_t(current_state.pos_z);

    // Altitude error
    control_signal_t alt_error = target_altitude - current_altitude;

    // PID controller for altitude
    control_signal_t alt_derivative = alt_error - prev_alt_error;

    // Update integral with anti-windup
    control_signal_t new_integral = alt_integral + alt_error;
    if (new_integral > control_signal_t(50.0)) new_integral = control_signal_t(50.0);
    if (new_integral < control_signal_t(-50.0)) new_integral = control_signal_t(-50.0);

    // PID output
    throttle_output = (kp_alt * alt_error) + (ki_alt * new_integral) + (kd_alt * alt_derivative);

    // Base throttle for hover (counteract gravity)
    control_signal_t hover_throttle = control_signal_t(45.0); // Adjust based on drone parameters

    // Combine with base hover throttle
    throttle_output += hover_throttle;

    // Limit throttle output
    if (throttle_output < control_signal_t(0)) throttle_output = control_signal_t(0);
    if (throttle_output > control_signal_t(100)) throttle_output = control_signal_t(100);

    // Update stored values
    alt_integral = new_integral;
    prev_alt_error = alt_error;
}
