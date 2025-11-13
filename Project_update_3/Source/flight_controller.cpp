#include "quadcopter_control.h"

void flight_controller(StateVector current_state, ControlCommands commands,
                      MotorSpeeds& motor_speeds, SystemParams params) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    static bool emergency_stop = false;
    control_signal_t roll_ctrl, pitch_ctrl, yaw_ctrl, throttle_ctrl;

    // Use the dedicated attitude controller for roll, pitch, yaw control
    attitude_controller(current_state, commands, roll_ctrl, pitch_ctrl, yaw_ctrl, throttle_ctrl);

    // Handle altitude control separately (since it's different from attitude control)
    static control_signal_t alt_integral = 0, prev_alt_error = 0;
    const control_signal_t kp_alt = control_signal_t(1.2), ki_alt = control_signal_t(0.2), kd_alt = control_signal_t(0.4);

    control_signal_t alt_error = commands.throttle - control_signal_t(current_state.pos_z);
    control_signal_t alt_output;

    pid_controller(alt_error, alt_integral, prev_alt_error, alt_output, kp_alt, ki_alt, kd_alt);

    // Update altitude integral and previous error
    alt_integral += alt_error;
    prev_alt_error = alt_error;

    // Ensure altitude control is within bounds
    if (alt_output < control_signal_t(0)) alt_output = control_signal_t(0);
    if (alt_output > control_signal_t(100)) alt_output = control_signal_t(100);

    // Combine altitude control with throttle command
    control_signal_t final_throttle = throttle_ctrl + (alt_output * control_signal_t(0.3));
    if (final_throttle > control_signal_t(100)) final_throttle = control_signal_t(100);
    if (final_throttle < control_signal_t(0)) final_throttle = control_signal_t(0);

    // Mix motor outputs
    motor_mixer(final_throttle, roll_ctrl, pitch_ctrl, yaw_ctrl, motor_speeds);

    // Apply safety monitoring
    safety_monitor(motor_speeds, current_state, emergency_stop, params);

    // Check for emergency stop conditions from commands
    // Simple absolute value calculation using if statements
    control_signal_t abs_roll, abs_pitch, abs_yaw;

    if (commands.roll_cmd < control_signal_t(0)) {
        abs_roll = -commands.roll_cmd;
    } else {
        abs_roll = commands.roll_cmd;
    }

    if (commands.pitch_cmd < control_signal_t(0)) {
        abs_pitch = -commands.pitch_cmd;
    } else {
        abs_pitch = commands.pitch_cmd;
    }

    if (commands.yaw_cmd < control_signal_t(0)) {
        abs_yaw = -commands.yaw_cmd;
    } else {
        abs_yaw = commands.yaw_cmd;
    }

    if (commands.throttle < control_signal_t(1.0) &&
        abs_roll < control_signal_t(1.0) &&
        abs_pitch < control_signal_t(1.0) &&
        abs_yaw < control_signal_t(1.0)) {
        emergency_stop = true;
    }

    // Allow reset of emergency stop if new commands are received
    if (commands.throttle > control_signal_t(5.0) && emergency_stop) {
        emergency_stop = false;
    }
}
