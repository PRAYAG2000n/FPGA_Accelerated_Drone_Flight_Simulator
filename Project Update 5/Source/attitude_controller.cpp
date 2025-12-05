// attitude_controller.cpp - OPTIMIZED FOR TIMING
#include "quadcopter_control.h"

void attitude_controller(StateVector current_state, ControlCommands commands,
                        control_signal_t& roll_ctrl, control_signal_t& pitch_ctrl,
                        control_signal_t& yaw_ctrl, control_signal_t& throttle_ctrl) {
    #pragma HLS INLINE off
    #pragma HLS PIPELINE II=2  // Allow 2 cycles

    // Static state
    static control_signal_t roll_integral = control_signal_t(0);
    static control_signal_t pitch_integral = control_signal_t(0);
    static control_signal_t yaw_integral = control_signal_t(0);
    static control_signal_t prev_roll_error = control_signal_t(0);
    static control_signal_t prev_pitch_error = control_signal_t(0);
    static control_signal_t prev_yaw_error = control_signal_t(0);

    // Use same gains for roll/pitch to reduce constants
    const control_signal_t kp = control_signal_t(2.5);
    const control_signal_t ki = control_signal_t(0.01);
    const control_signal_t kd = control_signal_t(0.8);
    const control_signal_t kp_yaw = control_signal_t(2.0);
    const control_signal_t ki_yaw = control_signal_t(0.005);
    const control_signal_t kd_yaw = control_signal_t(0.3);

    // Calculate errors
    control_signal_t roll_error = commands.roll_cmd - control_signal_t(current_state.roll);
    control_signal_t pitch_error = commands.pitch_cmd - control_signal_t(current_state.pitch);
    control_signal_t yaw_error = commands.yaw_cmd - control_signal_t(current_state.yaw);

    // Yaw wrap (simplified)
    const control_signal_t PI = control_signal_t(3.14159);
    if (yaw_error > PI) yaw_error -= control_signal_t(6.28318);
    else if (yaw_error < -PI) yaw_error += control_signal_t(6.28318);

    // Derivatives
    control_signal_t roll_deriv = roll_error - prev_roll_error;
    control_signal_t pitch_deriv = pitch_error - prev_pitch_error;
    control_signal_t yaw_deriv = yaw_error - prev_yaw_error;

    // Update integrals with anti-windup
    const control_signal_t max_int = control_signal_t(0.5);

    roll_integral += roll_error;
    if (roll_integral > max_int) roll_integral = max_int;
    else if (roll_integral < -max_int) roll_integral = -max_int;

    pitch_integral += pitch_error;
    if (pitch_integral > max_int) pitch_integral = max_int;
    else if (pitch_integral < -max_int) pitch_integral = -max_int;

    yaw_integral += yaw_error;
    if (yaw_integral > max_int) yaw_integral = max_int;
    else if (yaw_integral < -max_int) yaw_integral = -max_int;

    // PID outputs - compute P, I, D terms separately then sum
    // This allows HLS to parallelize the multiplications
    control_signal_t roll_p = kp * roll_error;
    control_signal_t roll_i = ki * roll_integral;
    control_signal_t roll_d = kd * roll_deriv;

    control_signal_t pitch_p = kp * pitch_error;
    control_signal_t pitch_i = ki * pitch_integral;
    control_signal_t pitch_d = kd * pitch_deriv;

    control_signal_t yaw_p = kp_yaw * yaw_error;
    control_signal_t yaw_i = ki_yaw * yaw_integral;
    control_signal_t yaw_d = kd_yaw * yaw_deriv;

    #pragma HLS BIND_OP variable=roll_p op=mul impl=dsp
    #pragma HLS BIND_OP variable=pitch_p op=mul impl=dsp
    #pragma HLS BIND_OP variable=yaw_p op=mul impl=dsp

    roll_ctrl = roll_p + roll_i + roll_d;
    pitch_ctrl = pitch_p + pitch_i + pitch_d;
    yaw_ctrl = yaw_p + yaw_i + yaw_d;

    // Update previous errors
    prev_roll_error = roll_error;
    prev_pitch_error = pitch_error;
    prev_yaw_error = yaw_error;

    // Output throttle
    throttle_ctrl = commands.throttle;

    // Clamp outputs
    const control_signal_t max_ctrl = control_signal_t(0.5);
    if (roll_ctrl > max_ctrl) roll_ctrl = max_ctrl;
    else if (roll_ctrl < -max_ctrl) roll_ctrl = -max_ctrl;
    if (pitch_ctrl > max_ctrl) pitch_ctrl = max_ctrl;
    else if (pitch_ctrl < -max_ctrl) pitch_ctrl = -max_ctrl;
    if (yaw_ctrl > max_ctrl) yaw_ctrl = max_ctrl;
    else if (yaw_ctrl < -max_ctrl) yaw_ctrl = -max_ctrl;
}
