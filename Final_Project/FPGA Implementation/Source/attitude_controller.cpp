#include "quadcopter_control.h"
#include "hls_math.h"

void attitude_controller(StateVector current_state, ControlCommands commands,
                        control_signal_t& roll_ctrl, control_signal_t& pitch_ctrl,
                        control_signal_t& yaw_ctrl, control_signal_t& throttle_ctrl) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    static control_signal_t roll_integral = 0, pitch_integral = 0, yaw_integral = 0;
    static control_signal_t prev_roll_error = 0, prev_pitch_error = 0, prev_yaw_error = 0;

    // PID Gains
    const control_signal_t kp_roll = control_signal_t(1.5);
    const control_signal_t ki_roll = control_signal_t(0.02);
    const control_signal_t kd_roll = control_signal_t(1.2);

    const control_signal_t kp_pitch = control_signal_t(1.5);
    const control_signal_t ki_pitch = control_signal_t(0.02);
    const control_signal_t kd_pitch = control_signal_t(1.2);

    const control_signal_t kp_yaw = control_signal_t(1.8);
    const control_signal_t ki_yaw = control_signal_t(0.01);
    const control_signal_t kd_yaw = control_signal_t(0.5);

    // Convert radians to degrees
    const fp32_t rad_to_deg = fp32_t(57.2957795);
    control_signal_t roll_deg = control_signal_t(current_state.roll * rad_to_deg);
    control_signal_t pitch_deg = control_signal_t(current_state.pitch * rad_to_deg);
    control_signal_t yaw_deg = control_signal_t(current_state.yaw * rad_to_deg);

    // Calculate errors
    control_signal_t deadband = control_signal_t(0.5);
    control_signal_t roll_error = commands.roll_cmd - roll_deg;
    control_signal_t pitch_error = commands.pitch_cmd - pitch_deg;
    control_signal_t yaw_error = commands.yaw_cmd - yaw_deg;

    // Apply deadband
    if (roll_error < control_signal_t(0)) {
        if (-roll_error < deadband) roll_error = 0;
    } else {
        if (roll_error < deadband) roll_error = 0;
    }

    if (pitch_error < control_signal_t(0)) {
        if (-pitch_error < deadband) pitch_error = 0;
    } else {
        if (pitch_error < deadband) pitch_error = 0;
    }

    if (yaw_error < control_signal_t(0)) {
        if (-yaw_error < deadband) yaw_error = 0;
    } else {
        if (yaw_error < deadband) yaw_error = 0;
    }

    // Normalize yaw
    if (yaw_error > control_signal_t(180)) yaw_error -= control_signal_t(360);
    if (yaw_error < control_signal_t(-180)) yaw_error += control_signal_t(360);

    // PID controllers
    control_signal_t roll_output, pitch_output, yaw_output;
    pid_controller(roll_error, roll_integral, prev_roll_error, roll_output, kp_roll, ki_roll, kd_roll);
    pid_controller(pitch_error, pitch_integral, prev_pitch_error, pitch_output, kp_pitch, ki_pitch, kd_pitch);
    pid_controller(yaw_error, yaw_integral, prev_yaw_error, yaw_output, kp_yaw, ki_yaw, kd_yaw);

    // Update state
    roll_integral += roll_error;
    pitch_integral += pitch_error;
    yaw_integral += yaw_error;
    prev_roll_error = roll_error;
    prev_pitch_error = pitch_error;
    prev_yaw_error = yaw_error;

    // Output
    roll_ctrl = roll_output;
    pitch_ctrl = pitch_output;
    yaw_ctrl = yaw_output;
    throttle_ctrl = commands.throttle;

    // Limits
    if (roll_ctrl > control_signal_t(20)) roll_ctrl = control_signal_t(20);
    if (roll_ctrl < control_signal_t(-20)) roll_ctrl = control_signal_t(-20);
    if (pitch_ctrl > control_signal_t(20)) pitch_ctrl = control_signal_t(20);
    if (pitch_ctrl < control_signal_t(-20)) pitch_ctrl = control_signal_t(-20);
    if (yaw_ctrl > control_signal_t(30)) yaw_ctrl = control_signal_t(30);
    if (yaw_ctrl < control_signal_t(-30)) yaw_ctrl = control_signal_t(-30);
}
