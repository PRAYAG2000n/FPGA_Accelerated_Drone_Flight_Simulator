#include "quadcopter_control.h"

void attitude_controller(StateVector current_state, ControlCommands commands,
                        control_signal_t& roll_ctrl, control_signal_t& pitch_ctrl,
                        control_signal_t& yaw_ctrl, control_signal_t& throttle_ctrl) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    static control_signal_t roll_integral = 0, pitch_integral = 0, yaw_integral = 0;
    static control_signal_t prev_roll_error = 0, prev_pitch_error = 0, prev_yaw_error = 0;

    // PID gains for attitude control - tuned for stability
    const control_signal_t kp_roll = control_signal_t(3.0);
    const control_signal_t ki_roll = control_signal_t(0.05);
    const control_signal_t kd_roll = control_signal_t(0.8);

    const control_signal_t kp_pitch = control_signal_t(3.0);
    const control_signal_t ki_pitch = control_signal_t(0.05);
    const control_signal_t kd_pitch = control_signal_t(0.8);

    const control_signal_t kp_yaw = control_signal_t(2.5);
    const control_signal_t ki_yaw = control_signal_t(0.02);
    const control_signal_t kd_yaw = control_signal_t(0.3);

    // Convert current angles from radians to degrees for control
    const fp32_t rad_to_deg = fp32_t(180.0 / 3.14159265358979323846);
    control_signal_t roll_deg = control_signal_t(current_state.roll * rad_to_deg);
    control_signal_t pitch_deg = control_signal_t(current_state.pitch * rad_to_deg);
    control_signal_t yaw_deg = control_signal_t(current_state.yaw * rad_to_deg);

    // Calculate attitude errors
    control_signal_t roll_error = commands.roll_cmd - roll_deg;
    control_signal_t pitch_error = commands.pitch_cmd - pitch_deg;
    control_signal_t yaw_error = commands.yaw_cmd - yaw_deg;

    // Normalize yaw error for shortest path
    if (yaw_error > control_signal_t(180)) yaw_error -= control_signal_t(360);
    if (yaw_error < control_signal_t(-180)) yaw_error += control_signal_t(360);

    // PID controllers for attitude
    control_signal_t roll_output, pitch_output, yaw_output;

    pid_controller(roll_error, roll_integral, prev_roll_error, roll_output, kp_roll, ki_roll, kd_roll);
    pid_controller(pitch_error, pitch_integral, prev_pitch_error, pitch_output, kp_pitch, ki_pitch, kd_pitch);
    pid_controller(yaw_error, yaw_integral, prev_yaw_error, yaw_output, kp_yaw, ki_yaw, kd_yaw);

    // Update integrals and previous errors
    roll_integral += roll_error;
    pitch_integral += pitch_error;
    yaw_integral += yaw_error;

    prev_roll_error = roll_error;
    prev_pitch_error = pitch_error;
    prev_yaw_error = yaw_error;

    // Output control signals
    roll_ctrl = roll_output;
    pitch_ctrl = pitch_output;
    yaw_ctrl = yaw_output;
    throttle_ctrl = commands.throttle;

    // Apply control limits to prevent excessive maneuvers
    if (roll_ctrl > control_signal_t(30)) roll_ctrl = control_signal_t(30);
    if (roll_ctrl < control_signal_t(-30)) roll_ctrl = control_signal_t(-30);
    if (pitch_ctrl > control_signal_t(30)) pitch_ctrl = control_signal_t(30);
    if (pitch_ctrl < control_signal_t(-30)) pitch_ctrl = control_signal_t(-30);
    if (yaw_ctrl > control_signal_t(60)) yaw_ctrl = control_signal_t(60);
    if (yaw_ctrl < control_signal_t(-60)) yaw_ctrl = control_signal_t(-60);
}
