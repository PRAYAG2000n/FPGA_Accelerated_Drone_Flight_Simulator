#include "quadcopter_control.h"

void process_keyboard_input(key_input_t key, ControlCommands& commands) {
    #pragma HLS PIPELINE off
    #pragma HLS INLINE off

    // Persistent command state
    static control_signal_t throttle = control_signal_t(0);
    static control_signal_t roll_cmd = control_signal_t(0);
    static control_signal_t pitch_cmd = control_signal_t(0);
    static control_signal_t yaw_cmd = control_signal_t(0);
    static bool emergency_cmd = false;

    // Increments (in radians for angles, normalized 0-1 for throttle)
    const control_signal_t angle_increment = control_signal_t(0.05);  // ~3 degrees
    const control_signal_t throttle_increment = control_signal_t(0.05);  // 5%

    int key_val = key.to_int();

    // Process key commands
    if (key_val == 119) {  // 'w' - throttle up
        throttle = throttle + throttle_increment;
        emergency_cmd = false;
    }
    else if (key_val == 115) {  // 's' - throttle down
        throttle = throttle - throttle_increment;
        emergency_cmd = false;
    }
    else if (key_val == 38) {  // Up arrow - pitch forward (nose down)
        pitch_cmd = pitch_cmd - angle_increment;
        emergency_cmd = false;
    }
    else if (key_val == 40) {  // Down arrow - pitch backward (nose up)
        pitch_cmd = pitch_cmd + angle_increment;
        emergency_cmd = false;
    }
    else if (key_val == 37) {  // Left arrow - roll left
        roll_cmd = roll_cmd - angle_increment;
        emergency_cmd = false;
    }
    else if (key_val == 39) {  // Right arrow - roll right
        roll_cmd = roll_cmd + angle_increment;
        emergency_cmd = false;
    }
    else if (key_val == 97) {  // 'a' - yaw left
        yaw_cmd = yaw_cmd - angle_increment;
        emergency_cmd = false;
    }
    else if (key_val == 100) {  // 'd' - yaw right
        yaw_cmd = yaw_cmd + angle_increment;
        emergency_cmd = false;
    }
    else if (key_val == 32) {  // Space - emergency stop
        emergency_cmd = true;
    }

    // Limit throttle [0, 1]
    if (throttle < control_signal_t(0)) {
        throttle = control_signal_t(0);
    }
    if (throttle > control_signal_t(1)) {
        throttle = control_signal_t(1);
    }

    // Limit roll [-45°, +45°] in radians = [-0.785, +0.785]
    const control_signal_t max_angle = control_signal_t(0.785);
    if (roll_cmd < -max_angle) {
        roll_cmd = -max_angle;
    }
    if (roll_cmd > max_angle) {
        roll_cmd = max_angle;
    }

    // Limit pitch
    if (pitch_cmd < -max_angle) {
        pitch_cmd = -max_angle;
    }
    if (pitch_cmd > max_angle) {
        pitch_cmd = max_angle;
    }

    // Limit yaw [-180°, +180°] in radians = [-PI, +PI]
    const control_signal_t PI = control_signal_t(3.14159265);
    if (yaw_cmd < -PI) {
        yaw_cmd = -PI;
    }
    if (yaw_cmd > PI) {
        yaw_cmd = PI;
    }

    // Apply decay for self-centering (return to level flight)
    const control_signal_t decay = control_signal_t(0.95);
    roll_cmd = roll_cmd * decay;
    pitch_cmd = pitch_cmd * decay;
    yaw_cmd = yaw_cmd * decay;

    // Set output commands
    commands.throttle = throttle;
    commands.roll_cmd = roll_cmd;
    commands.pitch_cmd = pitch_cmd;
    commands.yaw_cmd = yaw_cmd;
    commands.emergency_stop = emergency_cmd;
}
