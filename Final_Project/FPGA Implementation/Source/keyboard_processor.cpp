#include "quadcopter_control.h"

void process_keyboard_input(key_input_t key, ControlCommands& commands) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    static control_signal_t throttle = control_signal_t(0);
    static control_signal_t roll_cmd = control_signal_t(0);
    static control_signal_t pitch_cmd = control_signal_t(0);
    static control_signal_t yaw_cmd = control_signal_t(0);
    static bool emergency_cmd = false;

    const control_signal_t cmd_increment = control_signal_t(8.0);
    const control_signal_t throttle_increment = control_signal_t(5.0);

    int key_val = key.to_int();

    if (key_val == 119) {        // 'w' - throttle up
        throttle += throttle_increment;
        emergency_cmd = false;
    }
    else if (key_val == 115) {   // 's' - throttle down
        throttle -= throttle_increment;
        emergency_cmd = false;
    }
    else if (key_val == 38) {    // Up arrow - pitch forward
        pitch_cmd += cmd_increment;
        emergency_cmd = false;
    }
    else if (key_val == 40) {    // Down arrow - pitch backward
        pitch_cmd -= cmd_increment;
        emergency_cmd = false;
    }
    else if (key_val == 37) {    // Left arrow - roll left
        roll_cmd -= cmd_increment;
        emergency_cmd = false;
    }
    else if (key_val == 39) {    // Right arrow - roll right
        roll_cmd += cmd_increment;
        emergency_cmd = false;
    }
    else if (key_val == 97) {    // 'a' - yaw left
        yaw_cmd -= cmd_increment;
        emergency_cmd = false;
    }
    else if (key_val == 100) {   // 'd' - yaw right
        yaw_cmd += cmd_increment;
        emergency_cmd = false;
    }
    else if (key_val == 32) {    // Space - emergency stop
        emergency_cmd = true;
    }

    // Limits
    if (throttle < control_signal_t(0)) throttle = control_signal_t(0);
    if (throttle > control_signal_t(100)) throttle = control_signal_t(100);
    if (roll_cmd < control_signal_t(-45)) roll_cmd = control_signal_t(-45);
    if (roll_cmd > control_signal_t(45)) roll_cmd = control_signal_t(45);
    if (pitch_cmd < control_signal_t(-45)) pitch_cmd = control_signal_t(-45);
    if (pitch_cmd > control_signal_t(45)) pitch_cmd = control_signal_t(45);
    if (yaw_cmd < control_signal_t(-180)) yaw_cmd = control_signal_t(-180);
    if (yaw_cmd > control_signal_t(180)) yaw_cmd = control_signal_t(180);

    // Decay
    roll_cmd *= control_signal_t(0.85);
    pitch_cmd *= control_signal_t(0.85);
    yaw_cmd *= control_signal_t(0.9);

    commands.throttle = throttle;
    commands.roll_cmd = roll_cmd;
    commands.pitch_cmd = pitch_cmd;
    commands.yaw_cmd = yaw_cmd;
    commands.emergency_stop = emergency_cmd;
}
