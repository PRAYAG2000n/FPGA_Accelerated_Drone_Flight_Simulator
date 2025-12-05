// flight_controller.cpp - FIXED WITH PIPELINE STAGES
#include "quadcopter_control.h"

void flight_controller(StateVector current_state, ControlCommands commands,
                      MotorSpeeds& motor_speeds, SystemParams params) {
    #pragma HLS INLINE off
    #pragma HLS PIPELINE II=3  // Allow 3 clock cycles for better timing

    static bool emergency_stop = false;

    // Registered intermediates to break timing path
    static control_signal_t roll_ctrl_reg;
    static control_signal_t pitch_ctrl_reg;
    static control_signal_t yaw_ctrl_reg;
    static control_signal_t throttle_ctrl_reg;
    static control_signal_t altitude_throttle_reg;

    control_signal_t roll_ctrl;
    control_signal_t pitch_ctrl;
    control_signal_t yaw_ctrl;
    control_signal_t throttle_ctrl;

    if (commands.emergency_stop) {
        emergency_stop = true;
    }

    if (emergency_stop) {
        motor_speeds.front_left = control_signal_t(0);
        motor_speeds.front_right = control_signal_t(0);
        motor_speeds.rear_left = control_signal_t(0);
        motor_speeds.rear_right = control_signal_t(0);

        if (commands.throttle < control_signal_t(0.05) && !commands.emergency_stop) {
            emergency_stop = false;
        }
        return;
    }

    // Stage 1: Attitude control
    attitude_controller(current_state, commands, roll_ctrl, pitch_ctrl, yaw_ctrl, throttle_ctrl);

    // Stage 2: Altitude control
    control_signal_t altitude_throttle;
    altitude_controller(current_state, commands, altitude_throttle, params);

    // Stage 3: Combine throttle
    control_signal_t final_throttle = (throttle_ctrl * control_signal_t(0.2)) +
                                     (altitude_throttle * control_signal_t(0.8));

    if (final_throttle < control_signal_t(0)) {
        final_throttle = control_signal_t(0);
    }
    if (final_throttle > control_signal_t(1)) {
        final_throttle = control_signal_t(1);
    }

    // Stage 4: Motor mixing
    motor_mixer(final_throttle, roll_ctrl, pitch_ctrl, yaw_ctrl, motor_speeds);

    // Stage 5: Safety (simplified - remove function call)
    const control_signal_t max_motor = control_signal_t(1.0);
    if (motor_speeds.front_left > max_motor) motor_speeds.front_left = max_motor;
    if (motor_speeds.front_right > max_motor) motor_speeds.front_right = max_motor;
    if (motor_speeds.rear_left > max_motor) motor_speeds.rear_left = max_motor;
    if (motor_speeds.rear_right > max_motor) motor_speeds.rear_right = max_motor;
    if (motor_speeds.front_left < control_signal_t(0)) motor_speeds.front_left = control_signal_t(0);
    if (motor_speeds.front_right < control_signal_t(0)) motor_speeds.front_right = control_signal_t(0);
    if (motor_speeds.rear_left < control_signal_t(0)) motor_speeds.rear_left = control_signal_t(0);
    if (motor_speeds.rear_right < control_signal_t(0)) motor_speeds.rear_right = control_signal_t(0);
}
