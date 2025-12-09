#include "quadcopter_control.h"

void flight_controller(StateVector current_state, ControlCommands commands,
                      MotorSpeeds& motor_speeds, SystemParams params) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    static bool emergency_stop = false;

    control_signal_t roll_ctrl, pitch_ctrl, yaw_ctrl, throttle_ctrl;

    if (commands.emergency_stop) {
        emergency_stop = true;
    }

    // Attitude control
    attitude_controller(current_state, commands, roll_ctrl, pitch_ctrl, yaw_ctrl, throttle_ctrl);

    // Altitude control
    control_signal_t altitude_throttle;
    altitude_controller(current_state, commands, altitude_throttle, params);

    // Combine throttle (80% altitude, 20% attitude)
    control_signal_t final_throttle = (throttle_ctrl * control_signal_t(0.2)) +
                                     (altitude_throttle * control_signal_t(0.8));

    if (final_throttle < control_signal_t(0)) final_throttle = control_signal_t(0);
    if (final_throttle > control_signal_t(100)) final_throttle = control_signal_t(100);

    // Motor mixing
    motor_mixer(final_throttle, roll_ctrl, pitch_ctrl, yaw_ctrl, motor_speeds);

    // Safety check
    safety_monitor(motor_speeds, current_state, emergency_stop, params);

    // Reset emergency if all commands zero
    if (emergency_stop && commands.throttle < control_signal_t(1.0) &&
        commands.roll_cmd == control_signal_t(0) &&
        commands.pitch_cmd == control_signal_t(0) &&
        commands.yaw_cmd == control_signal_t(0) &&
        !commands.emergency_stop) {
        emergency_stop = false;
    }
}
