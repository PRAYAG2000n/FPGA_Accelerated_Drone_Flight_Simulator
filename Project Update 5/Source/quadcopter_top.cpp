#include "quadcopter_control.h"

void quadcopter_system(
    hls::stream<IMUData>& imu_input,
    hls::stream<key_input_t>& command_input,
    hls::stream<StateVector>& state_output,
    hls::stream<MotorSpeeds>& motor_output,
    SystemParams params
) {
    #pragma HLS INTERFACE mode=axis port=imu_input
    #pragma HLS INTERFACE mode=axis port=command_input
    #pragma HLS INTERFACE mode=axis port=state_output
    #pragma HLS INTERFACE mode=axis port=motor_output
    #pragma HLS INTERFACE mode=s_axilite port=params bundle=CTRL
    #pragma HLS INTERFACE mode=s_axilite port=return bundle=CTRL

    #pragma HLS PIPELINE off

    static StateVector current_state;
    static ControlCommands commands;
    static MotorSpeeds motor_speeds;
    static bool initialized = false;

    if (!initialized) {
        current_state.roll = fp32_t(0);
        current_state.pitch = fp32_t(0);
        current_state.yaw = fp32_t(0);
        current_state.pos_x = fp32_t(0);
        current_state.pos_y = fp32_t(0);
        current_state.pos_z = fp32_t(50.0);
        current_state.vel_x = fp32_t(0);
        current_state.vel_y = fp32_t(0);
        current_state.vel_z = fp32_t(0);
        current_state.ang_vel_x = fp32_t(0);
        current_state.ang_vel_y = fp32_t(0);
        current_state.ang_vel_z = fp32_t(0);

        commands.throttle = control_signal_t(0);
        commands.roll_cmd = control_signal_t(0);
        commands.pitch_cmd = control_signal_t(0);
        commands.yaw_cmd = control_signal_t(0);
        commands.emergency_stop = false;

        motor_speeds.front_left = control_signal_t(0);
        motor_speeds.front_right = control_signal_t(0);
        motor_speeds.rear_left = control_signal_t(0);
        motor_speeds.rear_right = control_signal_t(0);

        initialized = true;
    }

    const fp32_t alpha = fp32_t(0.98);

    if (!imu_input.empty() && !command_input.empty()) {
        IMUData imu_data;
        key_input_t key_input;

        imu_input.read(imu_data);
        command_input.read(key_input);

        process_keyboard_input(key_input, commands);
        complementary_filter(imu_data, current_state, alpha);
        flight_controller(current_state, commands, motor_speeds, params);

        state_output.write(current_state);
        motor_output.write(motor_speeds);
    } else {
        motor_speeds.front_left = control_signal_t(0);
        motor_speeds.front_right = control_signal_t(0);
        motor_speeds.rear_left = control_signal_t(0);
        motor_speeds.rear_right = control_signal_t(0);

        state_output.write(current_state);
        motor_output.write(motor_speeds);
    }
}
