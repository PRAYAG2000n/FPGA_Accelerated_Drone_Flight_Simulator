#include "quadcopter_control.h"

void quadcopter_system(
    hls::stream<IMUData>& imu_input,
    hls::stream<key_input_t>& command_input,
    hls::stream<StateVector>& state_output,
    hls::stream<MotorSpeeds>& motor_output,
    SystemParams params
) {
    #pragma HLS INTERFACE ap_ctrl_none port=return
    #pragma HLS INTERFACE axis port=imu_input
    #pragma HLS INTERFACE axis port=command_input
    #pragma HLS INTERFACE axis port=state_output
    #pragma HLS INTERFACE axis port=motor_output
    #pragma HLS INTERFACE s_axilite port=params bundle=params
    #pragma HLS DATAFLOW

    static StateVector current_state = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    static ControlCommands commands = {0, 0, 0, 0};
    static MotorSpeeds motor_speeds = {0, 0, 0, 0};
    static IMUData imu_data;
    static key_input_t key_input;

    const float_t alpha = float_t(0.98);  // Complementary filter coefficient

    // Read inputs from AXI-Stream
    if (!imu_input.empty() && !command_input.empty()) {
        imu_input.read(imu_data);
        command_input.read(key_input);

        // Process keyboard input
        process_keyboard_input(key_input, commands);

        // State estimation using complementary filter
        complementary_filter(imu_data, current_state, alpha);

        // Flight control
        flight_controller(current_state, commands, motor_speeds, params);

        // Write outputs to AXI-Stream
        state_output.write(current_state);
        motor_output.write(motor_speeds);
    }
}
