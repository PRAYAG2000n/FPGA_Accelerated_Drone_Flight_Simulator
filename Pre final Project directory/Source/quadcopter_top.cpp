#include "quadcopter_control.h"

#ifdef ALVEO_U280_HW
// ============================================================================
// ALVEO U280 VERSION - Memory-mapped AXI interfaces for PCIe deployment
// ============================================================================
extern "C" {

void quadcopter_system(
    IMUData* imu_input,
    key_input_t* command_input,
    StateVector* state_output,
    MotorSpeeds* motor_output,
    fp32_t mass,
    fp32_t inertia_xx,
    fp32_t inertia_yy,
    fp32_t inertia_zz,
    fp32_t motor_thrust_coeff,
    fp32_t motor_torque_coeff,
    fp32_t arm_length,
    fp32_t sampling_period,
    fp32_t gravity,
    int num_samples
) {
    #pragma HLS INTERFACE m_axi port=imu_input offset=slave bundle=gmem0 depth=1024
    #pragma HLS INTERFACE m_axi port=command_input offset=slave bundle=gmem1 depth=1024
    #pragma HLS INTERFACE m_axi port=state_output offset=slave bundle=gmem2 depth=1024
    #pragma HLS INTERFACE m_axi port=motor_output offset=slave bundle=gmem3 depth=1024

    #pragma HLS INTERFACE s_axilite port=mass bundle=control
    #pragma HLS INTERFACE s_axilite port=inertia_xx bundle=control
    #pragma HLS INTERFACE s_axilite port=inertia_yy bundle=control
    #pragma HLS INTERFACE s_axilite port=inertia_zz bundle=control
    #pragma HLS INTERFACE s_axilite port=motor_thrust_coeff bundle=control
    #pragma HLS INTERFACE s_axilite port=motor_torque_coeff bundle=control
    #pragma HLS INTERFACE s_axilite port=arm_length bundle=control
    #pragma HLS INTERFACE s_axilite port=sampling_period bundle=control
    #pragma HLS INTERFACE s_axilite port=gravity bundle=control
    #pragma HLS INTERFACE s_axilite port=num_samples bundle=control
    #pragma HLS INTERFACE s_axilite port=return bundle=control

    SystemParams params;
    params.mass = mass;
    params.inertia_xx = inertia_xx;
    params.inertia_yy = inertia_yy;
    params.inertia_zz = inertia_zz;
    params.motor_thrust_coeff = motor_thrust_coeff;
    params.motor_torque_coeff = motor_torque_coeff;
    params.arm_length = arm_length;
    params.sampling_period = sampling_period;
    params.gravity = gravity;

    static StateVector current_state = {0, 0, 0, 0, 0, 50.0, 0, 0, 0, 0, 0, 0};
    static ControlCommands commands = {0, 0, 0, 0, false};
    static MotorSpeeds motor_speeds = {0, 0, 0, 0};
    const fp32_t alpha = fp32_t(0.98);

    PROCESS_LOOP:
    for (int i = 0; i < num_samples; i++) {
        #pragma HLS PIPELINE II=1

        IMUData imu_data = imu_input[i];
        key_input_t key_input = command_input[i];

        process_keyboard_input(key_input, commands);
        complementary_filter(imu_data, current_state, alpha);
        flight_controller(current_state, commands, motor_speeds, params);

        state_output[i] = current_state;
        motor_output[i] = motor_speeds;
    }
}

} // extern "C"

#else
// ============================================================================
// SIMULATION VERSION - AXI-Stream interfaces for HLS C-Sim and Co-Sim
// ============================================================================

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

    static StateVector current_state = {0, 0, 0, 0, 0, 50.0, 0, 0, 0, 0, 0, 0};
    static ControlCommands commands = {0, 0, 0, 0, false};
    static MotorSpeeds motor_speeds = {0, 0, 0, 0};

    const fp32_t alpha = fp32_t(0.98);

    // Only process if data is available
    if (!imu_input.empty() && !command_input.empty()) {
        IMUData imu_data;
        key_input_t key_input;

        // Read inputs
        imu_input.read(imu_data);
        command_input.read(key_input);

        // Process
        process_keyboard_input(key_input, commands);
        complementary_filter(imu_data, current_state, alpha);
        flight_controller(current_state, commands, motor_speeds, params);

        // Write outputs
        state_output.write(current_state);
        motor_output.write(motor_speeds);
    }
}

#endif
