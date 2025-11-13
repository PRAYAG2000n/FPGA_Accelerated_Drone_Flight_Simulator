#include "quadcopter_control.h"
#include <iostream>
#include <cmath>

// Simple test function for basic functionality
void run_basic_tests() {
    std::cout << "=== Basic Quadcopter System Tests ===" << std::endl;

    // Test case 1: Hover condition
    std::cout << "\nTest 1: Hover Condition" << std::endl;

    // Create streams
    hls::stream<IMUData> imu_input;
    hls::stream<key_input_t> command_input;
    hls::stream<StateVector> state_output;
    hls::stream<MotorSpeeds> motor_output;

    SystemParams params;
    params.mass = fp32_t(1.0);
    params.inertia_xx = fp32_t(0.01);
    params.inertia_yy = fp32_t(0.01);
    params.inertia_zz = fp32_t(0.02);
    params.motor_thrust_coeff = fp32_t(0.0001);
    params.motor_torque_coeff = fp32_t(0.00001);
    params.sampling_period = fp32_t(0.01);

    // Hover test: level IMU data with throttle command
    IMUData imu_hover;
    imu_hover.accel_x = sensor_data_t(0);
    imu_hover.accel_y = sensor_data_t(0);
    imu_hover.accel_z = sensor_data_t(9.81);  // Gravity
    imu_hover.gyro_x = sensor_data_t(0);
    imu_hover.gyro_y = sensor_data_t(0);
    imu_hover.gyro_z = sensor_data_t(0);
    imu_hover.mag_x = sensor_data_t(1.0);
    imu_hover.mag_y = sensor_data_t(0.0);
    imu_hover.mag_z = sensor_data_t(0.0);

    key_input_t hover_cmd = 119; // 'w' for throttle up

    imu_input.write(imu_hover);
    command_input.write(hover_cmd);

    quadcopter_system(imu_input, command_input, state_output, motor_output, params);

    StateVector state;
    MotorSpeeds motors;

    if (!state_output.empty()) state_output.read(state);
    if (!motor_output.empty()) motor_output.read(motors);

    std::cout << "State - Roll: " << state.roll << ", Pitch: " << state.pitch << ", Yaw: " << state.yaw << std::endl;
    std::cout << "Motors - FL: " << motors.front_left << ", FR: " << motors.front_right
              << ", RL: " << motors.rear_left << ", RR: " << motors.rear_right << std::endl;

    // Test case 2: Forward pitch
    std::cout << "\nTest 2: Forward Pitch" << std::endl;

    IMUData imu_pitch;
    imu_pitch.accel_x = sensor_data_t(2.0);
    imu_pitch.accel_y = sensor_data_t(0);
    imu_pitch.accel_z = sensor_data_t(9.6);
    imu_pitch.gyro_x = sensor_data_t(0);
    imu_pitch.gyro_y = sensor_data_t(0.1);
    imu_pitch.gyro_z = sensor_data_t(0);
    imu_pitch.mag_x = sensor_data_t(1.0);
    imu_pitch.mag_y = sensor_data_t(0.0);
    imu_pitch.mag_z = sensor_data_t(0.0);

    key_input_t pitch_cmd = 38; // Up arrow

    imu_input.write(imu_pitch);
    command_input.write(pitch_cmd);

    quadcopter_system(imu_input, command_input, state_output, motor_output, params);

    if (!state_output.empty()) state_output.read(state);
    if (!motor_output.empty()) motor_output.read(motors);

    std::cout << "State - Roll: " << state.roll << ", Pitch: " << state.pitch << ", Yaw: " << state.yaw << std::endl;
    std::cout << "Motors - FL: " << motors.front_left << ", FR: " << motors.front_right
              << ", RL: " << motors.rear_left << ", RR: " << motors.rear_right << std::endl;

    // Test case 3: Emergency stop
    std::cout << "\nTest 3: Emergency Stop" << std::endl;

    IMUData imu_stop;
    imu_stop.accel_x = sensor_data_t(0);
    imu_stop.accel_y = sensor_data_t(0);
    imu_stop.accel_z = sensor_data_t(9.81);
    imu_stop.gyro_x = sensor_data_t(0);
    imu_stop.gyro_y = sensor_data_t(0);
    imu_stop.gyro_z = sensor_data_t(0);
    imu_stop.mag_x = sensor_data_t(1.0);
    imu_stop.mag_y = sensor_data_t(0.0);
    imu_stop.mag_z = sensor_data_t(0.0);

    key_input_t stop_cmd = 32; // Space bar

    imu_input.write(imu_stop);
    command_input.write(stop_cmd);

    quadcopter_system(imu_input, command_input, state_output, motor_output, params);

    if (!state_output.empty()) state_output.read(state);
    if (!motor_output.empty()) motor_output.read(motors);

    std::cout << "State - Roll: " << state.roll << ", Pitch: " << state.pitch << ", Yaw: " << state.yaw << std::endl;
    std::cout << "Motors - FL: " << motors.front_left << ", FR: " << motors.front_right
              << ", RL: " << motors.rear_left << ", RR: " << motors.rear_right << std::endl;
    std::cout << "All motors should be 0 for emergency stop." << std::endl;
}

// Test individual components
void test_individual_components() {
    std::cout << "\n=== Individual Component Tests ===" << std::endl;

    // Test complementary filter
    std::cout << "\nTesting Complementary Filter:" << std::endl;
    IMUData test_imu;
    test_imu.accel_x = sensor_data_t(0.5);
    test_imu.accel_y = sensor_data_t(-0.3);
    test_imu.accel_z = sensor_data_t(9.8);
    test_imu.gyro_x = sensor_data_t(0.1);
    test_imu.gyro_y = sensor_data_t(-0.05);
    test_imu.gyro_z = sensor_data_t(0.02);
    test_imu.mag_x = sensor_data_t(0.9);
    test_imu.mag_y = sensor_data_t(-0.2);
    test_imu.mag_z = sensor_data_t(0.1);

    StateVector test_state = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    complementary_filter(test_imu, test_state, fp32_t(0.98));

    std::cout << "Filtered State - Roll: " << test_state.roll
              << ", Pitch: " << test_state.pitch
              << ", Yaw: " << test_state.yaw << std::endl;

    // Test PID controller
    std::cout << "\nTesting PID Controller:" << std::endl;
    control_signal_t error = control_signal_t(10.0);
    control_signal_t integral = control_signal_t(5.0);
    control_signal_t prev_error = control_signal_t(8.0);
    control_signal_t output;

    pid_controller(error, integral, prev_error, output,
                   control_signal_t(1.0), control_signal_t(0.1), control_signal_t(0.5));

    std::cout << "PID Output: " << output << std::endl;

    // Test motor mixer
    std::cout << "\nTesting Motor Mixer:" << std::endl;
    MotorSpeeds test_motors;
    motor_mixer(control_signal_t(50.0), control_signal_t(10.0),
                control_signal_t(-5.0), control_signal_t(2.0), test_motors);

    std::cout << "Mixed Motors - FL: " << test_motors.front_left
              << ", FR: " << test_motors.front_right
              << ", RL: " << test_motors.rear_left
              << ", RR: " << test_motors.rear_right << std::endl;
}

// Test attitude controller
void test_attitude_controller() {
    std::cout << "\n=== Attitude Controller Test ===" << std::endl;

    StateVector test_state;
    test_state.roll = fp32_t(0.1);   // ~5.7 degrees
    test_state.pitch = fp32_t(-0.05); // ~-2.9 degrees
    test_state.yaw = fp32_t(0.3);    // ~17.2 degrees

    ControlCommands commands;
    commands.throttle = control_signal_t(60.0);
    commands.roll_cmd = control_signal_t(0.0);
    commands.pitch_cmd = control_signal_t(10.0); // Command 10 degrees pitch
    commands.yaw_cmd = control_signal_t(0.0);

    control_signal_t roll_ctrl, pitch_ctrl, yaw_ctrl, throttle_ctrl;
    attitude_controller(test_state, commands, roll_ctrl, pitch_ctrl, yaw_ctrl, throttle_ctrl);

    std::cout << "Attitude Control Outputs:" << std::endl;
    std::cout << "Roll Ctrl: " << roll_ctrl << ", Pitch Ctrl: " << pitch_ctrl
              << ", Yaw Ctrl: " << yaw_ctrl << ", Throttle: " << throttle_ctrl << std::endl;
}

// Test safety monitor
void test_safety_monitor() {
    std::cout << "\n=== Safety Monitor Test ===" << std::endl;

    MotorSpeeds test_motors;
    test_motors.front_left = control_signal_t(80.0);
    test_motors.front_right = control_signal_t(85.0);
    test_motors.rear_left = control_signal_t(75.0);
    test_motors.rear_right = control_signal_t(120.0); // This should trigger safety

    StateVector test_state = {0, 0, 0, 0, 0, 100.0, 0, 0, 0}; // High altitude

    SystemParams params;
    params.mass = fp32_t(1.0);
    params.inertia_xx = fp32_t(0.01);
    params.inertia_yy = fp32_t(0.01);
    params.inertia_zz = fp32_t(0.02);
    params.motor_thrust_coeff = fp32_t(0.0001);
    params.motor_torque_coeff = fp32_t(0.00001);
    params.sampling_period = fp32_t(0.01);

    bool emergency_stop = false;

    std::cout << "Before Safety Monitor:" << std::endl;
    std::cout << "Motors - FL: " << test_motors.front_left
              << ", FR: " << test_motors.front_right
              << ", RL: " << test_motors.rear_left
              << ", RR: " << test_motors.rear_right << std::endl;

    safety_monitor(test_motors, test_state, emergency_stop, params);

    std::cout << "After Safety Monitor:" << std::endl;
    std::cout << "Motors - FL: " << test_motors.front_left
              << ", FR: " << test_motors.front_right
              << ", RL: " << test_motors.rear_left
              << ", RR: " << test_motors.rear_right << std::endl;
    std::cout << "Emergency Stop: " << (emergency_stop ? "TRUE" : "FALSE") << std::endl;
}

int main() {
    std::cout << "Quadcopter Control System Test Suite" << std::endl;
    std::cout << "====================================" << std::endl;

    // Run all tests
    run_basic_tests();
    test_individual_components();
    test_attitude_controller();
    test_safety_monitor();

    std::cout << "\nAll tests completed successfully!" << std::endl;
    return 0;
}
