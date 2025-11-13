#include "quadcopter_control.h"
#include <iostream>

// Use the existing types from our header
void physics_simulation(MotorSpeeds motors, IMUData& sensors, StateVector& state) {
    #pragma HLS INLINE off

    static ap_fixed<16,8> velocity = 0;
    static ap_fixed<16,8> altitude = 50.0; // Start at altitude 50

    // Calculate total thrust (average of all motors)
    ap_fixed<16,8> total_thrust = (motors.front_left + motors.front_right +
                                  motors.rear_left + motors.rear_right) / ap_fixed<16,8>(4.0);

    // Simple physics: acceleration = (thrust - gravity) / mass
    // Assume gravity requires ~50% throttle to hover
    ap_fixed<16,8> acceleration = (total_thrust - ap_fixed<16,8>(50.0)) / ap_fixed<16,8>(10.0);

    // Integrate acceleration to get velocity
    velocity += acceleration * ap_fixed<16,8>(0.01); // dt = 0.01s

    // Integrate velocity to get altitude
    altitude += velocity * ap_fixed<16,8>(0.01);

    // Ensure altitude doesn't go below 0
    if (altitude < ap_fixed<16,8>(0)) {
        altitude = ap_fixed<16,8>(0);
        velocity = ap_fixed<16,8>(0);
    }

    // Generate simulated sensor data
    sensors.accel_x = ap_fixed<16,8>(0);  // Assume level flight for now
    sensors.accel_y = ap_fixed<16,8>(0);
    sensors.accel_z = acceleration + ap_fixed<16,8>(9.81); // Add gravity

    // Simulate gyro data based on motor differences
    sensors.gyro_x = (motors.front_left - motors.front_right) * ap_fixed<16,8>(0.01);
    sensors.gyro_y = (motors.front_left - motors.rear_left) * ap_fixed<16,8>(0.01);
    sensors.gyro_z = ap_fixed<16,8>(0); // No yaw for now

    // Simulate magnetometer (fixed for now)
    sensors.mag_x = ap_fixed<16,8>(1.0);
    sensors.mag_y = ap_fixed<16,8>(0.0);
    sensors.mag_z = ap_fixed<16,8>(0.0);

    // Update state
    state.pos_z = altitude;
}

void print_sensor_data(IMUData sensors) {
    std::cout << "IMU Data - Accel: (" << sensors.accel_x << ", " << sensors.accel_y
              << ", " << sensors.accel_z << "), Gyro: (" << sensors.gyro_x
              << ", " << sensors.gyro_y << ", " << sensors.gyro_z
              << "), Mag: (" << sensors.mag_x << ", " << sensors.mag_y
              << ", " << sensors.mag_z << ")" << std::endl;
}

void print_drone_state(StateVector state) {
    std::cout << "Drone State - Altitude: " << state.pos_z
              << ", Roll: " << state.roll << ", Pitch: " << state.pitch
              << ", Yaw: " << state.yaw << std::endl;
}

void print_motor_speeds(MotorSpeeds motors) {
    std::cout << "Motor Speeds - FL: " << motors.front_left
              << ", FR: " << motors.front_right << ", RL: " << motors.rear_left
              << ", RR: " << motors.rear_right << std::endl;
}

int main() {
    std::cout << "Starting FPGA Quadcopter Simulation" << std::endl;
    std::cout << "===================================" << std::endl;

    // Initialize system
    IMUData sensors = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    StateVector state = {0, 0, 0, 0, 0, 50.0, 0, 0, 0}; // Start at altitude 50
    MotorSpeeds motors = {0, 0, 0, 0};

    // System parameters
    SystemParams params;
    params.mass = 1.0;
    params.inertia_xx = 0.01;
    params.inertia_yy = 0.01;
    params.inertia_zz = 0.02;
    params.motor_thrust_coeff = 0.0001;
    params.motor_torque_coeff = 0.00001;
    params.sampling_period = 0.01;

    // Test sequence: throttle up, pitch forward, roll right, throttle down
    key_input_t test_sequence[] = {119, 119, 38, 39, 115, 115}; // W, W, UP, RIGHT, S, S
    const char* cmd_names[] = {"THROTTLE UP", "THROTTLE UP", "PITCH FWD", "ROLL RIGHT", "THROTTLE DOWN", "THROTTLE DOWN"};

    std::cout << "Initial State:" << std::endl;
    print_drone_state(state);
    std::cout << std::endl;

    // Create streams for the FPGA system
    hls::stream<IMUData> imu_input;
    hls::stream<key_input_t> command_input;
    hls::stream<StateVector> state_output;
    hls::stream<MotorSpeeds> motor_output;

    // Run simulation steps
    for (int i = 0; i < 6; i++) {
        std::cout << "Step " << i+1 << ": " << cmd_names[i] << std::endl;

        // Send current sensor data and command to FPGA
        imu_input.write(sensors);
        command_input.write(test_sequence[i]);

        // Run FPGA system
        quadcopter_system(imu_input, command_input, state_output, motor_output, params);

        // Read results from FPGA
        if (!state_output.empty()) {
            state_output.read(state);
        }
        if (!motor_output.empty()) {
            motor_output.read(motors);
        }

        // Run physics simulation with new motor commands
        physics_simulation(motors, sensors, state);

        // Print results
        print_motor_speeds(motors);
        print_sensor_data(sensors);
        print_drone_state(state);
        std::cout << "----------------------------------------" << std::endl;
    }

    // Test emergency stop
    std::cout << "Testing Emergency Stop (SPACE):" << std::endl;
    imu_input.write(sensors);
    command_input.write(32); // Space bar for emergency stop

    quadcopter_system(imu_input, command_input, state_output, motor_output, params);

    if (!state_output.empty()) state_output.read(state);
    if (!motor_output.empty()) motor_output.read(motors);

    print_motor_speeds(motors);
    std::cout << "All motors should be 0 for emergency stop." << std::endl;

    std::cout << "Simulation completed successfully!" << std::endl;
    return 0;
}
