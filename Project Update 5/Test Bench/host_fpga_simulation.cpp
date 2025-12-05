#include "quadcopter_control.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <sys/stat.h>

// Helper functions
template<typename T>
double to_double(T value) {
    return value.to_double();
}

template<typename T>
T from_double(double value) {
    return T(value);
}

// Generate IMU data from current state
void generate_imu_data(StateVector& state, IMUData& sensors, double gravity = 9.80859375) {
    double roll = to_double(state.roll);
    double pitch = to_double(state.pitch);
    double yaw = to_double(state.yaw);

    double ax = -gravity * sin(pitch);
    double ay = gravity * sin(roll) * cos(pitch);
    double az = gravity * cos(roll) * cos(pitch);

    sensors.accel_x = from_double<sensor_data_t>(ax);
    sensors.accel_y = from_double<sensor_data_t>(ay);
    sensors.accel_z = from_double<sensor_data_t>(az);

    sensors.gyro_x = from_double<sensor_data_t>(to_double(state.ang_vel_x));
    sensors.gyro_y = from_double<sensor_data_t>(to_double(state.ang_vel_y));
    sensors.gyro_z = from_double<sensor_data_t>(to_double(state.ang_vel_z));

    sensors.mag_x = from_double<sensor_data_t>(cos(yaw));
    sensors.mag_y = from_double<sensor_data_t>(sin(yaw));
    sensors.mag_z = from_double<sensor_data_t>(0.0);
}

// Apply motor forces to update state
void apply_physics(MotorSpeeds& motors, StateVector& state, double dt = 0.01) {
    double m1 = to_double(motors.front_left);
    double m2 = to_double(motors.front_right);
    double m3 = to_double(motors.rear_left);
    double m4 = to_double(motors.rear_right);

    double roll_torque = ((m1 + m3) - (m2 + m4)) * 0.5;
    double pitch_torque = ((m3 + m4) - (m1 + m2)) * 0.5;
    double yaw_torque = ((m1 + m4) - (m2 + m3)) * 0.1;

    double ang_vel_x = to_double(state.ang_vel_x) + roll_torque * dt * 10.0;
    double ang_vel_y = to_double(state.ang_vel_y) + pitch_torque * dt * 10.0;
    double ang_vel_z = to_double(state.ang_vel_z) + yaw_torque * dt * 10.0;

    state.ang_vel_x = from_double<fp32_t>(ang_vel_x);
    state.ang_vel_y = from_double<fp32_t>(ang_vel_y);
    state.ang_vel_z = from_double<fp32_t>(ang_vel_z);

    double roll = to_double(state.roll) + ang_vel_x * dt;
    double pitch = to_double(state.pitch) + ang_vel_y * dt;
    double yaw = to_double(state.yaw) + ang_vel_z * dt;

    state.roll = from_double<fp32_t>(roll);
    state.pitch = from_double<fp32_t>(pitch);
    state.yaw = from_double<fp32_t>(yaw);
}

int main() {
    std::cout << "Starting Drone Controller Test Bench - Project Update 3" << std::endl;
    std::cout << "Testing ALL modules: keyboard_processor, complementary_filter," << std::endl;
    std::cout << "attitude_controller, altitude_controller, flight_controller," << std::endl;
    std::cout << "motor_mixer, pid_controller, safety_monitor" << std::endl;
    std::cout << std::endl;

    // Create HLS streams for top-level function
    hls::stream<IMUData> imu_stream;
    hls::stream<key_input_t> key_stream;
    hls::stream<StateVector> state_stream;
    hls::stream<MotorSpeeds> motor_stream;

    // Initialize state
    StateVector state;
    state.roll = fp32_t(0);
    state.pitch = fp32_t(0);
    state.yaw = fp32_t(0);
    state.pos_x = fp32_t(0);
    state.pos_y = fp32_t(0);
    state.pos_z = fp32_t(10.0);
    state.vel_x = fp32_t(0);
    state.vel_y = fp32_t(0);
    state.vel_z = fp32_t(0);
    state.ang_vel_x = fp32_t(0);
    state.ang_vel_y = fp32_t(0);
    state.ang_vel_z = fp32_t(0);

    // Initialize sensors
    IMUData sensors;
    generate_imu_data(state, sensors);

    // Initialize motors
    MotorSpeeds motors;
    motors.front_left = control_signal_t(0);
    motors.front_right = control_signal_t(0);
    motors.rear_left = control_signal_t(0);
    motors.rear_right = control_signal_t(0);

    // Initialize commands (output from keyboard_processor)
    ControlCommands commands;
    commands.throttle = control_signal_t(0);
    commands.roll_cmd = control_signal_t(0);
    commands.pitch_cmd = control_signal_t(0);
    commands.yaw_cmd = control_signal_t(0);
    commands.emergency_stop = false;

    // System parameters
    SystemParams params;
    params.mass = fp32_t(1.2);
    params.inertia_xx = fp32_t(0.015);
    params.inertia_yy = fp32_t(0.015);
    params.inertia_zz = fp32_t(0.025);
    params.motor_thrust_coeff = fp32_t(0.12);
    params.motor_torque_coeff = fp32_t(0.01);
    params.arm_length = fp32_t(0.25);
    params.sampling_period = fp32_t(0.01);
    params.gravity = fp32_t(9.81);

    const fp32_t alpha = fp32_t(0.98);

    // FPGA state output
    StateVector fpga_state;

    // Create output directory
    mkdir("tb", 0777);

    // Open CSV file
    std::ofstream csv_file("controller_output.csv");
    csv_file << "Step,Roll,Pitch,Yaw,RollCmd,PitchCmd,Throttle,RollCtrl,PitchCtrl,YawCtrl,"
             << "AltThrottle,FinalThrottle,M1,M2,M3,M4,Emergency" << std::endl;

    std::cout << "==========================================" << std::endl;
    std::cout << "DIAGNOSTIC TEST WITH KNOWN VALUES" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << std::endl;

    // ================================================================
    // TEST 1: Level drone - test each module individually
    // ================================================================
    std::cout << "Test 1: Level drone - Testing each module" << std::endl;
    std::cout << std::endl;

    // Reset state
    state.roll = fp32_t(0);
    state.pitch = fp32_t(0);
    state.yaw = fp32_t(0);
    state.ang_vel_x = fp32_t(0);
    state.ang_vel_y = fp32_t(0);
    state.ang_vel_z = fp32_t(0);

    generate_imu_data(state, sensors);

    // --- MODULE 1: KEYBOARD PROCESSOR ---
    std::cout << "[MODULE 1] KEYBOARD_PROCESSOR" << std::endl;
    std::cout << "  Input key: 119 ('w' = throttle up)" << std::endl;

    key_input_t test_key = key_input_t(119);  // 'w'
    process_keyboard_input(test_key, commands);

    std::cout << "  Output commands:" << std::endl;
    std::cout << "    throttle = " << to_double(commands.throttle) << std::endl;
    std::cout << "    roll_cmd = " << to_double(commands.roll_cmd) << " rad" << std::endl;
    std::cout << "    pitch_cmd = " << to_double(commands.pitch_cmd) << " rad" << std::endl;
    std::cout << "    yaw_cmd = " << to_double(commands.yaw_cmd) << " rad" << std::endl;
    std::cout << "    emergency_stop = " << (commands.emergency_stop ? "true" : "false") << std::endl;
    std::cout << std::endl;

    // Send more throttle commands to get to hover
    for (int i = 0; i < 9; i++) {
        process_keyboard_input(key_input_t(119), commands);
    }
    std::cout << "  After 10 throttle up commands:" << std::endl;
    std::cout << "    throttle = " << to_double(commands.throttle) << std::endl;
    std::cout << std::endl;

    // --- MODULE 2: COMPLEMENTARY FILTER ---
    std::cout << "[MODULE 2] COMPLEMENTARY_FILTER" << std::endl;
    std::cout << "  Input IMU:" << std::endl;
    std::cout << "    accel = [" << to_double(sensors.accel_x) << ", "
              << to_double(sensors.accel_y) << ", " << to_double(sensors.accel_z) << "]" << std::endl;
    std::cout << "    gyro = [" << to_double(sensors.gyro_x) << ", "
              << to_double(sensors.gyro_y) << ", " << to_double(sensors.gyro_z) << "]" << std::endl;
    std::cout << "  Input state (before): roll=" << to_double(state.roll)
              << ", pitch=" << to_double(state.pitch) << ", yaw=" << to_double(state.yaw) << std::endl;
    std::cout << "  Alpha = " << to_double(alpha) << std::endl;

    complementary_filter(sensors, state, alpha);

    std::cout << "  Output state (after): roll=" << to_double(state.roll)
              << ", pitch=" << to_double(state.pitch) << ", yaw=" << to_double(state.yaw) << std::endl;
    std::cout << std::endl;

    // --- MODULE 3: ATTITUDE CONTROLLER ---
    std::cout << "[MODULE 3] ATTITUDE_CONTROLLER" << std::endl;
    std::cout << "  Input state: roll=" << to_double(state.roll)
              << ", pitch=" << to_double(state.pitch) << ", yaw=" << to_double(state.yaw) << std::endl;
    std::cout << "  Input commands: roll_cmd=" << to_double(commands.roll_cmd)
              << ", pitch_cmd=" << to_double(commands.pitch_cmd)
              << ", yaw_cmd=" << to_double(commands.yaw_cmd) << std::endl;

    control_signal_t roll_ctrl, pitch_ctrl, yaw_ctrl, throttle_ctrl;
    attitude_controller(state, commands, roll_ctrl, pitch_ctrl, yaw_ctrl, throttle_ctrl);

    std::cout << "  Output control signals:" << std::endl;
    std::cout << "    roll_ctrl = " << to_double(roll_ctrl) << std::endl;
    std::cout << "    pitch_ctrl = " << to_double(pitch_ctrl) << std::endl;
    std::cout << "    yaw_ctrl = " << to_double(yaw_ctrl) << std::endl;
    std::cout << "    throttle_ctrl = " << to_double(throttle_ctrl) << std::endl;
    std::cout << std::endl;

    // --- MODULE 4: ALTITUDE CONTROLLER ---
    std::cout << "[MODULE 4] ALTITUDE_CONTROLLER" << std::endl;
    std::cout << "  Input state: pos_z=" << to_double(state.pos_z) << std::endl;
    std::cout << "  Input commands: throttle=" << to_double(commands.throttle) << std::endl;

    control_signal_t altitude_throttle;
    altitude_controller(state, commands, altitude_throttle, params);

    std::cout << "  Output altitude_throttle = " << to_double(altitude_throttle) << std::endl;
    std::cout << std::endl;

    // --- MODULE 5: MOTOR MIXER ---
    std::cout << "[MODULE 5] MOTOR_MIXER" << std::endl;
    control_signal_t final_throttle = (throttle_ctrl * control_signal_t(0.2)) +
                                      (altitude_throttle * control_signal_t(0.8));
    std::cout << "  Input throttle (combined) = " << to_double(final_throttle) << std::endl;
    std::cout << "  Input roll_ctrl = " << to_double(roll_ctrl) << std::endl;
    std::cout << "  Input pitch_ctrl = " << to_double(pitch_ctrl) << std::endl;
    std::cout << "  Input yaw_ctrl = " << to_double(yaw_ctrl) << std::endl;

    motor_mixer(final_throttle, roll_ctrl, pitch_ctrl, yaw_ctrl, motors);

    std::cout << "  Output motors:" << std::endl;
    std::cout << "    M1 (front_left) = " << to_double(motors.front_left) << std::endl;
    std::cout << "    M2 (front_right) = " << to_double(motors.front_right) << std::endl;
    std::cout << "    M3 (rear_left) = " << to_double(motors.rear_left) << std::endl;
    std::cout << "    M4 (rear_right) = " << to_double(motors.rear_right) << std::endl;
    std::cout << std::endl;

    // --- MODULE 6: SAFETY MONITOR ---
    std::cout << "[MODULE 6] SAFETY_MONITOR" << std::endl;
    std::cout << "  Input motors: [" << to_double(motors.front_left) << ", "
              << to_double(motors.front_right) << ", " << to_double(motors.rear_left)
              << ", " << to_double(motors.rear_right) << "]" << std::endl;
    std::cout << "  Input state: roll=" << to_double(state.roll) * 57.3 << " deg, pitch="
              << to_double(state.pitch) * 57.3 << " deg" << std::endl;

    bool emergency = false;
    safety_monitor(motors, state, emergency, params);

    std::cout << "  Output motors (after safety): [" << to_double(motors.front_left) << ", "
              << to_double(motors.front_right) << ", " << to_double(motors.rear_left)
              << ", " << to_double(motors.rear_right) << "]" << std::endl;
    std::cout << "  Emergency stop = " << (emergency ? "true" : "false") << std::endl;
    std::cout << std::endl;

    // --- MODULE 7: FLIGHT CONTROLLER (combines attitude + altitude + mixer + safety) ---
    std::cout << "[MODULE 7] FLIGHT_CONTROLLER (integrated test)" << std::endl;
    std::cout << "  This module internally calls:" << std::endl;
    std::cout << "    - attitude_controller" << std::endl;
    std::cout << "    - altitude_controller" << std::endl;
    std::cout << "    - motor_mixer" << std::endl;
    std::cout << "    - safety_monitor" << std::endl;

    MotorSpeeds motors_fc;
    flight_controller(state, commands, motors_fc, params);

    std::cout << "  Output motors from flight_controller:" << std::endl;
    std::cout << "    M1 = " << to_double(motors_fc.front_left) << std::endl;
    std::cout << "    M2 = " << to_double(motors_fc.front_right) << std::endl;
    std::cout << "    M3 = " << to_double(motors_fc.rear_left) << std::endl;
    std::cout << "    M4 = " << to_double(motors_fc.rear_right) << std::endl;
    std::cout << std::endl;

    std::cout << "=== END MODULE TEST ===" << std::endl;
    std::cout << std::endl;

    // ================================================================
    // TEST 2: Roll right command
    // ================================================================
    std::cout << "==========================================" << std::endl;
    std::cout << "Test 2: Roll right command (key 39)" << std::endl;
    std::cout << "==========================================" << std::endl;

    std::cout << "[MODULE 1] KEYBOARD_PROCESSOR" << std::endl;
    std::cout << "  Input key: 39 (right arrow = roll right)" << std::endl;
    process_keyboard_input(key_input_t(39), commands);
    std::cout << "  Output: roll_cmd = " << to_double(commands.roll_cmd) << " rad" << std::endl;
    std::cout << std::endl;

    std::cout << "[MODULE 3] ATTITUDE_CONTROLLER" << std::endl;
    attitude_controller(state, commands, roll_ctrl, pitch_ctrl, yaw_ctrl, throttle_ctrl);
    std::cout << "  Roll error = " << to_double(commands.roll_cmd) - to_double(state.roll) << " rad" << std::endl;
    std::cout << "  Roll control output = " << to_double(roll_ctrl) << std::endl;
    std::cout << std::endl;

    std::cout << "[MODULE 5] MOTOR_MIXER" << std::endl;
    altitude_controller(state, commands, altitude_throttle, params);
    final_throttle = (throttle_ctrl * control_signal_t(0.2)) + (altitude_throttle * control_signal_t(0.8));
    motor_mixer(final_throttle, roll_ctrl, pitch_ctrl, yaw_ctrl, motors);
    std::cout << "  Motors: [" << to_double(motors.front_left) << ", "
              << to_double(motors.front_right) << ", " << to_double(motors.rear_left)
              << ", " << to_double(motors.rear_right) << "]" << std::endl;
    double roll_diff = (to_double(motors.front_left) + to_double(motors.rear_left)) -
                       (to_double(motors.front_right) + to_double(motors.rear_right));
    std::cout << "  Roll differential (M1+M3)-(M2+M4) = " << roll_diff << std::endl;
    std::cout << std::endl;

    // ================================================================
    // MAIN TEST: 100 iterations using quadcopter_system (TOP FUNCTION)
    // ================================================================
    std::cout << "==========================================" << std::endl;
    std::cout << "MAIN TEST: 100 iterations using quadcopter_system" << std::endl;
    std::cout << "- Steps 0-29: Hover (no commands)" << std::endl;
    std::cout << "- Steps 30-59: Roll right (key 39)" << std::endl;
    std::cout << "- Steps 60-99: Pitch forward (key 38)" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << std::endl;

    // Reset everything
    state.roll = fp32_t(0);
    state.pitch = fp32_t(0);
    state.yaw = fp32_t(0);
    state.pos_z = fp32_t(10.0);
    state.ang_vel_x = fp32_t(0);
    state.ang_vel_y = fp32_t(0);
    state.ang_vel_z = fp32_t(0);

    commands.throttle = control_signal_t(0);
    commands.roll_cmd = control_signal_t(0);
    commands.pitch_cmd = control_signal_t(0);
    commands.yaw_cmd = control_signal_t(0);
    commands.emergency_stop = false;

    // Initialize throttle using quadcopter_system
    for (int i = 0; i < 10; i++) {
        generate_imu_data(state, sensors);
        imu_stream.write(sensors);
        key_stream.write(key_input_t(119));  // 'w' throttle up

        // CALL TOP-LEVEL FUNCTION (required for co-simulation)
        quadcopter_system(imu_stream, key_stream, state_stream, motor_stream, params);

        if (!state_stream.empty()) state_stream.read(fpga_state);
        if (!motor_stream.empty()) motor_stream.read(motors);
    }

    // Main loop - 100 iterations
    for (int step = 0; step < 100; step++) {
        // Determine key input
        key_input_t key_cmd;
        const char* phase_name;
        if (step < 30) {
            key_cmd = key_input_t(0);  // no command
            phase_name = "HOVER";
        } else if (step < 60) {
            key_cmd = key_input_t(39);  // roll right
            phase_name = "ROLL_RIGHT";
        } else {
            key_cmd = key_input_t(38);  // pitch forward
            phase_name = "PITCH_FWD";
        }

        // Generate IMU
        generate_imu_data(state, sensors);

        // Write to streams
        imu_stream.write(sensors);
        key_stream.write(key_cmd);

        // CALL TOP-LEVEL FUNCTION (required for co-simulation)
        quadcopter_system(imu_stream, key_stream, state_stream, motor_stream, params);

        // Read outputs from FPGA
        if (!state_stream.empty()) state_stream.read(fpga_state);
        if (!motor_stream.empty()) motor_stream.read(motors);

        // Also call individual modules for detailed debug output
        process_keyboard_input(key_cmd, commands);
        complementary_filter(sensors, state, alpha);
        attitude_controller(state, commands, roll_ctrl, pitch_ctrl, yaw_ctrl, throttle_ctrl);
        altitude_controller(state, commands, altitude_throttle, params);
        final_throttle = (throttle_ctrl * control_signal_t(0.2)) +
                        (altitude_throttle * control_signal_t(0.8));
        motor_mixer(final_throttle, roll_ctrl, pitch_ctrl, yaw_ctrl, motors);
        safety_monitor(motors, state, emergency, params);

        // Apply physics
        apply_physics(motors, state);

        // Print detailed debug every 10 steps
        bool print_detail = (step % 10 == 0) || (step == 30) || (step == 60);

        if (print_detail) {
            std::cout << "========== Step " << step << "/100 [" << phase_name << "] ==========" << std::endl;

            std::cout << "[KEYBOARD_PROCESSOR] key=" << key_cmd.to_int() << std::endl;
            std::cout << "  -> throttle=" << to_double(commands.throttle)
                      << ", roll_cmd=" << to_double(commands.roll_cmd)
                      << ", pitch_cmd=" << to_double(commands.pitch_cmd) << std::endl;

            std::cout << "[COMPLEMENTARY_FILTER] IMU: accel=[" << to_double(sensors.accel_x)
                      << "," << to_double(sensors.accel_y) << "," << to_double(sensors.accel_z)
                      << "], gyro=[" << to_double(sensors.gyro_x) << "," << to_double(sensors.gyro_y)
                      << "," << to_double(sensors.gyro_z) << "]" << std::endl;
            std::cout << "  -> roll=" << to_double(state.roll) << " rad ("
                      << to_double(state.roll)*57.3 << " deg), pitch=" << to_double(state.pitch)
                      << " rad, yaw=" << to_double(state.yaw) << " rad" << std::endl;

            std::cout << "[ATTITUDE_CONTROLLER] setpoints: roll=" << to_double(commands.roll_cmd)
                      << ", pitch=" << to_double(commands.pitch_cmd) << std::endl;
            std::cout << "  -> roll_ctrl=" << to_double(roll_ctrl) << ", pitch_ctrl="
                      << to_double(pitch_ctrl) << ", yaw_ctrl=" << to_double(yaw_ctrl) << std::endl;

            std::cout << "[ALTITUDE_CONTROLLER] pos_z=" << to_double(state.pos_z)
                      << ", throttle_cmd=" << to_double(commands.throttle) << std::endl;
            std::cout << "  -> altitude_throttle=" << to_double(altitude_throttle) << std::endl;

            std::cout << "[MOTOR_MIXER] final_throttle=" << to_double(final_throttle) << std::endl;
            std::cout << "  -> M1=" << to_double(motors.front_left) << ", M2="
                      << to_double(motors.front_right) << ", M3=" << to_double(motors.rear_left)
                      << ", M4=" << to_double(motors.rear_right) << std::endl;

            roll_diff = (to_double(motors.front_left) + to_double(motors.rear_left)) -
                        (to_double(motors.front_right) + to_double(motors.rear_right));
            double pitch_diff = (to_double(motors.rear_left) + to_double(motors.rear_right)) -
                               (to_double(motors.front_left) + to_double(motors.front_right));
            std::cout << "  -> roll_diff=" << roll_diff << ", pitch_diff=" << pitch_diff << std::endl;

            std::cout << "[SAFETY_MONITOR] checking limits..." << std::endl;
            std::cout << "  -> emergency=" << (emergency ? "TRUE" : "false") << std::endl;

            std::cout << "[FPGA OUTPUT] roll=" << to_double(fpga_state.roll)
                      << ", pitch=" << to_double(fpga_state.pitch)
                      << ", yaw=" << to_double(fpga_state.yaw) << std::endl;
            std::cout << std::endl;
        }

        // Write to CSV
        csv_file << step << "," << to_double(state.roll) << "," << to_double(state.pitch) << ","
                 << to_double(state.yaw) << "," << to_double(commands.roll_cmd) << ","
                 << to_double(commands.pitch_cmd) << "," << to_double(commands.throttle) << ","
                 << to_double(roll_ctrl) << "," << to_double(pitch_ctrl) << "," << to_double(yaw_ctrl) << ","
                 << to_double(altitude_throttle) << "," << to_double(final_throttle) << ","
                 << to_double(motors.front_left) << "," << to_double(motors.front_right) << ","
                 << to_double(motors.rear_left) << "," << to_double(motors.rear_right) << ","
                 << (emergency ? 1 : 0) << std::endl;
    }

    csv_file.close();

    std::cout << "==========================================" << std::endl;
    std::cout << "TEST COMPLETED SUCCESSFULLY!" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "FINAL STATE:" << std::endl;
    std::cout << "  Roll = " << to_double(state.roll) << " rad (" << to_double(state.roll)*57.3 << " deg)" << std::endl;
    std::cout << "  Pitch = " << to_double(state.pitch) << " rad (" << to_double(state.pitch)*57.3 << " deg)" << std::endl;
    std::cout << "  Yaw = " << to_double(state.yaw) << " rad" << std::endl;
    std::cout << "  Motors = [" << to_double(motors.front_left) << ", " << to_double(motors.front_right)
              << ", " << to_double(motors.rear_left) << ", " << to_double(motors.rear_right) << "]" << std::endl;
    std::cout << std::endl;
    std::cout << "CSV saved to: controller_output.csv" << std::endl;

    return 0;
}
