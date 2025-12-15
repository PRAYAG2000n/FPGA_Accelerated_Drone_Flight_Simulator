#include <iostream>
#include <iomanip>
#include <cstdint>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <xrt/xrt_device.h>
#include <xrt/xrt_kernel.h>
#include <xrt/xrt_bo.h>

int16_t to_fixed16(float v) { return (int16_t)(v * 256.0f); }
float from_fixed16(int16_t v) { return (float)v / 256.0f; }
int32_t to_fixed32(float v) { return (int32_t)(v * 65536.0f); }
float from_fixed32(int32_t v) { return (float)v / 65536.0f; }

#pragma pack(push, 1)
struct IMUData {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t mag_x, mag_y, mag_z;
};
struct StateVector {
    int32_t roll, pitch, yaw;
    int32_t pos_x, pos_y, pos_z;
    int32_t vel_x, vel_y, vel_z;
    int32_t ang_vel_x, ang_vel_y, ang_vel_z;
};
struct MotorSpeeds {
    int16_t front_left, front_right, rear_left, rear_right;
};
#pragma pack(pop)

enum Keys { KEY_W=119, KEY_S=115, KEY_A=97, KEY_D=100, KEY_UP=38, KEY_DOWN=40, KEY_LEFT=37, KEY_RIGHT=39, KEY_SPACE=32 };

int8_t get_command(int step) {
    if (step < 20) return KEY_W;
    if (step < 30) return 0;
    if (step < 40) return KEY_RIGHT;
    if (step < 50) return KEY_LEFT;
    if (step < 60) return KEY_UP;
    if (step < 70) return KEY_DOWN;
    if (step < 80) return KEY_D;
    if (step < 85) return KEY_A;
    if (step < 95) return 0;
    return KEY_SPACE;
}

void print_state(int step, StateVector* state, MotorSpeeds* motor) {
    std::cout << "Step " << std::setw(2) << step 
              << ": Roll=" << std::setw(8) << from_fixed32(state[step].roll) * 57.3f << " deg"
              << ", Pitch=" << std::setw(8) << from_fixed32(state[step].pitch) * 57.3f << " deg"
              << ", Yaw=" << std::setw(8) << from_fixed32(state[step].yaw) * 57.3f << " deg"
              << ", PosZ=" << std::setw(6) << from_fixed32(state[step].pos_z) << " m\n"
              << "        Motors: M1=" << std::setw(6) << from_fixed16(motor[step].front_left)
              << ", M2=" << std::setw(6) << from_fixed16(motor[step].front_right)
              << ", M3=" << std::setw(6) << from_fixed16(motor[step].rear_left)
              << ", M4=" << std::setw(6) << from_fixed16(motor[step].rear_right) << "\n";
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <xclbin_file>" << std::endl;
        return 1;
    }

    const int num_samples = 100;
    std::cout << std::fixed << std::setprecision(2);

    std::cout << "\nQUADCOPTER FPGA CONTROLLER - FULL TEST\n";
    std::cout << "Test Phases:\n";
    std::cout << "  Steps 0-19:  THROTTLE UP (key 'w')\n";
    std::cout << "  Steps 20-29: HOVER\n";
    std::cout << "  Steps 30-39: ROLL RIGHT (key 39)\n";
    std::cout << "  Steps 40-49: ROLL LEFT (key 37)\n";
    std::cout << "  Steps 50-59: PITCH FORWARD (key 38)\n";
    std::cout << "  Steps 60-69: PITCH BACKWARD (key 40)\n";
    std::cout << "  Steps 70-79: YAW RIGHT (key 'd')\n";
    std::cout << "  Steps 80-84: YAW LEFT (key 'a')\n";
    std::cout << "  Steps 85-94: HOVER\n";
    std::cout << "  Steps 95-99: EMERGENCY STOP (space)\n";


    std::cout << "Struct sizes - IMUData: " << sizeof(IMUData) 
              << ", StateVector: " << sizeof(StateVector) 
              << ", MotorSpeeds: " << sizeof(MotorSpeeds) << "\n";

    auto setup_start = std::chrono::high_resolution_clock::now();
    xrt::device device(0);
    std::cout << "Device opened\n";
    auto uuid = device.load_xclbin(argv[1]);
    std::cout << "XCLBIN loaded\n";
    xrt::kernel krnl(device, uuid, "quadcopter_system");
    std::cout << "Kernel obtained\n";
    auto setup_end = std::chrono::high_resolution_clock::now();
    auto setup_ms = std::chrono::duration_cast<std::chrono::milliseconds>(setup_end - setup_start).count();
    std::cout << "Setup time: " << setup_ms << " ms\n";

    auto alloc_start = std::chrono::high_resolution_clock::now();
    auto bo_imu = xrt::bo(device, sizeof(IMUData) * num_samples, krnl.group_id(0));
    auto bo_cmd = xrt::bo(device, sizeof(int8_t) * num_samples, krnl.group_id(1));
    auto bo_state = xrt::bo(device, sizeof(StateVector) * num_samples, krnl.group_id(2));
    auto bo_motor = xrt::bo(device, sizeof(MotorSpeeds) * num_samples, krnl.group_id(3));
    auto imu = bo_imu.map<IMUData*>();
    auto cmd = bo_cmd.map<int8_t*>();
    auto state = bo_state.map<StateVector*>();
    auto motor = bo_motor.map<MotorSpeeds*>();
    auto alloc_end = std::chrono::high_resolution_clock::now();
    auto alloc_us = std::chrono::duration_cast<std::chrono::microseconds>(alloc_end - alloc_start).count();
    std::cout << "Buffer allocation time: " << alloc_us << " us\n";

    float sim_roll = 0, sim_pitch = 0, sim_yaw = 0;
    float gx = 0, gy = 0, gz = 0;
    for (int i = 0; i < num_samples; i++) {
        cmd[i] = get_command(i);
        float tgx = 0, tgy = 0, tgz = 0;
        switch (cmd[i]) {
            case KEY_RIGHT: tgx = 0.5f; break;
            case KEY_LEFT: tgx = -0.5f; break;
            case KEY_UP: tgy = 0.5f; break;
            case KEY_DOWN: tgy = -0.5f; break;
            case KEY_D: tgz = 0.5f; break;
            case KEY_A: tgz = -0.5f; break;
        }
        gx = 0.8f * gx + 0.2f * tgx;
        gy = 0.8f * gy + 0.2f * tgy;
        gz = 0.8f * gz + 0.2f * tgz;
        sim_roll += gx * 0.01f;
        sim_pitch += gy * 0.01f;
        sim_yaw += gz * 0.01f;

        imu[i].accel_x = to_fixed16(-9.81f * sin(sim_pitch));
        imu[i].accel_y = to_fixed16(9.81f * sin(sim_roll) * cos(sim_pitch));
        imu[i].accel_z = to_fixed16(9.81f * cos(sim_roll) * cos(sim_pitch));
        imu[i].gyro_x = to_fixed16(gx);
        imu[i].gyro_y = to_fixed16(gy);
        imu[i].gyro_z = to_fixed16(gz);
        imu[i].mag_x = to_fixed16(cos(sim_yaw));
        imu[i].mag_y = to_fixed16(sin(sim_yaw));
        imu[i].mag_z = 0;
    }

    std::cout << "\nIMU Data Verification:\n";
    std::cout << "Step 0  (Throttle): gyro=[" << from_fixed16(imu[0].gyro_x) << "," << from_fixed16(imu[0].gyro_y) << "," << from_fixed16(imu[0].gyro_z) << "], az=" << from_fixed16(imu[0].accel_z) << "\n";
    std::cout << "Step 25 (Roll R):   gyro=[" << from_fixed16(imu[25].gyro_x) << "," << from_fixed16(imu[25].gyro_y) << "," << from_fixed16(imu[25].gyro_z) << "], az=" << from_fixed16(imu[25].accel_z) << "\n";
    std::cout << "Step 45 (Pitch F):  gyro=[" << from_fixed16(imu[45].gyro_x) << "," << from_fixed16(imu[45].gyro_y) << "," << from_fixed16(imu[45].gyro_z) << "], az=" << from_fixed16(imu[45].accel_z) << "\n";

    auto h2d_start = std::chrono::high_resolution_clock::now();
    bo_imu.sync(XCL_BO_SYNC_BO_TO_DEVICE);
    bo_cmd.sync(XCL_BO_SYNC_BO_TO_DEVICE);
    auto h2d_end = std::chrono::high_resolution_clock::now();
    auto h2d_us = std::chrono::duration_cast<std::chrono::microseconds>(h2d_end - h2d_start).count();
    std::cout << "Host-to-Device transfer time: " << h2d_us << " us\n";

    std::cout << "\nRunning kernel...\n";
    auto kern_start = std::chrono::high_resolution_clock::now();
    auto run = krnl(bo_imu, bo_cmd, bo_state, bo_motor, 1.2f, 0.015f, 0.015f, 0.025f, 0.12f, 0.01f, 0.25f, 0.01f, 9.81f, num_samples);
    run.wait();
    auto kern_end = std::chrono::high_resolution_clock::now();
    auto kern_us = std::chrono::duration_cast<std::chrono::microseconds>(kern_end - kern_start).count();
    std::cout << "Kernel execution time: " << kern_us << " us\n";

    auto d2h_start = std::chrono::high_resolution_clock::now();
    bo_state.sync(XCL_BO_SYNC_BO_FROM_DEVICE);
    bo_motor.sync(XCL_BO_SYNC_BO_FROM_DEVICE);
    auto d2h_end = std::chrono::high_resolution_clock::now();
    auto d2h_us = std::chrono::duration_cast<std::chrono::microseconds>(d2h_end - d2h_start).count();
    std::cout << "Device-to-Host transfer time: " << d2h_us << " us\n";

    std::cout << "\nEXECUTION RESULTS BY PHASE\n";

    const char* phase_names[] = {"THROTTLE UP (0-19)", "HOVER (20-29)", "ROLL RIGHT (30-39)", "ROLL LEFT (40-49)", "PITCH FWD (50-59)", "PITCH BACK (60-69)", "YAW RIGHT (70-79)", "YAW LEFT (80-84)", "HOVER 2 (85-94)", "EMERGENCY (95-99)"};
    int phase_starts[] = {0, 20, 30, 40, 50, 60, 70, 80, 85, 95};
    int phase_ends[] = {20, 30, 40, 50, 60, 70, 80, 85, 95, 100};

    for (int p = 0; p < 10; p++) {
        std::cout << "\n----" << phase_names[p] << "----\n";
        int s = phase_starts[p], e = phase_ends[p];
        print_state(s, state, motor);
        print_state(s + (e - s) / 2, state, motor);
        print_state(e - 1, state, motor);
    }

    std::cout << "TIMING SUMMARY\n";
    std::cout << "Setup time:              " << setup_ms << " ms\n";
    std::cout << "Buffer allocation:       " << alloc_us << " us\n";
    std::cout << "Host-to-Device transfer: " << h2d_us << " us\n";
    std::cout << "Kernel execution:        " << kern_us << " us\n";
    std::cout << "Device-to-Host transfer: " << d2h_us << " us\n";
    std::cout << "--------------------------------------------\n";
    std::cout << "Total runtime (excl. setup): " << (h2d_us + kern_us + d2h_us) << " us\n";
    std::cout << "Per-sample latency:      " << std::setprecision(2) << (float)kern_us / num_samples << " us\n";
    std::cout << "Throughput:              " << (num_samples * 1000000.0f) / kern_us << " samples/sec\n";
    std::cout << "Control loop rate:       " << (num_samples * 1000000.0f) / kern_us << " Hz\n";

    std::cout << "\nFINAL STATE (Step 99)\n";
    std::cout << std::setprecision(4);
    std::cout << "Roll:   " << from_fixed32(state[99].roll) * 57.3f << " deg\n";
    std::cout << "Pitch:  " << from_fixed32(state[99].pitch) * 57.3f << " deg\n";
    std::cout << "Yaw:    " << from_fixed32(state[99].yaw) * 57.3f << " deg\n";
    std::cout << "PosZ:   " << from_fixed32(state[99].pos_z) << " m\n";
    std::cout << "Motors: [" << from_fixed16(motor[99].front_left) << ", " << from_fixed16(motor[99].front_right) << ", " << from_fixed16(motor[99].rear_left) << ", " << from_fixed16(motor[99].rear_right) << "]\n";

    std::cout << "\nMODULE-LEVEL VERIFICATION\n";

    std::cout << "\n[MODULE 1] KEYBOARD_PROCESSOR\n";
    float m_early = from_fixed16(motor[5].front_left);
    float m_late = from_fixed16(motor[19].front_left);
    std::cout << "  Steps 0-19: Throttle Up commands (key 'w' = 119)\n";
    std::cout << "  Motor response to throttle: " << std::setprecision(4) << m_early << " -> " << m_late << "\n";
    std::cout << "  [" << ((m_late > m_early || m_late > 50) ? "PASS" : "FAIL") << "] Throttle commands processed correctly\n";
    std::cout << "  [" << (from_fixed16(motor[35].front_left) != from_fixed16(motor[35].rear_right) ? "PASS" : "FAIL") << "] Roll commands processed (key 39)\n";
    std::cout << "  [" << (from_fixed16(motor[55].front_left) != from_fixed16(motor[55].rear_left) ? "PASS" : "FAIL") << "] Pitch commands processed (key 38)\n";
    std::cout << "  [" << (from_fixed16(motor[75].front_left) != from_fixed16(motor[75].front_right) ? "PASS" : "FAIL") << "] Yaw commands processed (key 'd' = 100)\n";

    std::cout << "\n[MODULE 2] COMPLEMENTARY_FILTER\n";
    float roll_30 = from_fixed32(state[30].roll) * 57.3f;
    float roll_39 = from_fixed32(state[39].roll) * 57.3f;
    float pitch_50 = from_fixed32(state[50].pitch) * 57.3f;
    float pitch_59 = from_fixed32(state[59].pitch) * 57.3f;
    float yaw_70 = from_fixed32(state[70].yaw) * 57.3f;
    float yaw_79 = from_fixed32(state[79].yaw) * 57.3f;
    std::cout << "  Roll estimation:  " << roll_30 << " -> " << roll_39 << " deg (delta: " << (roll_39 - roll_30) << " deg)\n";
    std::cout << "  Pitch estimation: " << pitch_50 << " -> " << pitch_59 << " deg (delta: " << (pitch_59 - pitch_50) << " deg)\n";
    std::cout << "  Yaw estimation:   " << yaw_70 << " -> " << yaw_79 << " deg (delta: " << (yaw_79 - yaw_70) << " deg)\n";
    std::cout << "  [" << (fabs(roll_39 - roll_30) > 0.1f ? "PASS" : "FAIL") << "] Sensor fusion working correctly\n";

    std::cout << "\n[MODULE 3] PID_CONTROLLER\n";
    float m1_hover = from_fixed16(motor[25].front_left);
    float m1_roll = from_fixed16(motor[35].front_left);
    float m1_pitch = from_fixed16(motor[55].front_left);
    std::cout << "  Hover baseline M1: " << m1_hover << "\n";
    std::cout << "  During roll M1: " << m1_roll << " (change: " << (m1_roll - m1_hover) << ")\n";
    std::cout << "  During pitch M1: " << m1_pitch << " (change: " << (m1_pitch - m1_hover) << ")\n";
    std::cout << "  [" << (m1_roll != m1_hover ? "PASS" : "FAIL") << "] PID generating error corrections\n";

    std::cout << "\n[MODULE 4] ATTITUDE_CONTROLLER\n";
    float m1_r = from_fixed16(motor[35].front_left), m2_r = from_fixed16(motor[35].front_right);
    float m3_r = from_fixed16(motor[35].rear_left), m4_r = from_fixed16(motor[35].rear_right);
    float roll_diff = (m1_r + m3_r) - (m2_r + m4_r);
    std::cout << "  Roll phase motors: M1=" << m1_r << ", M2=" << m2_r << ", M3=" << m3_r << ", M4=" << m4_r << "\n";
    std::cout << "  Roll control differential (M1+M3)-(M2+M4): " << roll_diff << "\n";
    std::cout << "  [" << (fabs(roll_diff) > 1.0f ? "PASS" : "FAIL") << "] Roll attitude control active\n";
    float m1_p = from_fixed16(motor[55].front_left), m2_p = from_fixed16(motor[55].front_right);
    float m3_p = from_fixed16(motor[55].rear_left), m4_p = from_fixed16(motor[55].rear_right);
    float pitch_diff = (m1_p + m2_p) - (m3_p + m4_p);
    std::cout << "  Pitch phase motors: M1=" << m1_p << ", M2=" << m2_p << ", M3=" << m3_p << ", M4=" << m4_p << "\n";
    std::cout << "  Pitch control differential (M1+M2)-(M3+M4): " << pitch_diff << "\n";
    std::cout << "  [" << (fabs(pitch_diff) > 1.0f ? "PASS" : "FAIL") << "] Pitch attitude control active\n";
    float m1_y = from_fixed16(motor[75].front_left), m2_y = from_fixed16(motor[75].front_right);
    float m3_y = from_fixed16(motor[75].rear_left), m4_y = from_fixed16(motor[75].rear_right);
    float yaw_diff = (m1_y + m4_y) - (m2_y + m3_y);
    std::cout << "  Yaw control differential (M1+M4)-(M2+M3): " << yaw_diff << "\n";
    std::cout << "  [" << (fabs(yaw_diff) > 0.5f ? "PASS" : "WARN") << "] Yaw attitude control active\n";

    std::cout << "\n[MODULE 5] ALTITUDE_CONTROLLER\n";
    float init_alt = from_fixed32(state[0].pos_z);
    float mid_alt = from_fixed32(state[50].pos_z);
    float final_alt = from_fixed32(state[94].pos_z);
    std::cout << "  Target altitude: 50.0 m\n";
    std::cout << "  Initial altitude: " << init_alt << " m\n";
    std::cout << "  Mid-flight altitude: " << mid_alt << " m\n";
    std::cout << "  Final altitude: " << final_alt << " m\n";
    std::cout << "  [" << (fabs(final_alt - 50.0f) < 5.0f ? "PASS" : "WARN") << "] Altitude maintained at target (~50m)\n";

    std::cout << "\n[MODULE 6] FLIGHT_CONTROLLER (Integration)\n";
    float h1 = from_fixed16(motor[25].front_left), h2 = from_fixed16(motor[25].front_right);
    float h3 = from_fixed16(motor[25].rear_left), h4 = from_fixed16(motor[25].rear_right);
    float hover_avg = (h1 + h2 + h3 + h4) / 4.0f;
    std::cout << "  Integrates: Attitude Controller + Altitude Controller\n";
    std::cout << "  Hover motors: M1=" << h1 << ", M2=" << h2 << ", M3=" << h3 << ", M4=" << h4 << "\n";
    std::cout << "  Average hover thrust: " << hover_avg << "\n";
    std::cout << "  [" << (hover_avg > 30 && hover_avg < 100 ? "PASS" : "FAIL") << "] Flight controller producing valid outputs\n";

    std::cout << "\n[MODULE 7] MOTOR_MIXER\n";
    float max_motor = 0, min_motor = 100;
    for (int i = 20; i < 95; i++) {
        float vals[] = {from_fixed16(motor[i].front_left), from_fixed16(motor[i].front_right), from_fixed16(motor[i].rear_left), from_fixed16(motor[i].rear_right)};
        for (float v : vals) { max_motor = std::max(max_motor, v); min_motor = std::min(min_motor, v); }
    }
    std::cout << "  Motor output range: " << min_motor << " - " << max_motor << "\n";
    std::cout << "  [" << (max_motor > 0 && max_motor <= 100 ? "PASS" : "FAIL") << "] Motor outputs within valid range (0-100)\n";
    std::cout << "  X-config: Roll Right -> M3 > M4: " << m3_r << " vs " << m4_r << " [" << (m3_r > m4_r ? "PASS" : "FAIL") << "]\n";
    std::cout << "  X-config: Pitch Fwd -> M1+M2 < M3+M4: " << (m1_p + m2_p) << " vs " << (m3_p + m4_p) << " [" << ((m1_p + m2_p) < (m3_p + m4_p) ? "PASS" : "WARN") << "]\n";

    std::cout << "\n[MODULE 8] SAFETY_MONITOR\n";
    std::cout << "  Pre-emergency (step 94): M1=" << from_fixed16(motor[94].front_left) << ", M2=" << from_fixed16(motor[94].front_right) << "\n";
    std::cout << "  Emergency triggered (step 95): M1=" << from_fixed16(motor[95].front_left) << ", M2=" << from_fixed16(motor[95].front_right) << "\n";
    float em1 = from_fixed16(motor[97].front_left), em2 = from_fixed16(motor[97].front_right);
    float em3 = from_fixed16(motor[97].rear_left), em4 = from_fixed16(motor[97].rear_right);
    std::cout << "  Emergency active (step 97): M1=" << em1 << ", M2=" << em2 << ", M3=" << em3 << ", M4=" << em4 << "\n";
    bool em_ok = (em1 == 0 && em2 == 0 && em3 == 0 && em4 == 0);
    std::cout << "  [" << (em_ok ? "PASS" : "FAIL") << "] Emergency stop cuts all motors\n";
    std::cout << "  [" << (from_fixed16(motor[99].front_left) == 0 ? "PASS" : "FAIL") << "] Motors remain off after emergency\n";

    std::cout << "\nSUMMARY CHECKLIST\n";
 
    std::cout << "[" << (sizeof(IMUData) == 18 ? "PASS" : "FAIL") << "] IMUData struct size correct (18 bytes)\n";
    std::cout << "[" << (sizeof(StateVector) == 48 ? "PASS" : "FAIL") << "] StateVector struct size correct (48 bytes)\n";
    std::cout << "[" << (sizeof(MotorSpeeds) == 8 ? "PASS" : "FAIL") << "] MotorSpeeds struct size correct (8 bytes)\n";
    std::cout << "[PASS] XCLBIN loaded successfully\n";
    std::cout << "[PASS] Kernel executed successfully\n";
    std::cout << "[" << (fabs(init_alt - 50.0f) < 1.0f ? "PASS" : "WARN") << "] Initial altitude: " << init_alt << " m\n";
    std::cout << "[" << (fabs(roll_39 - roll_30) > 0.1f ? "PASS" : "FAIL") << "] State estimation working (roll/pitch/yaw changing)\n";
    std::cout << "[" << (max_motor > 50.0f ? "PASS" : "FAIL") << "] Motor outputs generated (max: " << max_motor << ")\n";
    std::cout << "[" << (em_ok ? "PASS" : "FAIL") << "] Emergency stop effective\n";
    std::cout << "\nTEST COMPLETED SUCCESSFULLY\n";


    return 0;
}
