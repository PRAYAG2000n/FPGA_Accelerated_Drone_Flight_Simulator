#include <iostream>
#include <iomanip>
#include <cstdint>
#include <cstring>
#include <xrt/xrt_device.h>
#include <xrt/xrt_kernel.h>
#include <xrt/xrt_bo.h>

// ============================================================================
// FIXED-POINT CONVERSION UTILITIES
// ap_fixed<16,8>: 8 integer bits, 8 fractional bits (range: -128 to 127.996)
// ap_fixed<32,16>: 16 integer bits, 16 fractional bits (range: -32768 to 32767.999)
// ============================================================================
int16_t to_fixed16(float val) {
    return (int16_t)(val * 256.0f);
}

float from_fixed16(int16_t val) {
    return (float)val / 256.0f;
}

int32_t to_fixed32(float val) {
    return (int32_t)(val * 65536.0f);
}

float from_fixed32(int32_t val) {
    return (float)val / 65536.0f;
}

// ============================================================================
// DATA STRUCTURES (Must match HLS exactly)
// ============================================================================
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
    int16_t front_left;
    int16_t front_right;
    int16_t rear_left;
    int16_t rear_right;
};
#pragma pack(pop)

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================
void print_separator(const char* title) {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << title << std::endl;
    std::cout << std::string(60, '=') << std::endl;
}

void print_imu(const IMUData& imu, int sample) {
    std::cout << "  IMU[" << sample << "]: "
              << "Accel=(" << from_fixed16(imu.accel_x) << ", "
              << from_fixed16(imu.accel_y) << ", "
              << from_fixed16(imu.accel_z) << ") "
              << "Gyro=(" << from_fixed16(imu.gyro_x) << ", "
              << from_fixed16(imu.gyro_y) << ", "
              << from_fixed16(imu.gyro_z) << ")" << std::endl;
}

void print_state(const StateVector& state, int sample) {
    std::cout << "  State[" << sample << "]: "
              << "Roll=" << std::setw(8) << from_fixed32(state.roll) << " "
              << "Pitch=" << std::setw(8) << from_fixed32(state.pitch) << " "
              << "Yaw=" << std::setw(8) << from_fixed32(state.yaw) << std::endl;
    std::cout << "             "
              << "PosX=" << std::setw(8) << from_fixed32(state.pos_x) << " "
              << "PosY=" << std::setw(8) << from_fixed32(state.pos_y) << " "
              << "PosZ=" << std::setw(8) << from_fixed32(state.pos_z) << std::endl;
    std::cout << "             "
              << "VelX=" << std::setw(8) << from_fixed32(state.vel_x) << " "
              << "VelY=" << std::setw(8) << from_fixed32(state.vel_y) << " "
              << "VelZ=" << std::setw(8) << from_fixed32(state.vel_z) << std::endl;
}

void print_motors(const MotorSpeeds& motors, int sample) {
    std::cout << "  Motors[" << sample << "]: "
              << "FL=" << std::setw(8) << from_fixed16(motors.front_left) << " "
              << "FR=" << std::setw(8) << from_fixed16(motors.front_right) << " "
              << "RL=" << std::setw(8) << from_fixed16(motors.rear_left) << " "
              << "RR=" << std::setw(8) << from_fixed16(motors.rear_right)
              << " (raw: " << motors.front_left << ", " << motors.front_right
              << ", " << motors.rear_left << ", " << motors.rear_right << ")" << std::endl;
}

void print_full_sample(const IMUData& imu, int8_t cmd, const StateVector& state, 
                       const MotorSpeeds& motors, int sample, const char* phase) {
    std::cout << "\n--- Sample " << sample << " [" << phase << "] ---" << std::endl;
    std::cout << "  Command: " << (int)cmd;
    if (cmd == 'w') std::cout << " ('w' = throttle up)";
    else if (cmd == 's') std::cout << " ('s' = throttle down)";
    else if (cmd == 39) std::cout << " (roll right)";
    else if (cmd == 37) std::cout << " (roll left)";
    else if (cmd == 38) std::cout << " (pitch forward)";
    else if (cmd == 40) std::cout << " (pitch back)";
    else if (cmd == 0) std::cout << " (neutral)";
    std::cout << std::endl;
    print_imu(imu, sample);
    print_state(state, sample);
    print_motors(motors, sample);
}

// ============================================================================
// MAIN
// ============================================================================
int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <xclbin>" << std::endl;
        return 1;
    }

    print_separator("QUADCOPTER FPGA FLIGHT CONTROLLER TEST");
    
    std::cout << "\nStruct sizes:" << std::endl;
    std::cout << "  IMUData:     " << sizeof(IMUData) << " bytes (expected: 18)" << std::endl;
    std::cout << "  StateVector: " << sizeof(StateVector) << " bytes (expected: 48)" << std::endl;
    std::cout << "  MotorSpeeds: " << sizeof(MotorSpeeds) << " bytes (expected: 8)" << std::endl;

    const int num_samples = 100;

    // ========================================================================
    // DEVICE SETUP
    // ========================================================================
    print_separator("INITIALIZING FPGA");
    
    auto device = xrt::device(0);
    std::cout << "Device opened successfully" << std::endl;

    auto uuid = device.load_xclbin(argv[1]);
    std::cout << "XCLBIN loaded: " << argv[1] << std::endl;

    auto kernel = xrt::kernel(device, uuid, "quadcopter_system");
    std::cout << "Kernel 'quadcopter_system' obtained" << std::endl;

    // ========================================================================
    // BUFFER ALLOCATION
    // ========================================================================
    print_separator("ALLOCATING BUFFERS");
    
    auto bo_imu = xrt::bo(device, num_samples * sizeof(IMUData), kernel.group_id(0));
    auto bo_cmd = xrt::bo(device, num_samples * sizeof(int8_t), kernel.group_id(1));
    auto bo_state = xrt::bo(device, num_samples * sizeof(StateVector), kernel.group_id(2));
    auto bo_motor = xrt::bo(device, num_samples * sizeof(MotorSpeeds), kernel.group_id(3));

    std::cout << "  IMU buffer:   " << num_samples * sizeof(IMUData) << " bytes" << std::endl;
    std::cout << "  CMD buffer:   " << num_samples * sizeof(int8_t) << " bytes" << std::endl;
    std::cout << "  State buffer: " << num_samples * sizeof(StateVector) << " bytes" << std::endl;
    std::cout << "  Motor buffer: " << num_samples * sizeof(MotorSpeeds) << " bytes" << std::endl;

    auto imu_data = bo_imu.map<IMUData*>();
    auto cmd_data = bo_cmd.map<int8_t*>();
    auto state_data = bo_state.map<StateVector*>();
    auto motor_data = bo_motor.map<MotorSpeeds*>();

    // ========================================================================
    // TEST DATA INITIALIZATION
    // ========================================================================
    print_separator("INITIALIZING TEST DATA");

    // Clear all buffers
    memset(imu_data, 0, num_samples * sizeof(IMUData));
    memset(cmd_data, 0, num_samples * sizeof(int8_t));
    memset(state_data, 0, num_samples * sizeof(StateVector));
    memset(motor_data, 0, num_samples * sizeof(MotorSpeeds));

    // Default IMU: level drone with gravity on Z
    for (int i = 0; i < num_samples; i++) {
        imu_data[i].accel_x = to_fixed16(0.0f);
        imu_data[i].accel_y = to_fixed16(0.0f);
        imu_data[i].accel_z = to_fixed16(9.81f);
        imu_data[i].gyro_x = to_fixed16(0.0f);
        imu_data[i].gyro_y = to_fixed16(0.0f);
        imu_data[i].gyro_z = to_fixed16(0.0f);
        imu_data[i].mag_x = to_fixed16(1.0f);
        imu_data[i].mag_y = to_fixed16(0.0f);
        imu_data[i].mag_z = to_fixed16(0.0f);
    }

    std::cout << "\nTest Sequence:" << std::endl;
    
    // Phase 1: Aggressive throttle up (0-29)
    std::cout << "  [0-29]  THROTTLE UP - Building thrust" << std::endl;
    for (int i = 0; i < 30; i++) {
        cmd_data[i] = 'w';
    }

    // Phase 2: Maintain hover (30-49)
    std::cout << "  [30-49] HOVER - Maintaining altitude" << std::endl;
    for (int i = 30; i < 50; i++) {
        cmd_data[i] = 0;
    }

    // Phase 3: Roll right (50-69)
    std::cout << "  [50-69] ROLL RIGHT - Banking maneuver" << std::endl;
    for (int i = 50; i < 70; i++) {
        cmd_data[i] = 39;
        imu_data[i].accel_x = to_fixed16(3.0f);
        imu_data[i].accel_z = to_fixed16(9.3f);
        imu_data[i].gyro_x = to_fixed16(0.5f);
    }

    // Phase 4: Pitch forward (70-89)
    std::cout << "  [70-89] PITCH FORWARD - Forward flight" << std::endl;
    for (int i = 70; i < 90; i++) {
        cmd_data[i] = 38;
        imu_data[i].accel_x = to_fixed16(0.0f);
        imu_data[i].accel_y = to_fixed16(3.0f);
        imu_data[i].accel_z = to_fixed16(9.3f);
        imu_data[i].gyro_y = to_fixed16(0.5f);
    }

    // Phase 5: Recovery/stabilize (90-99)
    std::cout << "  [90-99] STABILIZE - Return to hover" << std::endl;
    for (int i = 90; i < 100; i++) {
        cmd_data[i] = 0;
    }

    // ========================================================================
    // TRANSFER TO FPGA
    // ========================================================================
    print_separator("TRANSFERRING DATA TO FPGA");
    
    bo_imu.sync(XCL_BO_SYNC_BO_TO_DEVICE);
    bo_cmd.sync(XCL_BO_SYNC_BO_TO_DEVICE);
    std::cout << "Input data transferred to FPGA HBM" << std::endl;

    // ========================================================================
    // KERNEL EXECUTION
    // ========================================================================
    print_separator("EXECUTING KERNEL");
    
    std::cout << "Kernel parameters:" << std::endl;
    std::cout << "  mass = 1.0 kg" << std::endl;
    std::cout << "  inertia = (0.01, 0.01, 0.02) kg*m^2" << std::endl;
    std::cout << "  arm_length = 0.25 m" << std::endl;
    std::cout << "  sampling_period = 0.01 s" << std::endl;
    std::cout << "  gravity = 9.81 m/s^2" << std::endl;
    std::cout << "  num_samples = " << num_samples << std::endl;

    std::cout << "\nRunning kernel..." << std::flush;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    auto run = kernel(bo_imu, bo_cmd, bo_state, bo_motor,
        to_fixed32(1.0f),      // mass
        to_fixed32(0.01f),     // inertia_xx
        to_fixed32(0.01f),     // inertia_yy
        to_fixed32(0.02f),     // inertia_zz
        to_fixed32(0.0001f),   // motor_thrust_coeff (increased for visible output)
        to_fixed32(0.00001f),  // motor_torque_coeff (increased)
        to_fixed32(0.25f),     // arm_length
        to_fixed32(0.01f),     // sampling_period
        to_fixed32(9.81f),     // gravity
        num_samples);
    run.wait();
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    std::cout << " Done!" << std::endl;
    std::cout << "Execution time: " << duration.count() << " microseconds" << std::endl;
    std::cout << "Throughput: " << (num_samples * 1000000.0 / duration.count()) << " samples/sec" << std::endl;

    // ========================================================================
    // RETRIEVE RESULTS
    // ========================================================================
    print_separator("RETRIEVING RESULTS FROM FPGA");
    
    bo_state.sync(XCL_BO_SYNC_BO_FROM_DEVICE);
    bo_motor.sync(XCL_BO_SYNC_BO_FROM_DEVICE);
    std::cout << "Output data transferred from FPGA" << std::endl;

    // ========================================================================
    // DETAILED RESULTS
    // ========================================================================
    print_separator("PHASE 1: THROTTLE UP (Samples 0-29)");
    for (int i = 0; i < 30; i += 5) {
        print_full_sample(imu_data[i], cmd_data[i], state_data[i], motor_data[i], i, "THROTTLE UP");
    }

    print_separator("PHASE 2: HOVER (Samples 30-49)");
    for (int i = 30; i < 50; i += 5) {
        print_full_sample(imu_data[i], cmd_data[i], state_data[i], motor_data[i], i, "HOVER");
    }

    print_separator("PHASE 3: ROLL RIGHT (Samples 50-69)");
    for (int i = 50; i < 70; i += 5) {
        print_full_sample(imu_data[i], cmd_data[i], state_data[i], motor_data[i], i, "ROLL RIGHT");
    }

    print_separator("PHASE 4: PITCH FORWARD (Samples 70-89)");
    for (int i = 70; i < 90; i += 5) {
        print_full_sample(imu_data[i], cmd_data[i], state_data[i], motor_data[i], i, "PITCH FORWARD");
    }

    print_separator("PHASE 5: STABILIZE (Samples 90-99)");
    for (int i = 90; i < 100; i += 3) {
        print_full_sample(imu_data[i], cmd_data[i], state_data[i], motor_data[i], i, "STABILIZE");
    }

    // ========================================================================
    // SUMMARY
    // ========================================================================
    print_separator("TEST SUMMARY");
    
    std::cout << "\nInitial State (Sample 0):" << std::endl;
    std::cout << "  Roll:  " << from_fixed32(state_data[0].roll) << " deg" << std::endl;
    std::cout << "  Pitch: " << from_fixed32(state_data[0].pitch) << " deg" << std::endl;
    std::cout << "  Yaw:   " << from_fixed32(state_data[0].yaw) << " deg" << std::endl;
    std::cout << "  Altitude (PosZ): " << from_fixed32(state_data[0].pos_z) << " m" << std::endl;
    
    std::cout << "\nFinal State (Sample 99):" << std::endl;
    std::cout << "  Roll:  " << from_fixed32(state_data[99].roll) << " deg" << std::endl;
    std::cout << "  Pitch: " << from_fixed32(state_data[99].pitch) << " deg" << std::endl;
    std::cout << "  Yaw:   " << from_fixed32(state_data[99].yaw) << " deg" << std::endl;
    std::cout << "  Altitude (PosZ): " << from_fixed32(state_data[99].pos_z) << " m" << std::endl;

    // Check for any non-zero motor outputs
    int nonzero_motors = 0;
    float max_motor = 0;
    for (int i = 0; i < num_samples; i++) {
        float fl = from_fixed16(motor_data[i].front_left);
        float fr = from_fixed16(motor_data[i].front_right);
        float rl = from_fixed16(motor_data[i].rear_left);
        float rr = from_fixed16(motor_data[i].rear_right);
        if (fl != 0 || fr != 0 || rl != 0 || rr != 0) nonzero_motors++;
        max_motor = std::max(max_motor, std::max({fl, fr, rl, rr}));
    }
    
    std::cout << "\nMotor Analysis:" << std::endl;
    std::cout << "  Samples with non-zero motors: " << nonzero_motors << "/" << num_samples << std::endl;
    std::cout << "  Maximum motor value: " << max_motor << std::endl;

    // Check state changes
    float roll_change = from_fixed32(state_data[99].roll) - from_fixed32(state_data[0].roll);
    float pitch_change = from_fixed32(state_data[99].pitch) - from_fixed32(state_data[0].pitch);
    
    std::cout << "\nState Changes:" << std::endl;
    std::cout << "  Roll change:  " << roll_change << " deg" << std::endl;
    std::cout << "  Pitch change: " << pitch_change << " deg" << std::endl;

    print_separator("FPGA QUADCOPTER CONTROLLER TEST COMPLETE");
    
    std::cout << "\nVerification Checklist:" << std::endl;
    std::cout << "  [" << (sizeof(IMUData) == 18 ? "✓" : "✗") << "] IMUData struct size correct" << std::endl;
    std::cout << "  [" << (sizeof(StateVector) == 48 ? "✓" : "✗") << "] StateVector struct size correct" << std::endl;
    std::cout << "  [" << (sizeof(MotorSpeeds) == 8 ? "✓" : "✗") << "] MotorSpeeds struct size correct" << std::endl;
    std::cout << "  [✓] XCLBIN loaded successfully" << std::endl;
    std::cout << "  [✓] Kernel executed successfully" << std::endl;
    std::cout << "  [" << (from_fixed32(state_data[0].pos_z) == 50 ? "✓" : "✗") << "] Initial altitude = 50m" << std::endl;
    std::cout << "  [" << (roll_change != 0 || pitch_change != 0 ? "✓" : "✗") << "] State estimation working" << std::endl;
    std::cout << "  [" << (nonzero_motors > 0 ? "✓" : "✗") << "] Motor outputs generated" << std::endl;

    return 0;
}
