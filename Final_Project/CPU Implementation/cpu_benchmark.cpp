
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <chrono>
#include <fstream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <sstream>

#define NUM_SAMPLES 100

// DATA STRUCTURES
struct IMUData {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
};

struct StateVector {
    float roll, pitch, yaw;
    float pos_z;
    float ang_vel_x, ang_vel_y, ang_vel_z;
};

struct MotorSpeeds {
    float front_left, front_right;
    float rear_left, rear_right;
};

struct ControlCommands {
    float throttle;
    float roll_cmd, pitch_cmd, yaw_cmd;
};

struct PIDState {
    float roll_integral, pitch_integral, yaw_integral, alt_integral;
    float prev_roll_error, prev_pitch_error, prev_yaw_error, prev_alt_error;
};

// ============================================================================
// CPU INFO AND POWER ESTIMATION
// ============================================================================
struct CPUInfo {
    char model_name[256];
    int cores;
    float max_freq_ghz;
    float tdp_watts;
};

CPUInfo get_cpu_info() {
    CPUInfo info = {"Unknown", 1, 2.0f, 65.0f};

    // Get CPU model name
    std::ifstream cpuinfo("/proc/cpuinfo");
    std::string line;
    int core_count = 0;

    while (std::getline(cpuinfo, line)) {
        if (line.find("model name") != std::string::npos) {
            size_t pos = line.find(":");
            if (pos != std::string::npos) {
                strncpy(info.model_name, line.substr(pos + 2).c_str(), 255);
            }
        }
        if (line.find("processor") != std::string::npos) {
            core_count++;
        }
        if (line.find("cpu MHz") != std::string::npos) {
            size_t pos = line.find(":");
            if (pos != std::string::npos) {
                info.max_freq_ghz = std::stof(line.substr(pos + 2)) / 1000.0f;
            }
        }
    }
    info.cores = core_count > 0 ? core_count : 1;

    // Estimate TDP based on CPU model
    std::string model(info.model_name);
    if (model.find("Xeon") != std::string::npos) {
        if (model.find("Platinum") != std::string::npos) info.tdp_watts = 150.0f;
        else if (model.find("Gold") != std::string::npos) info.tdp_watts = 125.0f;
        else if (model.find("Silver") != std::string::npos) info.tdp_watts = 85.0f;
        else if (model.find("E5") != std::string::npos) info.tdp_watts = 120.0f;
        else if (model.find("E7") != std::string::npos) info.tdp_watts = 150.0f;
        else info.tdp_watts = 105.0f;
    } else if (model.find("EPYC") != std::string::npos) {
        info.tdp_watts = 180.0f;
    } else if (model.find("i9") != std::string::npos) {
        info.tdp_watts = 125.0f;
    } else if (model.find("i7") != std::string::npos) {
        info.tdp_watts = 95.0f;
    } else if (model.find("i5") != std::string::npos) {
        info.tdp_watts = 65.0f;
    } else if (model.find("Ryzen 9") != std::string::npos) {
        info.tdp_watts = 105.0f;
    } else if (model.find("Ryzen 7") != std::string::npos) {
        info.tdp_watts = 65.0f;
    }

    return info;
}

// Read CPU utilization
double get_cpu_utilization() {
    static long long prev_idle = 0, prev_total = 0;

    std::ifstream stat("/proc/stat");
    std::string line;
    std::getline(stat, line);

    std::istringstream iss(line);
    std::string cpu;
    long long user, nice, system, idle, iowait, irq, softirq, steal;
    iss >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

    long long idle_time = idle + iowait;
    long long total_time = user + nice + system + idle + iowait + irq + softirq + steal;

    long long idle_diff = idle_time - prev_idle;
    long long total_diff = total_time - prev_total;

    prev_idle = idle_time;
    prev_total = total_time;

    if (total_diff == 0) return 0;
    return 100.0 * (1.0 - (double)idle_diff / total_diff);
}

// CPU IMPLEMENTATION

inline float clamp(float val, float min_val, float max_val) {
    return fmaxf(min_val, fminf(max_val, val));
}

float cpu_pid(float error, float& integral, float& prev_error,
              float kp, float ki, float kd) {
    float derivative = error - prev_error;
    integral = clamp(integral + error, -50.0f, 50.0f);
    float output = kp * error + ki * integral + kd * derivative;
    prev_error = error;
    return clamp(output, -80.0f, 80.0f);
}

void flight_controller(const IMUData& imu, StateVector& state,
                       const ControlCommands& cmd, MotorSpeeds& motor,
                       PIDState& pid, float alpha) {
    const float dt = 0.01f;

    float roll_gyro = imu.gyro_x * dt;
    float pitch_gyro = imu.gyro_y * dt;

    float accel_mag_sq = imu.accel_x*imu.accel_x + imu.accel_y*imu.accel_y + imu.accel_z*imu.accel_z;

    if (accel_mag_sq > 64.0f && accel_mag_sq < 144.0f) {
        float roll_acc = imu.accel_y * 0.1f;
        float pitch_acc = -imu.accel_x * 0.1f;
        state.roll = alpha * (state.roll + roll_gyro) + (1.0f - alpha) * roll_acc;
        state.pitch = alpha * (state.pitch + pitch_gyro) + (1.0f - alpha) * pitch_acc;
    } else {
        state.roll += roll_gyro;
        state.pitch += pitch_gyro;
    }

    const float rad_to_deg = 57.2957795f;
    float roll_error = cmd.roll_cmd - state.roll * rad_to_deg;
    float pitch_error = cmd.pitch_cmd - state.pitch * rad_to_deg;
    float yaw_error = cmd.yaw_cmd - state.yaw * rad_to_deg;

    if (fabsf(roll_error) < 0.5f) roll_error = 0;
    if (fabsf(pitch_error) < 0.5f) pitch_error = 0;

    float roll_ctrl = clamp(cpu_pid(roll_error, pid.roll_integral, pid.prev_roll_error, 1.5f, 0.02f, 1.2f), -20.0f, 20.0f);
    float pitch_ctrl = clamp(cpu_pid(pitch_error, pid.pitch_integral, pid.prev_pitch_error, 1.5f, 0.02f, 1.2f), -20.0f, 20.0f);
    float yaw_ctrl = clamp(cpu_pid(yaw_error, pid.yaw_integral, pid.prev_yaw_error, 1.8f, 0.01f, 0.5f), -30.0f, 30.0f);

    float alt_error = cmd.throttle - state.pos_z;
    float alt_deriv = alt_error - pid.prev_alt_error;
    pid.alt_integral = clamp(pid.alt_integral + alt_error, -25.0f, 25.0f);
    float alt_throttle = clamp(1.5f * alt_error + 0.05f * pid.alt_integral + 1.0f * alt_deriv + 48.0f, 15.0f, 85.0f);
    pid.prev_alt_error = alt_error;

    float throttle = clamp(cmd.throttle * 0.2f + alt_throttle * 0.8f, 0.0f, 100.0f);
    float sr = roll_ctrl * 0.2f, sp = pitch_ctrl * 0.2f, sy = yaw_ctrl * 0.15f;

    motor.front_left  = clamp(throttle + sp + sr + sy, 0.0f, 100.0f);
    motor.front_right = clamp(throttle + sp - sr - sy, 0.0f, 100.0f);
    motor.rear_left   = clamp(throttle - sp + sr - sy, 0.0f, 100.0f);
    motor.rear_right  = clamp(throttle - sp - sr + sy, 0.0f, 100.0f);
}


// MAIN
int main() {

    printf("\nCPU Quadcopter Flight Controller Benchmark\n");
    printf("  Samples: %d\n", NUM_SAMPLES);

    // Get CPU info
    CPUInfo cpu_info = get_cpu_info();
    printf("CPU: %s\n", cpu_info.model_name);
    printf("Cores: %d, Freq: %.2f GHz, TDP: %.0f W\n\n",
           cpu_info.cores, cpu_info.max_freq_ghz, cpu_info.tdp_watts);

    // Allocate memory
    size_t imu_size = NUM_SAMPLES * sizeof(IMUData);
    size_t state_size = NUM_SAMPLES * sizeof(StateVector);
    size_t cmd_size = NUM_SAMPLES * sizeof(ControlCommands);
    size_t motor_size = NUM_SAMPLES * sizeof(MotorSpeeds);
    size_t pid_size = NUM_SAMPLES * sizeof(PIDState);
    size_t total_mem = imu_size + state_size + cmd_size + motor_size + pid_size;

    IMUData* imu = (IMUData*)malloc(imu_size);
    StateVector* states = (StateVector*)malloc(state_size);
    ControlCommands* cmds = (ControlCommands*)malloc(cmd_size);
    MotorSpeeds* motors = (MotorSpeeds*)malloc(motor_size);
    PIDState* pids = (PIDState*)malloc(pid_size);

    // Initialize data
    for (int i = 0; i < NUM_SAMPLES; i++) {
        imu[i] = {0.1f, 0.1f, 9.81f, 0.01f, 0.01f, 0.0f};
        states[i] = {0, 0, 0, 50.0f, 0, 0, 0};
        cmds[i] = {50.0f, 0, 0, 0};
        pids[i] = {0, 0, 0, 0, 0, 0, 0, 0};
    }

    // Warmup
    for (int j = 0; j < NUM_SAMPLES; j++) {
        flight_controller(imu[j], states[j], cmds[j], motors[j], pids[j], 0.98f);
    }

    // Initialize CPU utilization tracking
    get_cpu_utilization();

    // Run multiple times to get latency statistics
    const int RUNS = 100;
    std::vector<double> latencies(RUNS);
    std::vector<double> cpu_utils(RUNS);

    for (int r = 0; r < RUNS; r++) {
        // Reset state
        for (int i = 0; i < NUM_SAMPLES; i++) {
            states[i] = {0, 0, 0, 50.0f, 0, 0, 0};
            pids[i] = {0, 0, 0, 0, 0, 0, 0, 0};
        }

        auto start = std::chrono::high_resolution_clock::now();

        for (int j = 0; j < NUM_SAMPLES; j++) {
            flight_controller(imu[j], states[j], cmds[j], motors[j], pids[j], 0.98f);
        }

        auto end = std::chrono::high_resolution_clock::now();
        latencies[r] = std::chrono::duration<double, std::micro>(end - start).count();
        cpu_utils[r] = get_cpu_utilization();
    }

    // Calculate statistics
    double sum = std::accumulate(latencies.begin(), latencies.end(), 0.0);
    double mean = sum / RUNS;
    double min_lat = *std::min_element(latencies.begin(), latencies.end());
    double max_lat = *std::max_element(latencies.begin(), latencies.end());

    double sq_sum = 0;
    for (auto& l : latencies) sq_sum += (l - mean) * (l - mean);
    double std_dev = sqrt(sq_sum / RUNS);

    double throughput = NUM_SAMPLES / (mean / 1e6);

    // Calculate power based on single-core utilization
    // Power model: P = P_idle + (P_tdp - P_idle) * utilization * (cores_used / total_cores)
    double avg_util = std::accumulate(cpu_utils.begin(), cpu_utils.end(), 0.0) / RUNS;
    double idle_power = cpu_info.tdp_watts * 0.3;  // ~30% TDP at idle
    double single_core_fraction = 1.0 / cpu_info.cores;
    double power_w = idle_power + (cpu_info.tdp_watts - idle_power) * (avg_util / 100.0) * single_core_fraction;

    // For single-threaded workload, estimate per-core power
    double per_core_tdp = cpu_info.tdp_watts / cpu_info.cores;
    double active_power = per_core_tdp * 0.8;  // 80% of per-core TDP when active

  
    printf("\nRESULTS\n");
    
    printf("  Execution Time (mean):   %.2f us\n", mean);
    printf("  Execution Time (min):    %.2f us\n", min_lat);
    printf("  Execution Time (max):    %.2f us\n", max_lat);
    printf("  Execution Time (std):    %.2f us\n", std_dev);
    printf("  Latency (per sample):    %.4f us\n", mean / NUM_SAMPLES);
    printf("  Throughput:              %.2f samples/sec\n", throughput);
   
    printf("\nRESOURCE UTILIZATION\n");
  
    printf("  Memory Used:             %.2f KB\n", total_mem / 1024.0);
    printf("  Threads:                 1 (single-threaded)\n");
    printf("  CPU Utilization:         %.2f%% (of 1 core)\n", avg_util);

    printf("\nPOWER CONSUMPTION\n");

    printf("  CPU TDP:                 %.0f W\n", cpu_info.tdp_watts);
    printf("  Per-Core TDP:            %.2f W\n", per_core_tdp);
    printf("  Power (1 core active):   %.2f W\n", active_power);


    // Cleanup
    free(imu); free(states); free(cmds); free(motors); free(pids);

    return 0;
}
