// gpu_benchmark.cu
// GPU benchmark for quadcopter flight controller
// Compile: nvcc -O3 -arch=sm_70 -o gpu_benchmark gpu_benchmark.cu -lnvidia-ml

#include <cuda_runtime.h>
#include <nvml.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <numeric>

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

// GPU KERNEL
__device__ float gpu_pid(float error, float& integral, float& prev_error,
                         float kp, float ki, float kd) {
    float derivative = error - prev_error;
    integral += error;
    integral = fmaxf(-50.0f, fminf(50.0f, integral));
    float output = kp * error + ki * integral + kd * derivative;
    prev_error = error;
    return fmaxf(-80.0f, fminf(80.0f, output));
}

__global__ void flight_controller_kernel(
    const IMUData* __restrict__ imu_data,
    StateVector* __restrict__ states,
    const ControlCommands* __restrict__ commands,
    MotorSpeeds* __restrict__ motors,
    PIDState* __restrict__ pid_states,
    float alpha
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= NUM_SAMPLES) return;

    IMUData imu = imu_data[idx];
    StateVector state = states[idx];
    ControlCommands cmd = commands[idx];
    PIDState pid = pid_states[idx];
    MotorSpeeds motor;

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

    float roll_ctrl = gpu_pid(roll_error, pid.roll_integral, pid.prev_roll_error, 1.5f, 0.02f, 1.2f);
    float pitch_ctrl = gpu_pid(pitch_error, pid.pitch_integral, pid.prev_pitch_error, 1.5f, 0.02f, 1.2f);
    float yaw_ctrl = gpu_pid(yaw_error, pid.yaw_integral, pid.prev_yaw_error, 1.8f, 0.01f, 0.5f);

    roll_ctrl = fmaxf(-20.0f, fminf(20.0f, roll_ctrl));
    pitch_ctrl = fmaxf(-20.0f, fminf(20.0f, pitch_ctrl));
    yaw_ctrl = fmaxf(-30.0f, fminf(30.0f, yaw_ctrl));

    float alt_error = cmd.throttle - state.pos_z;
    float alt_deriv = alt_error - pid.prev_alt_error;
    pid.alt_integral += alt_error;
    pid.alt_integral = fmaxf(-25.0f, fminf(25.0f, pid.alt_integral));
    float alt_throttle = 1.5f * alt_error + 0.05f * pid.alt_integral + 1.0f * alt_deriv + 48.0f;
    alt_throttle = fmaxf(15.0f, fminf(85.0f, alt_throttle));
    pid.prev_alt_error = alt_error;

    float throttle = cmd.throttle * 0.2f + alt_throttle * 0.8f;
    throttle = fmaxf(0, fminf(100, throttle));

    float sr = roll_ctrl * 0.2f, sp = pitch_ctrl * 0.2f, sy = yaw_ctrl * 0.15f;
    motor.front_left  = fmaxf(0, fminf(100, throttle + sp + sr + sy));
    motor.front_right = fmaxf(0, fminf(100, throttle + sp - sr - sy));
    motor.rear_left   = fmaxf(0, fminf(100, throttle - sp + sr - sy));
    motor.rear_right  = fmaxf(0, fminf(100, throttle - sp - sr + sy));

    states[idx] = state;
    motors[idx] = motor;
    pid_states[idx] = pid;
}


// MAIN
int main() {

    printf("\nGPU Quadcopter Flight Controller Benchmark\n");
    printf("  Samples: %d\n", NUM_SAMPLES);


    // Initialize NVML
    nvmlInit();
    nvmlDevice_t nvml_device;
    nvmlDeviceGetHandleByIndex(0, &nvml_device);

    // GPU Info
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    printf("GPU: %s\n", prop.name);
    printf("SMs: %d\n\n", prop.multiProcessorCount);

    // Allocate memory
    size_t imu_size = NUM_SAMPLES * sizeof(IMUData);
    size_t state_size = NUM_SAMPLES * sizeof(StateVector);
    size_t cmd_size = NUM_SAMPLES * sizeof(ControlCommands);
    size_t motor_size = NUM_SAMPLES * sizeof(MotorSpeeds);
    size_t pid_size = NUM_SAMPLES * sizeof(PIDState);
    size_t total_mem = imu_size + state_size + cmd_size + motor_size + pid_size;

    IMUData* h_imu = (IMUData*)malloc(imu_size);
    StateVector* h_states = (StateVector*)malloc(state_size);
    ControlCommands* h_cmds = (ControlCommands*)malloc(cmd_size);
    MotorSpeeds* h_motors = (MotorSpeeds*)malloc(motor_size);
    PIDState* h_pids = (PIDState*)malloc(pid_size);

    IMUData *d_imu; StateVector *d_states; ControlCommands *d_cmds;
    MotorSpeeds *d_motors; PIDState *d_pids;

    cudaMalloc(&d_imu, imu_size);
    cudaMalloc(&d_states, state_size);
    cudaMalloc(&d_cmds, cmd_size);
    cudaMalloc(&d_motors, motor_size);
    cudaMalloc(&d_pids, pid_size);

    int threads = 256;
    int blocks = (NUM_SAMPLES + threads - 1) / threads;

    // Run multiple times to get latency statistics
    const int RUNS = 100;
    std::vector<float> latencies(RUNS);

    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    // Warmup
    for (int i = 0; i < NUM_SAMPLES; i++) {
        h_imu[i] = {0.1f, 0.1f, 9.81f, 0.01f, 0.01f, 0.0f};
        h_states[i] = {0, 0, 0, 50.0f, 0, 0, 0};
        h_cmds[i] = {50.0f, 0, 0, 0};
        h_pids[i] = {0, 0, 0, 0, 0, 0, 0, 0};
    }
    cudaMemcpy(d_imu, h_imu, imu_size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_states, h_states, state_size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_cmds, h_cmds, cmd_size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_pids, h_pids, pid_size, cudaMemcpyHostToDevice);

    flight_controller_kernel<<<blocks, threads>>>(d_imu, d_states, d_cmds, d_motors, d_pids, 0.98f);
    cudaDeviceSynchronize();

    // Get power
    unsigned int power_mw;
    nvmlDeviceGetPowerUsage(nvml_device, &power_mw);

    // Benchmark runs
    for (int r = 0; r < RUNS; r++) {
        // Reset state
        for (int i = 0; i < NUM_SAMPLES; i++) {
            h_states[i] = {0, 0, 0, 50.0f, 0, 0, 0};
            h_pids[i] = {0, 0, 0, 0, 0, 0, 0, 0};
        }
        cudaMemcpy(d_states, h_states, state_size, cudaMemcpyHostToDevice);
        cudaMemcpy(d_pids, h_pids, pid_size, cudaMemcpyHostToDevice);

        cudaEventRecord(start);
        flight_controller_kernel<<<blocks, threads>>>(d_imu, d_states, d_cmds, d_motors, d_pids, 0.98f);
        cudaEventRecord(stop);
        cudaEventSynchronize(stop);

        float ms;
        cudaEventElapsedTime(&ms, start, stop);
        latencies[r] = ms * 1000.0f; // Convert to us
    }

    // Calculate statistics
    float sum = std::accumulate(latencies.begin(), latencies.end(), 0.0f);
    float mean = sum / RUNS;
    float min_lat = *std::min_element(latencies.begin(), latencies.end());
    float max_lat = *std::max_element(latencies.begin(), latencies.end());

    float sq_sum = 0;
    for (auto& l : latencies) sq_sum += (l - mean) * (l - mean);
    float std_dev = sqrt(sq_sum / RUNS);

    float throughput = NUM_SAMPLES / (mean / 1e6);
    float power_w = power_mw / 1000.0f;

    printf("\nRESULTS\n");

    printf("  Execution Time (mean):   %.2f us\n", mean);
    printf("  Execution Time (min):    %.2f us\n", min_lat);
    printf("  Execution Time (max):    %.2f us\n", max_lat);
    printf("  Execution Time (std):    %.2f us\n", std_dev);
    printf("  Latency (per sample):    %.4f us\n", mean / NUM_SAMPLES);
    printf("  Throughput:              %.2f samples/sec\n", throughput);
    printf("\nRESOURCE UTILIZATION\n");
    printf("  Grid Size:               %d blocks x %d threads\n", blocks, threads);
    printf("  Active Threads:          %d\n", NUM_SAMPLES);
    printf("  Memory Used:             %.2f KB\n", total_mem / 1024.0f);

    printf("\nPOWER CONSUMPTION\n");

    printf("\nPower:                   %.2f W\n", power_w);


    // Cleanup
    cudaEventDestroy(start);
    cudaEventDestroy(stop);
    cudaFree(d_imu); cudaFree(d_states); cudaFree(d_cmds);
    cudaFree(d_motors); cudaFree(d_pids);
    free(h_imu); free(h_states); free(h_cmds); free(h_motors); free(h_pids);
    nvmlShutdown();

    return 0;
}
