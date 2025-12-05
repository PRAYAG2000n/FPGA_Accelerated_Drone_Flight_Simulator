#include "quadcopter_control.h"

void complementary_filter(IMUData imu, StateVector& state, fp32_t alpha) {
    #pragma HLS PIPELINE off
    #pragma HLS INLINE off

    fp32_t accel_x = fp32_t(imu.accel_x);
    fp32_t accel_y = fp32_t(imu.accel_y);
    fp32_t accel_z = fp32_t(imu.accel_z);
    fp32_t gyro_x = fp32_t(imu.gyro_x);
    fp32_t gyro_y = fp32_t(imu.gyro_y);
    fp32_t gyro_z = fp32_t(imu.gyro_z);

    const fp32_t dt = fp32_t(0.01);

    // Gyroscope integration
    fp32_t roll_gyro = state.roll + gyro_x * dt;
    fp32_t pitch_gyro = state.pitch + gyro_y * dt;
    fp32_t yaw_gyro = state.yaw + gyro_z * dt;

    // Simple accelerometer-based attitude (no division)
    // Use gravity constant for normalization instead of dividing by accel_z
    const fp32_t inv_gravity = fp32_t(0.102);  // 1/9.81 pre-computed

    fp32_t roll_acc = accel_y * inv_gravity;
    fp32_t pitch_acc = -accel_x * inv_gravity;

    // Complementary filter
    fp32_t one_minus_alpha = fp32_t(1.0) - alpha;
    state.roll = alpha * roll_gyro + one_minus_alpha * roll_acc;
    state.pitch = alpha * pitch_gyro + one_minus_alpha * pitch_acc;
    state.yaw = yaw_gyro;

    // Angle normalization
    const fp32_t PI = fp32_t(3.14159265);
    const fp32_t TWO_PI = fp32_t(6.28318530);

    if (state.roll > PI) {
        state.roll = state.roll - TWO_PI;
    } else if (state.roll < -PI) {
        state.roll = state.roll + TWO_PI;
    }

    if (state.pitch > PI) {
        state.pitch = state.pitch - TWO_PI;
    } else if (state.pitch < -PI) {
        state.pitch = state.pitch + TWO_PI;
    }

    if (state.yaw > PI) {
        state.yaw = state.yaw - TWO_PI;
    } else if (state.yaw < -PI) {
        state.yaw = state.yaw + TWO_PI;
    }
}
