#include "quadcopter_control.h"
#include "hls_math.h"

void complementary_filter(IMUData imu, StateVector& state, fp32_t alpha) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    static fp32_t roll_gyro = 0, pitch_gyro = 0, yaw_gyro = 0;

    fp32_t accel_x = imu.accel_x;
    fp32_t accel_y = imu.accel_y;
    fp32_t accel_z = imu.accel_z;
    fp32_t gyro_x = imu.gyro_x;
    fp32_t gyro_y = imu.gyro_y;
    fp32_t gyro_z = imu.gyro_z;

    // Gyro integration (dt = 0.01s)
    roll_gyro += gyro_x * fp32_t(0.01);
    pitch_gyro += gyro_y * fp32_t(0.01);
    yaw_gyro += gyro_z * fp32_t(0.01);

    // Accelerometer magnitude check
    fp32_t accel_mag_sq = accel_x * accel_x + accel_y * accel_y + accel_z * accel_z;

    if (accel_mag_sq > fp32_t(64.0) && accel_mag_sq < fp32_t(144.0)) {
        // Small-angle approximation
        fp32_t roll_acc = accel_y * fp32_t(0.1);
        fp32_t pitch_acc = -accel_x * fp32_t(0.1);

        // Complementary filter
        state.roll = alpha * (state.roll + roll_gyro) + (fp32_t(1) - alpha) * roll_acc;
        state.pitch = alpha * (state.pitch + pitch_gyro) + (fp32_t(1) - alpha) * pitch_acc;
    } else {
        state.roll += roll_gyro;
        state.pitch += pitch_gyro;
    }

    state.yaw += yaw_gyro;

    // Angle normalization
    const fp32_t PI = fp32_t(3.14159);
    const fp32_t TWO_PI = fp32_t(6.28318);

    if (state.roll > PI) state.roll -= TWO_PI;
    else if (state.roll < -PI) state.roll += TWO_PI;

    if (state.pitch > PI) state.pitch -= TWO_PI;
    else if (state.pitch < -PI) state.pitch += TWO_PI;

    if (state.yaw > PI) state.yaw -= TWO_PI;
    else if (state.yaw < -PI) state.yaw += TWO_PI;

    roll_gyro = 0;
    pitch_gyro = 0;
    yaw_gyro = 0;
}
