#include "quadcopter_control.h"
#include <cmath>
#include <hls_math.h>

void complementary_filter(IMUData imu, StateVector& state, fp32_t alpha) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    static fp32_t roll_gyro = 0, pitch_gyro = 0, yaw_gyro = 0;

    // Use fixed-point constants
    const fp32_t DT = fp32_t(0.01);  // 100Hz update rate
    const fp32_t PI = fp32_t(3.14159);
    const fp32_t TWO_PI = fp32_t(6.28318);

    // Gyroscope integration (short-term accuracy)
    roll_gyro += imu.gyro_x * DT;
    pitch_gyro += imu.gyro_y * DT;
    yaw_gyro += imu.gyro_z * DT;

    // Convert to double for math functions, then back to fp32_t
    double accel_x_d = imu.accel_x.to_double();
    double accel_y_d = imu.accel_y.to_double();
    double accel_z_d = imu.accel_z.to_double();

    // Accelerometer attitude estimation (long-term stability)
    double denominator1 = sqrt(accel_x_d * accel_x_d + accel_z_d * accel_z_d);
    double denominator2 = sqrt(accel_y_d * accel_y_d + accel_z_d * accel_z_d);

    fp32_t roll_acc = fp32_t(atan2(accel_y_d, denominator1));
    fp32_t pitch_acc = fp32_t(atan2(-accel_x_d, denominator2));

    // Convert magnetometer data to double
    double mag_x_d = imu.mag_x.to_double();
    double mag_y_d = imu.mag_y.to_double();
    double mag_z_d = imu.mag_z.to_double();

    // Convert angles to double for trig functions
    double roll_acc_d = roll_acc.to_double();
    double pitch_acc_d = pitch_acc.to_double();

    // Magnetometer yaw estimation
    double sin_roll = sin(roll_acc_d);
    double cos_roll = cos(roll_acc_d);
    double sin_pitch = sin(pitch_acc_d);
    double cos_pitch = cos(pitch_acc_d);

    double mag_x = mag_x_d * cos_pitch + mag_y_d * sin_roll * sin_pitch + mag_z_d * cos_roll * sin_pitch;
    double mag_y = mag_y_d * cos_roll - mag_z_d * sin_roll;
    fp32_t yaw_mag = fp32_t(atan2(-mag_y, mag_x));

    // Complementary filter fusion
    state.roll = alpha * (state.roll + roll_gyro) + (fp32_t(1) - alpha) * roll_acc;
    state.pitch = alpha * (state.pitch + pitch_gyro) + (fp32_t(1) - alpha) * pitch_acc;
    state.yaw = alpha * (state.yaw + yaw_gyro) + (fp32_t(1) - alpha) * yaw_mag;

    // Normalize angles to [-π, π]
    if (state.roll > PI) state.roll -= TWO_PI;
    if (state.roll < -PI) state.roll += TWO_PI;
    if (state.pitch > PI) state.pitch -= TWO_PI;
    if (state.pitch < -PI) state.pitch += TWO_PI;
    if (state.yaw > PI) state.yaw -= TWO_PI;
    if (state.yaw < -PI) state.yaw += TWO_PI;
}
