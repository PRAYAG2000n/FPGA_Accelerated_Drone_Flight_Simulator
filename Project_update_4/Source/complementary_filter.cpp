#include "quadcopter_control.h"
#include <cmath>

// Helper functions to convert ap_fixed to double for math operations
double to_double(fp32_t value) {
    return value.to_double();
}

double to_double(sensor_data_t value) {
    return value.to_double();
}

// Convert back to fp32_t after math operations
fp32_t to_fp32(double value) {
    return fp32_t(value);
}

void complementary_filter(IMUData imu, StateVector& state, fp32_t alpha) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    static fp32_t roll_gyro = 0, pitch_gyro = 0, yaw_gyro = 0;

    // Gyroscope integration (short-term accuracy)
    roll_gyro += imu.gyro_x * fp32_t(0.01);
    pitch_gyro += imu.gyro_y * fp32_t(0.01);
    yaw_gyro += imu.gyro_z * fp32_t(0.01);

    // Convert to double for math operations
    double accel_x_d = to_double(imu.accel_x);
    double accel_y_d = to_double(imu.accel_y);
    double accel_z_d = to_double(imu.accel_z);
    double mag_x_d = to_double(imu.mag_x);
    double mag_y_d = to_double(imu.mag_y);
    double mag_z_d = to_double(imu.mag_z);
    double roll_d = to_double(state.roll);
    double pitch_d = to_double(state.pitch);
    double yaw_d = to_double(state.yaw);

    // Accelerometer attitude estimation (long-term stability)
    double accel_magnitude = sqrt(accel_x_d * accel_x_d +
                                 accel_y_d * accel_y_d +
                                 accel_z_d * accel_z_d);

    // Only use accelerometer if magnitude is reasonable (not in free-fall)
    if (accel_magnitude > 8.0 && accel_magnitude < 12.0) {
        double roll_acc_d = atan2(accel_y_d, accel_z_d);
        double pitch_acc_d = atan2(-accel_x_d, sqrt(accel_y_d * accel_y_d + accel_z_d * accel_z_d));

        // Convert back to fp32_t
        fp32_t roll_acc = to_fp32(roll_acc_d);
        fp32_t pitch_acc = to_fp32(pitch_acc_d);

        // Complementary filter fusion
        state.roll = alpha * (state.roll + roll_gyro) + (fp32_t(1) - alpha) * roll_acc;
        state.pitch = alpha * (state.pitch + pitch_gyro) + (fp32_t(1) - alpha) * pitch_acc;
    } else {
        // Only gyro when accelerometer is unreliable
        state.roll += roll_gyro;
        state.pitch += pitch_gyro;
    }

    // Magnetometer yaw estimation
    double mag_x_rotated = mag_x_d * cos(pitch_d) +
                          mag_y_d * sin(roll_d) * sin(pitch_d) +
                          mag_z_d * cos(roll_d) * sin(pitch_d);
    double mag_y_rotated = mag_y_d * cos(roll_d) - mag_z_d * sin(roll_d);
    double yaw_mag_d = atan2(-mag_y_rotated, mag_x_rotated);

    fp32_t yaw_mag = to_fp32(yaw_mag_d);
    state.yaw = alpha * (state.yaw + yaw_gyro) + (fp32_t(1) - alpha) * yaw_mag;

    // Normalize angles to [-π, π]
    const fp32_t PI = fp32_t(3.14159265358979323846);
    const fp32_t TWO_PI = fp32_t(2) * PI;

    if (state.roll > PI) state.roll -= TWO_PI;
    if (state.roll < -PI) state.roll += TWO_PI;
    if (state.pitch > PI) state.pitch -= TWO_PI;
    if (state.pitch < -PI) state.pitch += TWO_PI;
    if (state.yaw > PI) state.yaw -= TWO_PI;
    if (state.yaw < -PI) state.yaw += TWO_PI;

    // Reset gyro integrals for next iteration
    roll_gyro = 0;
    pitch_gyro = 0;
    yaw_gyro = 0;
}
