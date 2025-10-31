#include "complementary_filter.h"
#include <cmath>

// Constructor
ComplementaryFilter::ComplementaryFilter(fp16_t alpha_val, fp16_t dt_val)
    : alpha(alpha_val), dt(dt_val) {
    init();
}

// Initialize filter
void ComplementaryFilter::init() {
    attitude.roll = 0;
    attitude.pitch = 0;
    attitude.yaw = 0;
}

// Calculate angles from accelerometer data
Attitude ComplementaryFilter::calculateAccelAngles(fp16_t accel_x, fp16_t accel_y, fp16_t accel_z) {
    Attitude accel_angles;

    // Calculate roll and pitch from accelerometer
    // Roll (rotation around X-axis)
    accel_angles.roll = atan2(accel_y, accel_z);

    // Pitch (rotation around Y-axis)
    fp16_t denominator = sqrt(accel_y * accel_y + accel_z * accel_z);
    accel_angles.pitch = atan2(-accel_x, denominator);

    // Yaw cannot be determined from accelerometer alone
    accel_angles.yaw = 0;

    return accel_angles;
}

// Update attitude estimate
Attitude ComplementaryFilter::update(const IMUData& imu) {
    // Calculate angles from accelerometer
    Attitude accel_angles = calculateAccelAngles(imu.accel_x, imu.accel_y, imu.accel_z);

    // Complementary filter update
    // Roll: α * (previous_roll + gyro_x * dt) + (1-α) * accel_roll
    attitude.roll = alpha * (attitude.roll + imu.gyro_x * dt) +
                   (1.0f - alpha) * accel_angles.roll;

    // Pitch: α * (previous_pitch + gyro_y * dt) + (1-α) * accel_pitch
    attitude.pitch = alpha * (attitude.pitch + imu.gyro_y * dt) +
                    (1.0f - alpha) * accel_angles.pitch;

    // Yaw: integrate gyro_z directly (no accelerometer correction for yaw)
    attitude.yaw = attitude.yaw + imu.gyro_z * dt;

    return attitude;
}

// Get current attitude
Attitude ComplementaryFilter::getAttitude() const {
    return attitude;
}