#include "complementary_filter.h"
#include <cmath>

// Helper functions for ap_fixed math operations
fp16_t atan2_fixed(fp16_t y, fp16_t x) {
    // Convert to double for math, then back to fixed-point
    return fp16_t(atan2(y.to_double(), x.to_double()));
}

fp16_t sqrt_fixed(fp16_t x) {
    // Convert to double for math, then back to fixed-point
    return fp16_t(sqrt(x.to_double()));
}

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
    #pragma HLS INLINE

    Attitude accel_angles;

    // Calculate roll and pitch from accelerometer
    // Roll (rotation around X-axis)
    accel_angles.roll = atan2_fixed(accel_y, accel_z);

    // Pitch (rotation around Y-axis)
    fp16_t denominator = sqrt_fixed(accel_y * accel_y + accel_z * accel_z);
    accel_angles.pitch = atan2_fixed(-accel_x, denominator);

    // Yaw cannot be determined from accelerometer alone
    accel_angles.yaw = 0;

    return accel_angles;
}

// Update attitude estimate
Attitude ComplementaryFilter::update(const IMUData& imu) {
    #pragma HLS PIPELINE II=1

    // Calculate angles from accelerometer
    Attitude accel_angles = calculateAccelAngles(imu.accel_x, imu.accel_y, imu.accel_z);

    // Use explicit type conversion to avoid operator ambiguity
    fp16_t one_minus_alpha = fp16_t(1.0) - alpha;

    // Complementary filter update
    // Roll: α * (previous_roll + gyro_x * dt) + (1-α) * accel_roll
    attitude.roll = alpha * (attitude.roll + imu.gyro_x * dt) +
                   one_minus_alpha * accel_angles.roll;

    // Pitch: α * (previous_pitch + gyro_y * dt) + (1-α) * accel_pitch
    attitude.pitch = alpha * (attitude.pitch + imu.gyro_y * dt) +
                    one_minus_alpha * accel_angles.pitch;

    // Yaw: integrate gyro_z directly (no accelerometer correction for yaw)
    attitude.yaw = attitude.yaw + imu.gyro_z * dt;

    return attitude;
}

// Get current attitude
Attitude ComplementaryFilter::getAttitude() const {
    return attitude;
}
