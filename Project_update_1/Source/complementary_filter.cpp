#include "complementary_filter.h"
#include <cmath>


ComplementaryFilter::ComplementaryFilter(fp16_t alpha_val, fp16_t dt_val)
    : alpha(alpha_val), dt(dt_val) {
    init();
}

void ComplementaryFilter::init() {
    attitude.roll = 0;
    attitude.pitch = 0;
    attitude.yaw = 0;
}


Attitude ComplementaryFilter::calculateAccelAngles(fp16_t accel_x, fp16_t accel_y, fp16_t accel_z) {
    Attitude accel_angles;

    accel_angles.roll = atan2(accel_y, accel_z);

    fp16_t denominator = sqrt(accel_y * accel_y + accel_z * accel_z);
    accel_angles.pitch = atan2(-accel_x, denominator);

    accel_angles.yaw = 0;

    return accel_angles;
}


Attitude ComplementaryFilter::update(const IMUData& imu) {

    Attitude accel_angles = calculateAccelAngles(imu.accel_x, imu.accel_y, imu.accel_z);

    attitude.roll = alpha * (attitude.roll + imu.gyro_x * dt) +
                   (1.0f - alpha) * accel_angles.roll;

    attitude.pitch = alpha * (attitude.pitch + imu.gyro_y * dt) +
                    (1.0f - alpha) * accel_angles.pitch;

    attitude.yaw = attitude.yaw + imu.gyro_z * dt;

    return attitude;
}

Attitude ComplementaryFilter::getAttitude() const {
    return attitude;

}

