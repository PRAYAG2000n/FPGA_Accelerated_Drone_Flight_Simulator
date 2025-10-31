#include "complementary_filter.h"

void drone_estimator(
    // Input interfaces
    fp16_t accel_x, fp16_t accel_y, fp16_t accel_z,
    fp16_t gyro_x,  fp16_t gyro_y,  fp16_t gyro_z,
    fp16_t mag_x,   fp16_t mag_y,   fp16_t mag_z,
    fp16_t dt,

    fp16_t& roll,   fp16_t& pitch,  fp16_t& yaw
) {
    #pragma HLS INTERFACE ap_ctrl_none port=return
    #pragma HLS INTERFACE ap_none port=accel_x
    #pragma HLS INTERFACE ap_none port=accel_y
    #pragma HLS INTERFACE ap_none port=accel_z
    #pragma HLS INTERFACE ap_none port=gyro_x
    #pragma HLS INTERFACE ap_none port=gyro_y
    #pragma HLS INTERFACE ap_none port=gyro_z
    #pragma HLS INTERFACE ap_none port=mag_x
    #pragma HLS INTERFACE ap_none port=mag_y
    #pragma HLS INTERFACE ap_none port=mag_z
    #pragma HLS INTERFACE ap_none port=dt
    #pragma HLS INTERFACE ap_vld port=roll
    #pragma HLS INTERFACE ap_vld port=pitch
    #pragma HLS INTERFACE ap_vld port=yaw

    static ComplementaryFilter filter(0.98f, 0.01f);

    IMUData imu;
    imu.accel_x = accel_x;
    imu.accel_y = accel_y;
    imu.accel_z = accel_z;
    imu.gyro_x = gyro_x;
    imu.gyro_y = gyro_y;
    imu.gyro_z = gyro_z;
    imu.mag_x = mag_x;
    imu.mag_y = mag_y;
    imu.mag_z = mag_z;


    Attitude attitude = filter.update(imu);


    roll = attitude.roll;
    pitch = attitude.pitch;
    yaw = attitude.yaw;

}
