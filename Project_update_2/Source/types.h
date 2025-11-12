#ifndef TYPES_H
#define TYPES_H

#include <ap_fixed.h>

// Fixed-point data types for FPGA optimization
typedef ap_fixed<16,8> fp16_t;      // 8 integer bits, 8 fractional bits
typedef ap_fixed<32,16> fp32_t;     // 16 integer bits, 16 fractional bits

// IMU data structure
struct IMUData {
    fp16_t accel_x, accel_y, accel_z;
    fp16_t gyro_x, gyro_y, gyro_z;
    fp16_t mag_x, mag_y, mag_z;
};

// Attitude structure
struct Attitude {
    fp16_t roll, pitch, yaw;
};

// Control output structure
struct ControlOutput {
    fp16_t motor1, motor2, motor3, motor4;
};

// PID Controller structure
struct PIDParams {
    fp16_t Kp;      // Proportional gain
    fp16_t Ki;      // Integral gain
    fp16_t Kd;      // Derivative gain
    fp16_t integral_limit;  // Anti-windup limit
    fp16_t output_limit;    // Output saturation limit
};

// Desired setpoint structure
struct Setpoint {
    fp16_t roll, pitch, yaw, throttle;
};

#endif
