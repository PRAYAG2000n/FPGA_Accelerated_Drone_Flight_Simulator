#ifndef TYPES_H
#define TYPES_H


typedef float fp16_t;
typedef float fp32_t;


struct IMUData {
    fp16_t accel_x, accel_y, accel_z;  // Accelerometer data (m/s²)
    fp16_t gyro_x, gyro_y, gyro_z;     // Gyroscope data (rad/s)
    fp16_t mag_x, mag_y, mag_z;        // Magnetometer data (uT)
};


struct Attitude {
    fp16_t roll, pitch, yaw;           // Roll, pitch, yaw angles (radians)
};

// Control output structure
struct ControlOutput {
    fp16_t motor1, motor2, motor3, motor4;  // Motor PWM signals
};

#endif

