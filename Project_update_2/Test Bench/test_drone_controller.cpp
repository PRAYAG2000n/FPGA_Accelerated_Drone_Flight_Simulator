#include <iostream>
#include <fstream>
#include <cmath>
#include "complementary_filter.h"
#include "pid_controller.h"
#include "motor_mixer.h"

// Include the top-level function declaration
extern void drone_controller(
    fp16_t accel_x, fp16_t accel_y, fp16_t accel_z,
    fp16_t gyro_x,  fp16_t gyro_y,  fp16_t gyro_z,
    fp16_t mag_x,   fp16_t mag_y,   fp16_t mag_z,
    fp16_t dt,
    fp16_t setpoint_roll, fp16_t setpoint_pitch,
    fp16_t setpoint_yaw,  fp16_t setpoint_throttle,
    fp16_t& roll,   fp16_t& pitch,  fp16_t& yaw,
    fp16_t& motor1, fp16_t& motor2, fp16_t& motor3, fp16_t& motor4
);

using namespace std;

// Helper function to convert to double for printing
double to_double(fp16_t value) {
    return value.to_double();
}

// Generate synthetic IMU data for testing
IMUData generate_imu_data(int step, fp16_t dt) {
    IMUData imu;

    // Simulate a hovering drone with small oscillations
    double time = to_double(step * dt);

    // Use explicit conversion
    imu.accel_x = fp16_t(0.1 * sin(2 * M_PI * 0.5 * time));
    imu.accel_y = fp16_t(0.1 * sin(2 * M_PI * 0.3 * time));
    imu.accel_z = fp16_t(9.81);  // Gravity

    imu.gyro_x = fp16_t(0.05 * cos(2 * M_PI * 0.3 * time));
    imu.gyro_y = fp16_t(0.05 * cos(2 * M_PI * 0.5 * time));
    imu.gyro_z = fp16_t(0.01);  // Small constant yaw rate

    imu.mag_x = fp16_t(0.2);
    imu.mag_y = fp16_t(0.0);
    imu.mag_z = fp16_t(0.4);

    return imu;
}

int main() {
    cout << "Starting Drone Controller Testbench" << endl;
    cout << "===================================" << endl;

    ofstream results_file("tb/controller_results.csv");
    results_file << "Step,Time,Roll,Pitch,Yaw,M1,M2,M3,M4,Target_Roll,Target_Pitch" << endl;

    const int NUM_STEPS = 100;
    const fp16_t DT = fp16_t(0.01);

    // Test scenario: Hover with step input
    fp16_t target_roll = fp16_t(0.0);
    fp16_t target_pitch = fp16_t(0.0);
    fp16_t target_yaw = fp16_t(0.0);
    fp16_t throttle = fp16_t(0.5);  // 50% throttle for hover

    for (int step = 0; step < NUM_STEPS; step++) {
        // Apply step input at step 30
        if (step == 30) {
            target_roll = fp16_t(0.2);  // 0.2 rad ~ 11.5 degrees
        }
        if (step == 60) {
            target_roll = fp16_t(0.0);
            target_pitch = fp16_t(0.1);  // 0.1 rad ~ 5.7 degrees
        }

        // Generate test data
        IMUData imu = generate_imu_data(step, DT);

        // Run drone controller
        fp16_t roll, pitch, yaw, m1, m2, m3, m4;
        drone_controller(
            imu.accel_x, imu.accel_y, imu.accel_z,
            imu.gyro_x, imu.gyro_y, imu.gyro_z,
            imu.mag_x, imu.mag_y, imu.mag_z,
            DT,
            target_roll, target_pitch, target_yaw, throttle,
            roll, pitch, yaw, m1, m2, m3, m4
        );

        // Write results
        results_file << step << "," << to_double(step * DT) << ","
                    << to_double(roll) << "," << to_double(pitch) << "," << to_double(yaw) << ","
                    << to_double(m1) << "," << to_double(m2) << "," << to_double(m3) << "," << to_double(m4) << ","
                    << to_double(target_roll) << "," << to_double(target_pitch) << endl;

        // Print progress
        if (step % 20 == 0) {
            cout << "Step " << step << "/" << NUM_STEPS
                 << " - Roll: " << to_double(roll) << " (target: " << to_double(target_roll) << ")"
                 << " - Motors: " << to_double(m1) << "," << to_double(m2)
                 << "," << to_double(m3) << "," << to_double(m4) << endl;
        }
    }

    cout << "\nTest Completed Successfully!" << endl;
    cout << "Results saved to tb/controller_results.csv" << endl;

    results_file.close();
    return 0;
}
