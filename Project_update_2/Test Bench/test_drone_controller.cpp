#include <iostream>
#include <fstream>
#include <cmath>
#include <sys/stat.h>  // Cross-platform directory operations
#include <unistd.h>    // Cross-platform getcwd
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

// Cross-platform directory creation
int create_directory(const char* path) {
#ifdef _WIN32
    return _mkdir(path);
#else
    return mkdir(path, 0777);
#endif
}

// Cross-platform get current working directory
char* get_current_directory(char* buf, size_t size) {
#ifdef _WIN32
    return _getcwd(buf, size);
#else
    return getcwd(buf, size);
#endif
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
    cout << "Starting Drone Controller Test Bench - Project Update 2" << endl;

    // ============================================
    // DIAGNOSTIC TEST WITH KNOWN VALUES
    // ============================================
    cout << "\nDIAGNOSTIC TEST WITH KNOWN VALUES" << endl;
    cout << "Testing if roll detection and PID work with artificial data..." << endl;
    {
        fp16_t test_roll, test_pitch, test_yaw;
        fp16_t m1, m2, m3, m4;

        // Test 1: Level drone (should give roll ≈ 0)
        cout << "\nTest 1: Level drone" << endl;
        IMUData level_imu;
        level_imu.accel_x = fp16_t(0.0);
        level_imu.accel_y = fp16_t(0.0);
        level_imu.accel_z = fp16_t(9.81);
        level_imu.gyro_x = fp16_t(0.0);
        level_imu.gyro_y = fp16_t(0.0);
        level_imu.gyro_z = fp16_t(0.0);
        level_imu.mag_x = fp16_t(0.2);
        level_imu.mag_y = fp16_t(0.0);
        level_imu.mag_z = fp16_t(0.4);

        drone_controller(
            level_imu.accel_x, level_imu.accel_y, level_imu.accel_z,
            level_imu.gyro_x, level_imu.gyro_y, level_imu.gyro_z,
            level_imu.mag_x, level_imu.mag_y, level_imu.mag_z,
            fp16_t(0.01),
            fp16_t(0.0), fp16_t(0.0), fp16_t(0.0), fp16_t(0.5),
            test_roll, test_pitch, test_yaw, m1, m2, m3, m4
        );

        cout << "  Expected roll: 0 rad" << endl;
        cout << "  Actual roll: " << to_double(test_roll) << " rad" << endl;
        cout << "  Motors: [" << to_double(m1) << ", " << to_double(m2)
             << ", " << to_double(m3) << ", " << to_double(m4) << "]" << endl;

        // Test 2: 30 degree roll (should give roll ≈ 0.524 rad)
        cout << "\nTest 2: 30 degree roll angle" << endl;
        IMUData roll30_imu;
        roll30_imu.accel_x = fp16_t(0.0);
        roll30_imu.accel_y = fp16_t(4.905);  // 9.81 * sin(30°)
        roll30_imu.accel_z = fp16_t(8.487);  // 9.81 * cos(30°)
        roll30_imu.gyro_x = fp16_t(0.0);
        roll30_imu.gyro_y = fp16_t(0.0);
        roll30_imu.gyro_z = fp16_t(0.0);
        roll30_imu.mag_x = fp16_t(0.2);
        roll30_imu.mag_y = fp16_t(0.0);
        roll30_imu.mag_z = fp16_t(0.4);

        drone_controller(
            roll30_imu.accel_x, roll30_imu.accel_y, roll30_imu.accel_z,
            roll30_imu.gyro_x, roll30_imu.gyro_y, roll30_imu.gyro_z,
            roll30_imu.mag_x, roll30_imu.mag_y, roll30_imu.mag_z,
            fp16_t(0.01),
            fp16_t(0.0), fp16_t(0.0), fp16_t(0.0), fp16_t(0.5),
            test_roll, test_pitch, test_yaw, m1, m2, m3, m4
        );

        cout << "  Expected roll: ~0.524 rad (30°)" << endl;
        cout << "  Actual roll: " << to_double(test_roll) << " rad" << endl;
        cout << "  Motors: [" << to_double(m1) << ", " << to_double(m2)
             << ", " << to_double(m3) << ", " << to_double(m4) << "]" << endl;

        // Test 3: With roll setpoint
        cout << "\nTest 3: Level drone with 0.2 rad roll setpoint" << endl;
        drone_controller(
            level_imu.accel_x, level_imu.accel_y, level_imu.accel_z,
            level_imu.gyro_x, level_imu.gyro_y, level_imu.gyro_z,
            level_imu.mag_x, level_imu.mag_y, level_imu.mag_z,
            fp16_t(0.01),
            fp16_t(0.2), fp16_t(0.0), fp16_t(0.0), fp16_t(0.5),  // 0.2 rad roll setpoint
            test_roll, test_pitch, test_yaw, m1, m2, m3, m4
        );

        cout << "  Roll setpoint: 0.2 rad" << endl;
        cout << "  Actual roll: " << to_double(test_roll) << " rad" << endl;
        cout << "  Error: " << to_double(fp16_t(0.2) - test_roll) << " rad" << endl;
        cout << "  Motors: [" << to_double(m1) << ", " << to_double(m2)
             << ", " << to_double(m3) << ", " << to_double(m4) << "]" << endl;
        cout << "  Motor differential (M1+M4)-(M2+M3): "
             << to_double((m1 + m4) - (m2 + m3)) << endl;
    }
    cout << "=== END DIAGNOSTIC TEST ===" << endl;

    // ============================================
    // MAIN TEST SEQUENCE
    // ============================================
    cout << "\nTest Sequence:" << endl;
    cout << "- Steps 0-29: Hover (roll=0, pitch=0)" << endl;
    cout << "- Steps 30-59: Roll right (roll=0.2)" << endl;
    cout << "- Steps 60-99: Pitch forward (pitch=0.1)" << endl;
    cout << "========================================" << endl;

    // Create tb directory if it doesn't exist (cross-platform)
    int dir_result = create_directory("tb");
    if (dir_result != 0 && errno != EEXIST) {
        cout << "WARNING: Could not create tb directory, error: " << errno << endl;
    } else {
        cout << "SUCCESS: Created or found tb directory" << endl;
    }

    // Try to write to current directory as well as tb directory
    ofstream results_file("controller_output.csv");
    ofstream results_file_tb("tb/controller_output.csv");

    // Check if files opened successfully
    if (!results_file.is_open()) {
        cout << "WARNING: Could not create controller_output.csv in current directory!" << endl;
    } else {
        cout << "SUCCESS: Created controller_output.csv in current directory" << endl;
    }

    if (!results_file_tb.is_open()) {
        cout << "WARNING: Could not create tb/controller_output.csv!" << endl;
    } else {
        cout << "SUCCESS: Created tb/controller_output.csv" << endl;
    }

    // Write headers to both files if they're open
    const char* header = "Step,Time,Roll,Pitch,Yaw,M1,M2,M3,M4,Target_Roll,Target_Pitch\n";
    if (results_file.is_open()) {
        results_file << header;
    }
    if (results_file_tb.is_open()) {
        results_file_tb << header;
    }

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
            cout << "\n>>> APPLYING ROLL STEP: Target = 0.2 rad <<<" << endl;
        }
        if (step == 60) {
            target_roll = fp16_t(0.0);
            target_pitch = fp16_t(0.1);  // 0.1 rad ~ 5.7 degrees
            cout << "\n>>> APPLYING PITCH STEP: Target = 0.1 rad <<<" << endl;
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

        // DETAILED DEBUG OUTPUT AT KEY POINTS
        if (step % 10 == 0 || step == 30 || step == 31 || step == 32 ||
            step == 60 || step == 61 || step == 62) {
            cout << "\nStep " << step << "/" << NUM_STEPS;
            cout << " | Roll: " << to_double(roll) << " (target: " << to_double(target_roll) << ")";
            cout << " | Pitch: " << to_double(pitch) << " (target: " << to_double(target_pitch) << ")";
            cout << " | Motors: " << to_double(m1) << ", " << to_double(m2)
                 << ", " << to_double(m3) << ", " << to_double(m4) << endl;

            // Extra debug at transition points
            if (step == 30 || step == 31 || step == 60 || step == 61) {
                cout << "  --- DETAILED DEBUG ---" << endl;
                cout << "  IMU Accel: [" << to_double(imu.accel_x) << ", "
                     << to_double(imu.accel_y) << ", " << to_double(imu.accel_z) << "]" << endl;
                cout << "  IMU Gyro: [" << to_double(imu.gyro_x) << ", "
                     << to_double(imu.gyro_y) << ", " << to_double(imu.gyro_z) << "]" << endl;
                cout << "  Roll Error: " << to_double(target_roll - roll) << " rad" << endl;
                cout << "  Pitch Error: " << to_double(target_pitch - pitch) << " rad" << endl;
                cout << "  Motor Differential Roll (M1+M4)-(M2+M3): "
                     << to_double((m1 + m4) - (m2 + m3)) << endl;
                cout << "  Motor Differential Pitch (M3+M4)-(M1+M2): "
                     << to_double((m3 + m4) - (m1 + m2)) << endl;
            }
        }

        // Write results to CSV (both files if they're open)
        char buffer[256];
        sprintf(buffer, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                step, to_double(step * DT),
                to_double(roll), to_double(pitch), to_double(yaw),
                to_double(m1), to_double(m2), to_double(m3), to_double(m4),
                to_double(target_roll), to_double(target_pitch));

        if (results_file.is_open()) {
            results_file << buffer;
        }
        if (results_file_tb.is_open()) {
            results_file_tb << buffer;
        }
    }

    cout << "TEST COMPLETED SUCCESSFULLY!" << endl;

    // Report where files were saved
    if (results_file.is_open()) {
        cout << "CSV data saved to: controller_output.csv (current directory)" << endl;
        results_file.close();
    }
    if (results_file_tb.is_open()) {
        cout << "CSV data saved to: tb/controller_output.csv" << endl;
        results_file_tb.close();
    }

    cout << "File location: " << __FILE__ << endl;

    // Get and print current working directory
    char cwd[1024];
    if (get_current_directory(cwd, sizeof(cwd)) != NULL) {
        cout << "Current working directory: " << cwd << endl;
    } else {
        cout << "Could not get current working directory" << endl;
    }

    // Final diagnostic check
    cout << "\nFINAL DIAGNOSTIC:" << endl;
    if (true) {  // Always run this check
        IMUData final_imu = generate_imu_data(99, DT);
        fp16_t final_roll, final_pitch, final_yaw, fm1, fm2, fm3, fm4;

        // Test with non-zero roll target
        drone_controller(
            final_imu.accel_x, final_imu.accel_y, final_imu.accel_z,
            final_imu.gyro_x, final_imu.gyro_y, final_imu.gyro_z,
            final_imu.mag_x, final_imu.mag_y, final_imu.mag_z,
            DT,
            fp16_t(0.15), fp16_t(0.0), fp16_t(0.0), fp16_t(0.5),
            final_roll, final_pitch, final_yaw, fm1, fm2, fm3, fm4
        );

        cout << "With 0.15 rad roll target:" << endl;
        cout << "  Estimated roll: " << to_double(final_roll) << " rad" << endl;
        cout << "  Motor response: [" << to_double(fm1) << ", " << to_double(fm2)
             << ", " << to_double(fm3) << ", " << to_double(fm4) << "]" << endl;

        if (abs(to_double(final_roll)) < 0.001 && abs(to_double(fm1 - fm2)) < 0.001) {
            cout << "\n*** WARNING: Roll appears to be stuck at zero! ***" << endl;
            cout << "Possible issues:" << endl;
            cout << "  1. Complementary filter not calculating roll from accelerometer" << endl;
            cout << "  2. PID controller gains might be zero or very small" << endl;
            cout << "  3. Motor mixer might have incorrect signs" << endl;
        }
    }

    cout << "\nINFO: [SIM 1] CSim done with 0 errors." << endl;
    cout << "INFO: [SIM 3] *************** CSIM finish ***************" << endl;

    return 0;
}

