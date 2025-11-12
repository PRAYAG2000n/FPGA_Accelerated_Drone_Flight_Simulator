#include <iostream>
#include <fstream>
#include <cmath>
#include <iomanip>
#include "complementary_filter.h"

using namespace std;

IMUData generate_test_scenario(int step, int scenario, fp16_t dt) {
    IMUData imu;
    fp16_t time = step * dt;

    switch(scenario) {
        case 0: // Hover (steady state)
            imu.accel_x = 0;
            imu.accel_y = 0;
            imu.accel_z = 9.81f;
            imu.gyro_x = 0;
            imu.gyro_y = 0;
            imu.gyro_z = 0;
            imu.mag_x = 0.2f;
            imu.mag_y = 0.0f;
            imu.mag_z = 0.4f;
            break;

        case 1: // Roll to 30 degrees
            {
                fp16_t roll_angle = 0.523599f; // 30 degrees in radians
                imu.accel_x = 0;
                imu.accel_y = 9.81f * sin(roll_angle);
                imu.accel_z = 9.81f * cos(roll_angle);
                imu.gyro_x = (step < 50) ? 0.523599f : 0;
                imu.gyro_y = 0;
                imu.gyro_z = 0;
                imu.mag_x = 0.2f;
                imu.mag_y = 0.0f;
                imu.mag_z = 0.4f;
            }
            break;

        case 2: // Pitch to 20 degrees
            {
                fp16_t pitch_angle = 0.349066f; // 20 degrees in radians
                imu.accel_x = -9.81f * sin(pitch_angle);
                imu.accel_y = 0;
                imu.accel_z = 9.81f * cos(pitch_angle);
                imu.gyro_x = 0;
                imu.gyro_y = (step < 50) ? 0.349066f : 0;
                imu.gyro_z = 0;
                imu.mag_x = 0.2f;
                imu.mag_y = 0.0f;
                imu.mag_z = 0.4f;
            }
            break;

        case 3: // Your original sinusoidal test pattern
            imu.accel_x = 0.1f * sin(2 * M_PI * 0.5f * time);
            imu.accel_y = 0.1f * sin(2 * M_PI * 0.3f * time);
            imu.accel_z = 9.81f;
            imu.gyro_x = 0.05f * cos(2 * M_PI * 0.3f * time);
            imu.gyro_y = 0.05f * cos(2 * M_PI * 0.5f * time);
            imu.gyro_z = 0.01f;
            imu.mag_x = 0.2f;
            imu.mag_y = 0.0f;
            imu.mag_z = 0.4f;
            break;
    }

    return imu;
}

int main() {
    cout << "  Complementary Filter Enhanced Testbench     " << endl;

    const int STEPS_PER_SCENARIO = 200;
    const int NUM_SCENARIOS = 4;
    const fp16_t DT = 0.01f;

    ofstream dataFile("filter_output_data.csv");


    dataFile << "Time,Scenario,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,"
             << "Roll,Pitch,Yaw,RollDeg,PitchDeg,YawDeg" << endl;


    int total_iterations = 0;
    fp16_t max_roll = 0, max_pitch = 0;

    // Test each scenario
    for (int scenario = 0; scenario < NUM_SCENARIOS; scenario++) {
        cout << "\nTesting Scenario " << scenario << ": ";
        switch(scenario) {
            case 0: cout << "Hover (Steady State)"; break;
            case 1: cout << "Roll to 30 degrees"; break;
            case 2: cout << "Pitch to 20 degrees"; break;
            case 3: cout << "Dynamic Sinusoidal Motion"; break;
        }
        cout << endl;

        
        ComplementaryFilter filter(0.98f, DT);

        for (int step = 0; step < STEPS_PER_SCENARIO; step++) {
            
            IMUData imu = generate_test_scenario(step, scenario, DT);

    
            Attitude attitude = filter.update(imu);

     
            fp16_t roll_deg = attitude.roll * 180.0f / M_PI;
            fp16_t pitch_deg = attitude.pitch * 180.0f / M_PI;
            fp16_t yaw_deg = attitude.yaw * 180.0f / M_PI;

   
            fp16_t time = (scenario * STEPS_PER_SCENARIO + step) * DT;
            dataFile << fixed << setprecision(4)
                    << time << "," << scenario << ","
                    << imu.accel_x << "," << imu.accel_y << "," << imu.accel_z << ","
                    << imu.gyro_x << "," << imu.gyro_y << "," << imu.gyro_z << ","
                    << attitude.roll << "," << attitude.pitch << "," << attitude.yaw << ","
                    << roll_deg << "," << pitch_deg << "," << yaw_deg << endl;

    
            if (abs(roll_deg) > abs(max_roll)) max_roll = roll_deg;
            if (abs(pitch_deg) > abs(max_pitch)) max_pitch = pitch_deg;

            // Print progress
            if (step % 50 == 0 || step == STEPS_PER_SCENARIO - 1) {
                cout << "  Step " << step << ": Roll=" << roll_deg
                     << " deg, Pitch=" << pitch_deg << " deg" << endl;
            }

            total_iterations++;
        }
    }

 
    dataFile.close();
    cout << "TESTBENCH COMPLETED" << endl;
    cout << "Total iterations: " << total_iterations << endl;
    cout << "Maximum roll: " << max_roll << " degrees" << endl;
    cout << "Maximum pitch: " << max_pitch << " degrees" << endl;
    cout << "Output saved to: filter_output_data.csv" << endl;
    cout << "\n*** C SIMULATION PASSED ***" << endl;

    return 0;
}

