#include "complementary_filter.h"
#include "pid_controller.h"
#include "motor_mixer.h"

// Top-level function for HLS synthesis
void drone_controller(
    // Input interfaces
    fp16_t accel_x, fp16_t accel_y, fp16_t accel_z,
    fp16_t gyro_x,  fp16_t gyro_y,  fp16_t gyro_z,
    fp16_t mag_x,   fp16_t mag_y,   fp16_t mag_z,
    fp16_t dt,
    fp16_t setpoint_roll, fp16_t setpoint_pitch,
    fp16_t setpoint_yaw,  fp16_t setpoint_throttle,
    // Output interfaces
    fp16_t& roll,   fp16_t& pitch,  fp16_t& yaw,
    fp16_t& motor1, fp16_t& motor2, fp16_t& motor3, fp16_t& motor4
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
    #pragma HLS INTERFACE ap_none port=setpoint_roll
    #pragma HLS INTERFACE ap_none port=setpoint_pitch
    #pragma HLS INTERFACE ap_none port=setpoint_yaw
    #pragma HLS INTERFACE ap_none port=setpoint_throttle
    #pragma HLS INTERFACE ap_vld port=roll
    #pragma HLS INTERFACE ap_vld port=pitch
    #pragma HLS INTERFACE ap_vld port=yaw
    #pragma HLS INTERFACE ap_vld port=motor1
    #pragma HLS INTERFACE ap_vld port=motor2
    #pragma HLS INTERFACE ap_vld port=motor3
    #pragma HLS INTERFACE ap_vld port=motor4

    // Static instances (maintain state between calls)
    static ComplementaryFilter attitude_estimator(fp16_t(0.98), fp16_t(0.01));

    // PID controllers for each axis
    static PIDParams roll_params  = {fp16_t(2.5), fp16_t(0.5), fp16_t(0.1), fp16_t(5.0), fp16_t(1.0)};
    static PIDParams pitch_params = {fp16_t(2.5), fp16_t(0.5), fp16_t(0.1), fp16_t(5.0), fp16_t(1.0)};
    static PIDParams yaw_params   = {fp16_t(1.0), fp16_t(0.1), fp16_t(0.05), fp16_t(2.0), fp16_t(1.0)};

    static PIDController roll_pid(roll_params, dt);
    static PIDController pitch_pid(pitch_params, dt);
    static PIDController yaw_pid(yaw_params, dt);

    static MotorMixer mixer;

    // Create IMU data structure
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

    // ======================================
    // Step 1: Estimate attitude
    // ======================================
    Attitude attitude = attitude_estimator.update(imu);

    // DEBUG: Print attitude estimation results
    #ifndef __SYNTHESIS__
    static int debug_counter = 0;
    if (debug_counter % 10 == 0 || debug_counter == 30 || debug_counter == 31) {
        printf("\n[DEBUG %d] ATTITUDE ESTIMATION \n", debug_counter);
        printf("  Input Accel: ax=%f, ay=%f, az=%f\n",
               accel_x.to_double(), accel_y.to_double(), accel_z.to_double());
        printf("  Input Gyro: gx=%f, gy=%f, gz=%f\n",
               gyro_x.to_double(), gyro_y.to_double(), gyro_z.to_double());
        printf("  Estimated Attitude: roll=%f, pitch=%f, yaw=%f rad\n",
               attitude.roll.to_double(), attitude.pitch.to_double(), attitude.yaw.to_double());
        printf("  Estimated Attitude: roll=%f, pitch=%f, yaw=%f deg\n",
               attitude.roll.to_double()*57.2958,
               attitude.pitch.to_double()*57.2958,
               attitude.yaw.to_double()*57.2958);
    }
    #endif

    // ======================================
    // Step 2: Run PID controllers
    // ======================================
    fp16_t roll_output = roll_pid.update(setpoint_roll, attitude.roll);
    fp16_t pitch_output = pitch_pid.update(setpoint_pitch, attitude.pitch);
    fp16_t yaw_output = yaw_pid.update(setpoint_yaw, attitude.yaw);

    // DEBUG: Print PID calculations
    #ifndef __SYNTHESIS__
    if (debug_counter % 10 == 0 || debug_counter == 30 || debug_counter == 31) {
        printf("[DEBUG %d] PID CONTROL \n", debug_counter);
        printf("  Setpoints: roll=%f, pitch=%f, yaw=%f\n",
               setpoint_roll.to_double(), setpoint_pitch.to_double(), setpoint_yaw.to_double());
        printf("  Errors: roll_err=%f, pitch_err=%f, yaw_err=%f\n",
               (setpoint_roll - attitude.roll).to_double(),
               (setpoint_pitch - attitude.pitch).to_double(),
               (setpoint_yaw - attitude.yaw).to_double());
        printf("  PID Outputs: roll=%f, pitch=%f, yaw=%f\n",
               roll_output.to_double(), pitch_output.to_double(), yaw_output.to_double());

        // Check if PID is responding
        if (abs((setpoint_roll - attitude.roll).to_double()) > 0.01 &&
            abs(roll_output.to_double()) < 0.001) {
            printf("  WARNING: Roll PID not responding to error!\n");
        }
    }
    #endif

    // Step 3: Mix controls to motor commands
    ControlOutput motors = mixer.mix(setpoint_throttle, roll_output,
                                     pitch_output, yaw_output);

    // DEBUG: Print motor mixing results
    #ifndef __SYNTHESIS__
    if (debug_counter % 10 == 0 || debug_counter == 30 || debug_counter == 31) {
        printf("[DEBUG %d] MOTOR MIXING \n", debug_counter);
        printf("  Throttle: %f\n", setpoint_throttle.to_double());
        printf("  Control inputs: roll=%f, pitch=%f, yaw=%f\n",
               roll_output.to_double(), pitch_output.to_double(), yaw_output.to_double());
        printf("  Motor outputs: M1=%f, M2=%f, M3=%f, M4=%f\n",
               motors.motor1.to_double(), motors.motor2.to_double(),
               motors.motor3.to_double(), motors.motor4.to_double());
        printf("  Motor diff (roll): (M1+M4)-(M2+M3) = %f\n",
               (motors.motor1 + motors.motor4 - motors.motor2 - motors.motor3).to_double());
        printf("  Motor diff (pitch): (M3+M4)-(M1+M2) = %f\n",
               (motors.motor3 + motors.motor4 - motors.motor1 - motors.motor2).to_double());
    }
    debug_counter++;
    #endif


    roll = attitude.roll;
    pitch = attitude.pitch;
    yaw = attitude.yaw;
    motor1 = motors.motor1;
    motor2 = motors.motor2;
    motor3 = motors.motor3;
    motor4 = motors.motor4;

    // FINAL DEBUG: Check for stuck values
    #ifndef __SYNTHESIS__
    static fp16_t last_roll = fp16_t(0);
    static int stuck_counter = 0;

    if (abs((attitude.roll - last_roll).to_double()) < 0.0001) {
        stuck_counter++;
        if (stuck_counter > 50 && debug_counter % 50 == 0) {
            printf("\n*** WARNING: Roll has been stuck at %f for %d iterations! ***\n",
                   attitude.roll.to_double(), stuck_counter);
        }
    } else {
        stuck_counter = 0;
    }
    last_roll = attitude.roll;
    #endif
}

