#ifndef QUADCOPTER_CONTROL_H
#define QUADCOPTER_CONTROL_H

#include <ap_int.h>
#include <ap_fixed.h>
#include <hls_stream.h>

// Define data types for fixed-point arithmetic
typedef ap_fixed<32,16> fp32_t;
typedef ap_fixed<16,8> sensor_data_t;
typedef ap_fixed<16,8> control_signal_t;
typedef ap_int<8> key_input_t;

// IMU Data structure
struct IMUData {
    sensor_data_t accel_x, accel_y, accel_z;
    sensor_data_t gyro_x, gyro_y, gyro_z;
    sensor_data_t mag_x, mag_y, mag_z;
};

// State vector structure - 6-DOF as per proposal
struct StateVector {
    fp32_t roll, pitch, yaw;           // Orientation
    fp32_t pos_x, pos_y, pos_z;        // Position
    fp32_t vel_x, vel_y, vel_z;        // Linear velocity
    fp32_t ang_vel_x, ang_vel_y, ang_vel_z; // Angular velocity
};

// Motor control structure
struct MotorSpeeds {
    control_signal_t front_left;
    control_signal_t front_right;
    control_signal_t rear_left;
    control_signal_t rear_right;
};

// Control commands structure
struct ControlCommands {
    control_signal_t throttle;
    control_signal_t roll_cmd;
    control_signal_t pitch_cmd;
    control_signal_t yaw_cmd;
    bool emergency_stop;
};

// System parameters structure
struct SystemParams {
    fp32_t mass;
    fp32_t inertia_xx, inertia_yy, inertia_zz;
    fp32_t motor_thrust_coeff;
    fp32_t motor_torque_coeff;
    fp32_t arm_length;
    fp32_t sampling_period;
    fp32_t gravity;
};

// Function prototypes
void complementary_filter(IMUData imu, StateVector& state, fp32_t alpha);
void pid_controller(control_signal_t error, control_signal_t integral,
                   control_signal_t previous_error, control_signal_t& output,
                   control_signal_t kp, control_signal_t ki, control_signal_t kd);
void flight_controller(StateVector current_state, ControlCommands commands,
                      MotorSpeeds& motor_speeds, SystemParams params);
void motor_mixer(control_signal_t throttle, control_signal_t roll_ctrl,
                control_signal_t pitch_ctrl, control_signal_t yaw_ctrl,
                MotorSpeeds& motor_speeds);
void process_keyboard_input(key_input_t key, ControlCommands& commands);
void attitude_controller(StateVector current_state, ControlCommands commands,
                        control_signal_t& roll_ctrl, control_signal_t& pitch_ctrl,
                        control_signal_t& yaw_ctrl, control_signal_t& throttle_ctrl);
void safety_monitor(MotorSpeeds& motor_speeds, StateVector& state,
                   bool& emergency_stop, SystemParams params);
void altitude_controller(StateVector current_state, ControlCommands commands,
                        control_signal_t& throttle_output, SystemParams params);

// 6-DOF Physics simulation for host
void six_dof_physics(MotorSpeeds motors, IMUData& sensors, StateVector& state, SystemParams params);

// Top-level function with AXI-Stream interfaces
void quadcopter_system(
    hls::stream<IMUData>& imu_input,
    hls::stream<key_input_t>& command_input,
    hls::stream<StateVector>& state_output,
    hls::stream<MotorSpeeds>& motor_output,
    SystemParams params
);

#endif
