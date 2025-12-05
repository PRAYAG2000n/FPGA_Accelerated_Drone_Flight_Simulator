#ifndef QUADCOPTER_CONTROL_H
#define QUADCOPTER_CONTROL_H

#include <ap_int.h>
#include <ap_fixed.h>
#include <hls_stream.h>

typedef ap_fixed<32,16> fp32_t;
typedef ap_fixed<16,8> sensor_data_t;
typedef ap_fixed<16,8> control_signal_t;
typedef ap_int<8> key_input_t;

struct IMUData {
    sensor_data_t accel_x;
    sensor_data_t accel_y;
    sensor_data_t accel_z;
    sensor_data_t gyro_x;
    sensor_data_t gyro_y;
    sensor_data_t gyro_z;
    sensor_data_t mag_x;
    sensor_data_t mag_y;
    sensor_data_t mag_z;
};

struct StateVector {
    fp32_t roll;
    fp32_t pitch;
    fp32_t yaw;
    fp32_t pos_x;
    fp32_t pos_y;
    fp32_t pos_z;
    fp32_t vel_x;
    fp32_t vel_y;
    fp32_t vel_z;
    fp32_t ang_vel_x;
    fp32_t ang_vel_y;
    fp32_t ang_vel_z;
};

struct MotorSpeeds {
    control_signal_t front_left;
    control_signal_t front_right;
    control_signal_t rear_left;
    control_signal_t rear_right;
};

struct ControlCommands {
    control_signal_t throttle;
    control_signal_t roll_cmd;
    control_signal_t pitch_cmd;
    control_signal_t yaw_cmd;
    bool emergency_stop;
};

struct SystemParams {
    fp32_t mass;
    fp32_t inertia_xx;
    fp32_t inertia_yy;
    fp32_t inertia_zz;
    fp32_t motor_thrust_coeff;
    fp32_t motor_torque_coeff;
    fp32_t arm_length;
    fp32_t sampling_period;
    fp32_t gravity;
};

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
void quadcopter_system(
    hls::stream<IMUData>& imu_input,
    hls::stream<key_input_t>& command_input,
    hls::stream<StateVector>& state_output,
    hls::stream<MotorSpeeds>& motor_output,
    SystemParams params
);

#endif
