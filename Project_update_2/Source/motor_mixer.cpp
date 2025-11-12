#include "motor_mixer.h"

// Quad-X mixing matrix
// [Throttle, Roll, Pitch, Yaw] to [M1, M2, M3, M4]
const fp16_t MotorMixer::MIXING_MATRIX[4][4] = {
    { 1.0, -1.0,  1.0, -1.0},  // Motor 1: FR
    { 1.0, -1.0, -1.0,  1.0},  // Motor 2: FL
    { 1.0,  1.0,  1.0,  1.0},  // Motor 3: BR
    { 1.0,  1.0, -1.0, -1.0}   // Motor 4: BL
};

// Convert control outputs to motor commands
ControlOutput MotorMixer::mix(fp16_t throttle, fp16_t roll_ctrl,
                             fp16_t pitch_ctrl, fp16_t yaw_ctrl) {
    #pragma HLS PIPELINE II=1

    ControlOutput motors;

    // Apply mixing matrix
    motors.motor1 = throttle * MIXING_MATRIX[0][0] +
                   roll_ctrl * MIXING_MATRIX[0][1] +
                   pitch_ctrl * MIXING_MATRIX[0][2] +
                   yaw_ctrl * MIXING_MATRIX[0][3];

    motors.motor2 = throttle * MIXING_MATRIX[1][0] +
                   roll_ctrl * MIXING_MATRIX[1][1] +
                   pitch_ctrl * MIXING_MATRIX[1][2] +
                   yaw_ctrl * MIXING_MATRIX[1][3];

    motors.motor3 = throttle * MIXING_MATRIX[2][0] +
                   roll_ctrl * MIXING_MATRIX[2][1] +
                   pitch_ctrl * MIXING_MATRIX[2][2] +
                   yaw_ctrl * MIXING_MATRIX[2][3];

    motors.motor4 = throttle * MIXING_MATRIX[3][0] +
                   roll_ctrl * MIXING_MATRIX[3][1] +
                   pitch_ctrl * MIXING_MATRIX[3][2] +
                   yaw_ctrl * MIXING_MATRIX[3][3];

    // Normalize outputs
    normalize(motors);

    return motors;
}

// Normalize motor outputs to [0, 1] range
void MotorMixer::normalize(ControlOutput& motors) {
    // Find maximum motor value
    fp16_t max_val = motors.motor1;
    if (motors.motor2 > max_val) max_val = motors.motor2;
    if (motors.motor3 > max_val) max_val = motors.motor3;
    if (motors.motor4 > max_val) max_val = motors.motor4;

    // Scale if any motor exceeds 1.0
    if (max_val > 1.0) {
        motors.motor1 = motors.motor1 / max_val;
        motors.motor2 = motors.motor2 / max_val;
        motors.motor3 = motors.motor3 / max_val;
        motors.motor4 = motors.motor4 / max_val;
    }

    // Ensure minimum value is 0
    if (motors.motor1 < 0) motors.motor1 = 0;
    if (motors.motor2 < 0) motors.motor2 = 0;
    if (motors.motor3 < 0) motors.motor3 = 0;
    if (motors.motor4 < 0) motors.motor4 = 0;
}
