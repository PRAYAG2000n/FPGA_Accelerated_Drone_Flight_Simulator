#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include "types.h"

class MotorMixer {
private:
    // Quad-X configuration mixing matrix
    static const fp16_t MIXING_MATRIX[4][4];

public:
    // Convert control outputs to motor commands
    ControlOutput mix(fp16_t throttle, fp16_t roll_ctrl,
                     fp16_t pitch_ctrl, fp16_t yaw_ctrl);

    // Normalize motor outputs to [0, 1] range
    void normalize(ControlOutput& motors);
};

#endif
