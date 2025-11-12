#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "types.h"

class PIDController {
private:
    PIDParams params;
    fp16_t prev_error;
    fp16_t integral;
    fp16_t dt;

public:
    // Constructor
    PIDController(PIDParams params_val = {1.0, 0.0, 0.0, 10.0, 1.0},
                  fp16_t dt_val = 0.01);

    // Initialize controller
    void init();

    // Update PID controller
    fp16_t update(fp16_t setpoint, fp16_t current_value);

    // Reset controller state
    void reset();

    // Set new parameters
    void setParams(PIDParams new_params);
};

#endif
