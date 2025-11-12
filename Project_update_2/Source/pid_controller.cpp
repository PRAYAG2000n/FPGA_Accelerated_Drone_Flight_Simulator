#include "pid_controller.h"

// Constructor
PIDController::PIDController(PIDParams params_val, fp16_t dt_val)
    : params(params_val), dt(dt_val) {
    init();
}

// Initialize controller
void PIDController::init() {
    prev_error = 0;
    integral = 0;
}

// Update PID controller
fp16_t PIDController::update(fp16_t setpoint, fp16_t current_value) {
    #pragma HLS PIPELINE II=1

    // Calculate error
    fp16_t error = setpoint - current_value;

    // Proportional term
    fp16_t proportional = params.Kp * error;

    // Integral term with anti-windup
    integral = integral + error * dt;

    // Apply integral limits
    if (integral > params.integral_limit) {
        integral = params.integral_limit;
    } else if (integral < -params.integral_limit) {
        integral = -params.integral_limit;
    }

    fp16_t integral_term = params.Ki * integral;

    // Derivative term
    fp16_t derivative = (error - prev_error) / dt;
    fp16_t derivative_term = params.Kd * derivative;

    // Update previous error
    prev_error = error;

    // Calculate output
    fp16_t output = proportional + integral_term + derivative_term;

    // Apply output limits
    if (output > params.output_limit) {
        output = params.output_limit;
    } else if (output < -params.output_limit) {
        output = -params.output_limit;
    }

    return output;
}

// Reset controller state
void PIDController::reset() {
    init();
}

// Set new parameters
void PIDController::setParams(PIDParams new_params) {
    params = new_params;
}
