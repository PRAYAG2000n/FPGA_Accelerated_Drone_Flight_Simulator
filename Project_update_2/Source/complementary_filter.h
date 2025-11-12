#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "types.h"

// Complementary Filter Class
class ComplementaryFilter {
private:
    fp16_t alpha;           // Filter coefficient (0.98 typical)
    fp16_t dt;              // Sampling time (seconds)
    Attitude attitude;      // Current attitude estimate
    
public:
    // Constructor
    ComplementaryFilter(fp16_t alpha_val = 0.98f, fp16_t dt_val = 0.01f);
    
    // Initialize filter
    void init();
    
    // Update attitude estimate with new IMU data
    Attitude update(const IMUData& imu);
    
    // Get current attitude
    Attitude getAttitude() const;
    
    // Calculate angle from accelerometer
    Attitude calculateAccelAngles(fp16_t accel_x, fp16_t accel_y, fp16_t accel_z);
};

#endif
