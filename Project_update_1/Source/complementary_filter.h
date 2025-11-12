#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "types.h"


class ComplementaryFilter {
private:
    fp16_t alpha;           // Filter coefficient (0.98 typical)
    fp16_t dt;              // Sampling time (seconds)
    Attitude attitude;      // Current attitude estimate
    
public:

    ComplementaryFilter(fp16_t alpha_val = 0.98f, fp16_t dt_val = 0.01f);
    
    void init();
    
    Attitude update(const IMUData& imu);
    
    Attitude getAttitude() const;
    
    Attitude calculateAccelAngles(fp16_t accel_x, fp16_t accel_y, fp16_t accel_z);
};

#endif
