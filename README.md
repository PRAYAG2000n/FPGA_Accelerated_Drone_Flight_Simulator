# FPGA-Accelerated Drone Flight Controller
**Course**: EECE 5698 – FPGA in the cloud
**Author**: Prayag Sridhar  
**Academic Year**: Fall 2025  
**Platform**: Xilinx Vitis HLS 2023.2

## Project Overview
This project implements a complete real-time quadcopter flight control system on FPGA hardware, featuring sensor fusion, PID control, and motor mixing. The system demonstrates how hardware acceleration significantly improves the performance of computationally intensive control algorithms compared to traditional software implementations, achieving deterministic microsecond-level latency critical for stable drone flight.

**Current Status**: Project Update 2  
**Features**: Attitude Estimation + PID Control + Motor Mixing Successfully Implemented

## System Architecture
```
IMU Sensors ====> Complementary Filter ====> PID Controller ====> Motor Mixer ====> Motors
(Accel+Gyro)      (Attitude Estimation)      (Error→Torque)      (Torque→PWM)     (M1-M4)
                        |                           |                   |
                  Roll, Pitch, Yaw            Control Signals      Individual
                    (radians)                  (PID output)       Motor Commands
```

## Project Milestones

###  Project Update 1 - Attitude Estimation
**Status**: Complete  
**Features Implemented**:
- Complementary filter for sensor fusion
- Real-time attitude estimation at 137 Hz
- HLS-optimized implementation

**Performance**:
- Operating Frequency: 137.99 MHz
- Latency: 0.2 - 2.4 μs
- Resource Usage: < 2% FPGA

###  Project Update 2 - PID Control System (Current)
**Status**: Complete  
**Branch**: `Project_update_2`  
**Features Implemented**:
- Three-axis PID controllers (Roll, Pitch, Yaw)
- Motor mixing matrix for X-configuration quadcopter
- Fixed-point arithmetic implementation (`ap_fixed<16,8>`)
- Integrated control pipeline from sensors to motors
- Comprehensive test framework with CSV logging

**Performance Achieved**:
- **Operating Frequency**: 137.2 MHz (37% above target)
- **Total Pipeline Latency**: 1.12 microseconds (112 clock cycles)
- **Resource Usage**:
  - DSP: 26.1% (48,160 blocks)
  - FF: 19.1% (32,915)
  - LUT: 4.2% (18,427)
  - BRAM: 0%
- **Control Performance**:
  - Rise Time: 0.4-0.5 seconds
  - Steady-State Error: 11 degree roll, 7 degree pitch (requires integral tuning)
  - Update Rate: 100 Hz sustained

**File Structure**:
```
Project_update_2/
├── solution1/
│   ├── csim/
│   │   └── build/          # C simulation files
│   └── syn/
│       └── report/         # Synthesis reports
├── src/
│   ├── drone_controller.cpp       
│   ├── complementary_filter.cpp   
│   ├── complementary_filter.h     
|   ├── motor_mixer.cpp
|   ├── motor_mixer.h
|   ├── pid_controller.cpp
|   ├── pid_controller.h
│   └── types.h                    # Data structures
├── testbench/
│   └── test_drone_controller.cpp  # Comprehensive test suite
├── Result/
│   ├── controller_output.csv      # Test data (100 steps)
│   └── tb/                        # Testbench outputs
└── docs/
    └── project_update_2_report.pdf # Technical report
```

###  Project Update 3 - Advanced Control (Planned)
**Target Date**: December 2025  
**Planned Features**:
- Integral control with anti-windup
- Extended Kalman Filter (15-state)
- Physics simulation integration (Gazebo)
- Adaptive control and gain scheduling
- Trajectory following capability
- Hardware-in-the-Loop testing

## Test Results

### Current Implementation (Update 2)

| Test Scenario | Target | Achieved | Error | Status |
|--------------|---------|----------|-------|---------|
| Static Hover | 0° | 0.45° | 0.45° | Stable |
| Roll Step (11.4°) | 0.199 rad | 0.008 rad | 0.191 rad |  Needs I-gain |
| Pitch Step (5.6°) | 0.098 rad | -0.023 rad | 0.121 rad |  Needs I-gain |
| Motor Saturation | <80% | 95.7% (M1) | - |  Near limit |
| Cross-coupling | <5% | <2% | - |  Excellent |

### Performance Comparison

| Metric | Software (ARM) | FPGA Update 1 | FPGA Update 2 | Improvement |
|--------|---------------|---------------|---------------|-------------|
| Frequency | 50 MHz | 137.99 MHz | 137.2 MHz | 2.7x |
| Latency | ~150 microsecond | 2.4 microseconds | 1.12 microseconds | 134x |
| Determinism | Variable | Fixed | Fixed |  |
| Power | 5 W | 2 W | 2.5 W | 2x |

## Build Instructions

### Prerequisites
- Xilinx Vitis HLS 2023.2
- Versal Premium (xcvp2802-vsva5601-3HP-e-S)

### Implementation Steps
- Create the .cpp and .h files and copy paste the codes and label them appropriately
- All source codes should be added to src/ and test bench should be added to test bench part of vitis
- Set drone_controller as top function and target clock to 10ns.
- Run C simulation and C synthesis

## Known Issues and Future Work

### Current Limitations
- Steady-state error due to missing integral control
- Motor saturation during aggressive maneuvers
- Fixed-point quantization limiting small angle resolution

### Next Steps
1. Implement integral control with anti-windup
2. Optimize PID gains using Ziegler-Nichols method
3. Add Extended Kalman Filter for better state estimation
4. Integrate physics simulation for realistic testing
5. Develop adaptive control for varying flight conditions

## Repository Structure
```
FPGA_Accelerated_Drone_Flight_Simulator/
├── Project_update_1/        # Attitude estimation only
├── Project_update_2/        # PID control implementation
├── scripts/                 # Analysis and build scripts
├── docs/                    # Reports and documentation
└── README.md               # This file
```


## License
Academic use only - EECE 5698 Course Project

