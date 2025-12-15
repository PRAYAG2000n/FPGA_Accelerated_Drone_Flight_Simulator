# FPGA-Accelerated Drone Flight Controller

**Course**: EECE 5698 – FPGA in the Cloud  
**Author**: Prayag Sridhar  
**Academic Year**: Fall 2025  
**Platform**: Xilinx Vitis 2023.2 / Alveo U280

## Project Overview

This project implements a complete real-time quadcopter flight control system on FPGA hardware, featuring sensor fusion, PID control, motor mixing, and hardware deployment on cloud FPGA instances. The system demonstrates how hardware acceleration significantly improves the performance of computationally intensive control algorithms compared to traditional software implementations, achieving deterministic microsecond-level latency critical for stable drone flight.

**Current Status**: Final Project Complete  
**Hardware Target**: Alveo U280 @ 175 MHz  
**Release**: [v1.0 - Quadcopter Vitis Build for Alveo U280](https://github.com/PRAYAG2000n/FPGA_Accelerated_Drone_Flight_Simulator/releases/tag/v1.0)

## System Architecture

```
IMU Sensors ====> Complementary Filter ====> PID Controller ====> Motor Mixer ====> Motors
(Accel+Gyro)      (Attitude Estimation)      (Error→Torque)      (Torque→PWM)     (M1-M4)
                        |                           |                   |
                  Roll, Pitch, Yaw            Control Signals      Individual
                    (radians)                  (PID output)       Motor Commands
```

## Project Milestones

### Project Update 1 - Attitude Estimation 
**Status**: Complete  

**Features Implemented**:
- Complementary filter for sensor fusion
- Real-time attitude estimation at 137 Hz
- HLS-optimized implementation

**Performance**:
- Operating Frequency: 137.99 MHz
- Latency: 0.2 - 2.4 microseconds
- Resource Usage: < 2% FPGA

---

### Project Update 2 - PID Control System 
**Status**: Complete  

**Features Implemented**:
- Three-axis PID controllers (Roll, Pitch, Yaw)
- Motor mixing matrix for X-configuration quadcopter
- Fixed-point arithmetic implementation (`ap_fixed<16,8>`)
- Integrated control pipeline from sensors to motors
- Comprehensive test framework with CSV logging

**Performance**:
- Operating Frequency: 137.2 MHz (37% above target)
- Total Pipeline Latency: 1.12 microseconds (112 clock cycles)
- Resource Usage: DSP 26.1%, FF 19.1%, LUT 4.2%

---

### Preliminary Report 
**Status**: Complete

**Deliverables**:
- System design documentation
- Initial performance analysis
- Integration planning

---

### Final Project - Alveo U280 Deployment 
**Status**: Complete  
**Release**: v1.0

**Features Implemented**:
- Complete flight controller deployed on Alveo U280
- Operating at 175 MHz
- Full Vitis build with xclbin generation
- Cloud-ready deployment package

**Release Assets**:
- `quadcopter_system_175mhz.xclbin` - FPGA bitstream
- `vivado_project.zip` - Complete Vivado project files

## Performance Summary

| Metric | Software (ARM) | FPGA (Final) | Improvement |
|--------|----------------|--------------|-------------|
| Frequency | 50 MHz | 175 MHz | 3.5x |
| Latency | ~150 μs | < 2 μs | 75x+ |
| Determinism | Variable | Fixed | ✓ |
| Power | 5 W | ~2.5 W | 2x efficiency |

## Repository Structure

```
FPGA_Accelerated_Drone_Flight_Simulator/
├── Project_update_1/        # Attitude estimation (complementary filter)
├── Project_update_2/        # PID control implementation
├── Preliminary_Report/      # Design documentation & analysis
├── Final_Project/           # Alveo U280 deployment
└── README.md                # This file
```

## Build Instructions

### Prerequisites
- Xilinx Vitis 2023.2
- Alveo U280 platform files
- XRT (Xilinx Runtime)

### Using Pre-built Release
1. Download `quadcopter_system_175mhz.xclbin` from [Releases](https://github.com/PRAYAG2000n/FPGA_Accelerated_Drone_Flight_Simulator/releases/tag/v1.0)
2. Install XRT runtime
3. Load bitstream: `xbutil program -d 0 -u quadcopter_system_175mhz.xclbin`

### Building from Source
1. Download `vivado_project.zip` from Releases
2. Extract and open in Vivado 2023.2
3. Run synthesis and implementation
4. Generate bitstream

### HLS Development
1. Create .cpp and .h files in `src/`
2. Add testbench to `testbench/`
3. Set `drone_controller` as top function
4. Target clock: 10ns (100 MHz) or 5.7ns (175 MHz)
5. Run C simulation and C synthesis

## Test Results

| Test Scenario | Target | Achieved | Status |
|---------------|--------|----------|--------|
| Static Hover | 0° | < 1° | Stable |
| Roll Step Response | Fast | 0.4-0.5s rise | Excellent |
| Pitch Step Response | Fast | 0.4-0.5s rise | Excellent |
| Cross-coupling | < 5% | < 2% |  Excellent |
| Motor Commands | Valid PWM | 0-100% range | Excellent |

## Key Technical Achievements

1. **175 MHz Operation** - Exceeded 100 MHz target by 75%
2. **Sub-2μs Latency** - 75x faster than software implementation
3. **Deterministic Timing** - Fixed latency for real-time control
4. **Low Resource Usage** - Efficient HLS optimization
5. **Cloud-Ready** - Deployable on AWS F1 / Alveo platforms

## License

Academic use only - EECE 5698 Course Project, Northeastern University


