# FPGA_Accelerated_Drone_Flight_Simulator

Course: EECE 5698 - Advanced Hardware Description Languages
Author: Prayag Sridhar
Academic Year: Fall 2024
Platform: Xilinx Vitis HLS 2023.2 | Virtex UltraScale+ FPGA
---
## Project Overview
This project implements a real-time drone flight control system on FPGA hardware, starting with attitude estimation and progressively adding control algorithms and physics simulation. The goal is to demonstrate how hardware acceleration can significantly improve the performance of computationally intensive control systems compared to traditional software implementations.
Current Status: Project Update 1 
Complementary Filter for Attitude Estimation - Successfully implemented and synthesized
---
## System Architecture
___________________      __________________       _________________
|                 |     |                  |     |                 |
│   IMU Sensors   │====>│  Complementary   │====>│    Attitude     │
│  (Accel + Gyro) │     │     Filter       │     │  (Roll, Pitch,  │
|_________________|     │    [FPGA]        │     │      Yaw)       │
                        └──────────────────┘     |_________________|
## Project Milestones
Project Update 1 (Current) - Attitude Estimation
**Status**: Complete
**Branch**: project_update_1
**Features Implemented:**
- Complementary filter for sensor fusion
- Fuses accelerometer and gyroscope data
- Real-time attitude estimation at 100+ Hz
- HLS-optimized implementation

**Performance Achieved:**
- Operating Frequency: 137.99 MHz (38% above 100 MHz target)
- Latency: 0.2 - 2.4 μs per calculation
- Resource Usage: < 2% of FPGA resources
- Accuracy: ±0.25° steady-state error

**Files Structure:**
project_update_1/
 src/
 - complementary_filter.cpp    # Core filter implementation
 - complementary_filter.h      # Filter class header
 - drone_estimator.cpp         # HLS top function
 - types.h                     # Data structures
 tb/
 - test_complementary_filter.cpp # Comprehensive testbench
 results/
 - filter_output_data.csv      # Test results (800 samples)
 - synthesis_reports/          # HLS synthesis reports
docs/
  project_update_1_report.pdf # Detailed technical report


🔄 Project Update 2 (In Progress) - Control System
Target Date: November 2024
Branch: project_update_2 (coming soon)
Planned Features:

PID Controllers: Independent control loops for roll, pitch, yaw
Motor Mixing Matrix: Convert control signals to motor commands
Fixed-Point Optimization: Convert from floating to fixed-point arithmetic
Parallel Processing: Multi-axis control in parallel

Expected Improvements:

Resource reduction by 50% with fixed-point
Support for quadcopter (4 motors) configuration
Closed-loop control simulation

🚀 Project Update 3 (Future) - Advanced Estimation
Target Date: December 2024
Branch: project_update_3 (planned)
Planned Features:

Extended Kalman Filter (EKF): Advanced sensor fusion
Magnetometer Integration: Yaw drift correction
Sensor Noise Modeling: Realistic sensor characteristics
Fault Detection: Sensor failure handling

🌐 Final Project - Complete System Integration
Target Date: December 2024
Branch: main (final integration)
Planned Features:

Physics Simulation: 6-DOF dynamics model on host CPU
Hardware-in-the-Loop (HIL): Real-time FPGA-CPU communication
Visualization: 3D trajectory and attitude display
Performance Analysis: FPGA vs CPU comparison

🛠️ Build Instructions
Prerequisites

Xilinx Vitis HLS 2023.2
Vivado 2023.2 (for bitstream generation)
Python 3.8+ (for visualization)
Access to Virtex UltraScale+ FPGA (or simulation)

Quick Start

Clone the repository

bashgit clone https://github.com/yourusername/fpga-drone-simulator.git
cd fpga-drone-simulator
git checkout project_update_1

Run C Simulation
```
bash 
vitis_hls -f scripts/run_csim.tcl
```
Run Synthesis
```
bash
vitis_hls -f scripts/run_synthesis.tcl
```

Test Results
Current Implementation (Update 1)
Test ScenarioTargetAchievedErrorHover (0°)0°0.00°0.00%Roll30°29.92°0.27%Pitch20°19.95°0.25%DynamicTracking<2° lagGood
🔧 Development Roadmap
mermaidgantt
    title FPGA Drone Simulator Development Timeline
    dateFormat  YYYY-MM-DD
    section Completed
    Attitude Estimation    :done, 2024-10-01, 2024-10-31
    section In Progress
    PID Controllers        :active, 2024-11-01, 2024-11-15
    Motor Mixing          :2024-11-10, 2024-11-20
    section Planned
    Fixed-Point Convert   :2024-11-15, 2024-11-25
    Extended Kalman Filter :2024-11-20, 2024-12-05
    Physics Integration   :2024-12-01, 2024-12-15
    Final Testing        :2024-12-10, 2024-12-20
💻 Hardware Requirements
Minimum (Simulation Only)

8GB RAM
Vitis HLS 2023.2
Linux Ubuntu 20.04+ or Windows 10

Recommended (Hardware Testing)

Xilinx Virtex UltraScale+ FPGA
16GB RAM
JTAG programmer
Real IMU sensor (optional)

📈 Performance Metrics
MetricSoftware (ARM)FPGA (Current)ImprovementFrequency~50 MHz137.99 MHz2.76×Latency~150 μs2.4 μs62.5×Power~5W~2W2.5×DeterministicNoYes✓
🤝 Contributing
This is an academic project for EECE 5698. While not open for direct contributions, feedback and suggestions are welcome through issues.
📚 References

Madgwick, S. (2010). "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
Xilinx Vitis HLS User Guide (UG1399)
"Hardware-in-the-loop simulation of UAV autonomous landing" - Szolc & Kryjak (2022)

📝 License
This project is part of academic coursework at Northeastern University. Code is provided for educational purposes.
📧 Contact
Author: Prayag Sridhar
Email: sridhar.pr@northeastern.edu
Course Instructor: [Instructor Name]
🔄 Version History
VersionDateDescriptionv0.1.0Oct 31, 2024Initial release - Complementary Filterv0.2.0Nov 2024PID Controllers (coming soon)v0.3.0Dec 2024EKF Implementation (planned)v1.0.0Dec 2024Final integrated system (planned)

📌 Notes

This repository will be continuously updated throughout Fall 2024
Each project update will be tagged and preserved in separate branches
Final integration will be merged to main branch in December 2024
Performance benchmarks are from Virtex UltraScale+ xcvu35p FPGA

⚠️ Known Issues

CSV output files generated in solution1/csim/build/ directory
Matplotlib not available on NERC cluster VMs
SCP file transfers may timeout on cluster

🎓 Academic Integrity
This code is submitted as coursework for EECE 5698. Please follow your institution's academic integrity policies if referencing this work.
