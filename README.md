# 🧠 FPGA-Accelerated Drone Flight Simulator  
**Course:** EECE 5698 – Advanced Hardware Description Languages  
**Author:** Prayag Sridhar  
**Academic Year:** Fall 2024  
**Platform:** Xilinx Vitis HLS 2023.2 | Alveo U280

---

## 📋 Project Overview  
This project implements a **real-time drone flight control system** on FPGA hardware, starting with attitude estimation and progressively adding control algorithms and physics simulation.  
The goal is to demonstrate how **hardware acceleration** can significantly improve the performance of computationally intensive control systems compared to traditional software implementations.

**Current Status:** ✅ *Project Update 1*  
**Feature:** Complementary Filter for Attitude Estimation — *Successfully implemented and synthesized*

---

## System Architecture
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   IMU Sensors   │────▶│  Complementary   │────▶│    Attitude     │
│  (Accel + Gyro) │     │     Filter       │     │  (Roll, Pitch,  │
└─────────────────┘     │    [FPGA]        │     │      Yaw)       │
                        └──────────────────┘     └─────────────────┘

---

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



---

### **Next Update – Integrated Control and Estimation System**  
**Target Date:** November – December 2024  
**Branch:** `next_update` *(in progress)*  

#### Planned Features  
- **Multi-Axis PID Control:** Independent loops for roll, pitch, yaw  
- **Motor Mixing Matrix:** Map control signals to motor PWM commands  
- **Fixed-Point Optimization:** Reduce resource usage and latency  
- **Extended Kalman Filter (EKF):** Advanced sensor fusion and noise modeling  
- **Magnetometer Integration:** Yaw drift correction and fault handling  
- **Physics Simulation (6-DOF):** CPU-side drone dynamics model  
- **Hardware-in-the-Loop (HIL):** Real-time FPGA↔CPU co-simulation  
- **Visualization:** 3D trajectory and attitude display via Python  
- **Performance Comparison:** FPGA vs CPU execution speed, latency, and power  

#### Expected Improvements  
| Area | Anticipated Gain |
|:------|:----------------|
| Resource Utilization | ↓ 50 % via fixed-point conversion |
| Latency | 10× faster control response |
| System Support | Full quadcopter (4-motor) stabilization |
| Fault Tolerance | EKF and sensor redundancy features |
| Visualization | Real-time Python dashboard for attitude tracking |

---

##  Build Instructions  

### Prerequisites  
- Xilinx Vitis HLS 2023.2  
- Vivado 2023.2 (for bitstream generation)  
- Python 3.8 + (for visualization)  
- U280

### Quick Start  
```bash
# Clone the repository
git clone https://github.com/yourusername/fpga-drone-simulator.git
cd fpga-drone-simulator
git checkout project_update_1

# Run C Simulation
vitis_hls -f scripts/run_csim.tcl

# Run Synthesis
vitis_hls -f scripts/run_synthesis.tcl

# View Results
python3 scripts/visualize_results.py
```
## Test results

### Current Implementation (Update 1)
Test Scenario	Target	Achieved	Error
Hover         	0°	   0.00°	  0.00 %
Roll	         30°  	29.92°	  0.27 %
Pitch	         20°	  19.95°	  0.25 %
Dynamic
Tracking    < 2°lag   	Good     	—

### Performance metrics

Metric     	Software (ARM)    	FPGA (Current)	 Improvement
Frequency	     ~50 megaHz	         137.99 megaHz	   2.76x
Latency      	~150 micro(s)	      2.4 micro(s)      62.5x
Power         	~5 W	            ~2 W	             2.5×
Deterministic   	No	              Yes             	-

## Contributing

This is an academic project for EECE 5698.
