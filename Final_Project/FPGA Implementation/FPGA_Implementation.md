# FPGA-Accelerated Quadcopter Flight Controller

**Author:** Prayag Sridhar  
**Course:** EECE 5698
**Date:** December 2025

---

## Project Overview

A complete quadcopter flight controller implemented on Xilinx Alveo U280 FPGA using Vitis HLS 2023.2. The system includes sensor fusion, PID control, motor mixing, and safety monitoring with deterministic real-time execution.

**Key Results:**
- Control Loop Rate: 549 kHz (549× faster than 1 kHz requirement)
- Per-Sample Latency: 1.82 µs (deterministic, zero jitter)
- Resource Utilization: 8.26% LUT, 5.22% FF

---

## Repository Structure

```
Final_Project/
├── FPGA Implementation/
│   ├── src/                    # HLS source files
│   │   ├── quadcopter_top.cpp
│   │   ├── quadcopter_control.h
│   │   ├── complementary_filter.cpp
│   │   ├── pid_controller.cpp
│   │   ├── attitude_controller.cpp
│   │   ├── altitude_controller.cpp
│   │   ├── flight_controller.cpp
│   │   ├── motor_mixer.cpp
│   │   ├── safety_monitor.cpp
│   │   └── keyboard_processor.cpp
│   └── Test Bench/
│       └── host.cpp            # Host application
└── releases/
    └── quadcopter_system_175mhz.xclbin  # Pre-built FPGA binary
```

---

## Prerequisites

- Xilinx Alveo U280 FPGA card
- Xilinx Vitis 2023.2
- Xilinx XRT 2023.2
- Linux (CentOS 7/8 or Ubuntu 18.04+)

---

## Quick Start (Using Pre-Built XCLBIN)

### Step 1: Clone Repository

```bash
git clone https://github.com/PRAYAG2000n/FPGA_Accelerated_Drone_Flight_Simulator.git
cd FPGA_Accelerated_Drone_Flight_Simulator/Final_Project/FPGA\ Implementation
```

### Step 2: Download Pre-Built XCLBIN

Download from: https://github.com/PRAYAG2000n/FPGA_Accelerated_Drone_Flight_Simulator/releases/tag/v1.0

```bash
wget https://github.com/PRAYAG2000n/FPGA_Accelerated_Drone_Flight_Simulator/releases/download/v1.0/quadcopter_system_175mhz.xclbin
```

### Step 3: Setup Environment

```bash
source /opt/xilinx/xrt/setup.sh
```

### Step 4: Compile Host Application

```bash
g++ -o host_app "Test Bench/host.cpp" \
    -I/opt/xilinx/xrt/include \
    -L/opt/xilinx/xrt/lib -lxrt_coreutil \
    -pthread -std=c++17
```

### Step 5: Reset FPGA and Run

```bash
xbutil reset --device 0000:37:00.1 --force
./host_app quadcopter_system_175mhz.xclbin
```

**Note:** Replace `0000:37:00.1` with your device BDF. Find it using `xbutil examine`.

---

## Building from Source (Optional)

If you want to rebuild the XCLBIN from source (~2 hours):

### Compile Kernel

```bash
source /opt/xilinx/Vitis/2023.2/settings64.sh
source /opt/xilinx/xrt/setup.sh

v++ -c -t hw \
    --platform xilinx_u280_gen3x16_xdma_1_202211_1 \
    --hls.clock 175000000:quadcopter_system \
    -k quadcopter_system \
    -o quadcopter_system_175mhz.xo \
    ./src/*.cpp -I./src
```

### Link Kernel

```bash
v++ -l -t hw \
    --platform xilinx_u280_gen3x16_xdma_1_202211_1 \
    --kernel_frequency 175 \
    -o quadcopter_system_175mhz.xclbin \
    quadcopter_system_175mhz.xo
```

### Compile Host

```bash
g++ -o host_app "Test Bench/host.cpp" \
    -I/opt/xilinx/xrt/include \
    -L/opt/xilinx/xrt/lib -lxrt_coreutil \
    -pthread -std=c++17
```

---

## Test Phases

The test runs 100 steps across 10 flight phases:

| Steps | Phase | Description |
|-------|-------|-------------|
| 0-19 | Throttle Up | Motors increase from 20% |
| 20-29 | Hover | Stable hover at 50m |
| 30-39 | Roll Right | Tilt right |
| 40-49 | Roll Left | Tilt left |
| 50-59 | Pitch Forward | Nose down |
| 60-69 | Pitch Backward | Nose up |
| 70-79 | Yaw Right | Rotate clockwise |
| 80-84 | Yaw Left | Rotate counter-clockwise |
| 85-94 | Hover | Return to stable hover |
| 95-99 | Emergency Stop | All motors to 0% |

---

## Expected Output

Successful execution shows:

```
QUADCOPTER FPGA CONTROLLER - FULL TEST
...
Kernel execution time: 182 us
Per-sample latency:    1.82 us
Throughput:            549450.56 samples/sec
Control loop rate:     549450.56 Hz
...
[PASS] All 8 modules verified
[PASS] Emergency stop effective
TEST COMPLETED SUCCESSFULLY
```

---

## Links

- **Repository:** https://github.com/PRAYAG2000n/FPGA_Accelerated_Drone_Flight_Simulator
- **Host Code:** https://github.com/PRAYAG2000n/FPGA_Accelerated_Drone_Flight_Simulator/blob/main/Final_Project/FPGA%20Implementation/Test%20Bench/host.cpp
- **Pre-built XCLBIN:** https://github.com/PRAYAG2000n/FPGA_Accelerated_Drone_Flight_Simulator/releases/tag/v1.0
