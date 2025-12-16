# CPU Implementation - Quadcopter Flight Controller

**Author:** Prayag Sridhar  
**Course:** EECE 5698 - Advanced Digital System Design  
**Date:** December 2025

---

## Overview

CPU baseline implementation of the quadcopter flight controller for performance comparison with FPGA. Runs on Intel Xeon Gold 6132 processor.

**Results:**
- Execution Time (mean): 7.48 µs
- Per-Sample Latency: 0.0748 µs
- Throughput: 13,360,696 samples/sec
- Power Consumption: 3.57 W

---

## Prerequisites

- Linux (CentOS 7/8 or Ubuntu 18.04+)
- GCC 7.0+

---

## Quick Start

### Step 1: Clone Repository

```bash
git clone https://github.com/PRAYAG2000n/FPGA_Accelerated_Drone_Flight_Simulator.git
cd "FPGA_Accelerated_Drone_Flight_Simulator/Final_Project/CPU Implementation"
```

### Step 2: Compile

```bash
g++ -O3 -o cpu_benchmark cpu_benchmark.cpp
```

### Step 3: Run

```bash
./cpu_benchmark
```

---

## Expected Output

```
CPU Quadcopter Flight Controller Benchmark
  Samples: 100
CPU: Intel(R) Xeon(R) Gold 6132 CPU @ 2.60GHz
Cores: 28, Freq: 3.70 GHz, TDP: 125 W

RESULTS
  Execution Time (mean):    7.48 us
  Execution Time (min):     7.45 us
  Execution Time (max):     7.65 us
  Execution Time (std):     0.02 us
  Latency (per sample):     0.0748 us
  Throughput:               13360696.04 samples/sec

RESOURCE UTILIZATION
  Memory Used:              11.33 KB
  Threads:                  1 (single-threaded)
  CPU Utilization:          1.50% (of 1 core)

POWER CONSUMPTION
  CPU TDP:                  125 W
  Per-Core TDP:             4.46 W
  Power (1 core active):    3.57 W
```

---
## Implementation video
- **CPU implementation:** https://www.loom.com/share/0ce497ae31e44db6b602b4741e565d4f

---
## Links

- **CPU Implementation:** https://github.com/PRAYAG2000n/FPGA_Accelerated_Drone_Flight_Simulator/tree/main/Final_Project/CPU%20Implementation
