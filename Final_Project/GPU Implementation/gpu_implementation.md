# GPU Implementation - Quadcopter Flight Controller

**Author:** Prayag Sridhar  
**Course:** EECE 5698 - Advanced Digital System Design  
**Date:** December 2025

---

## Overview

GPU implementation of the quadcopter flight controller using NVIDIA CUDA for performance comparison with FPGA. Runs on NVIDIA Tesla V100-SXM2-32GB. This was done in northeastern explorer cluster.

**Results:**
- Execution Time (mean): 5.68 µs
- Per-Sample Latency: 0.0568 µs
- Throughput: 17,596,714 samples/sec
- Power Consumption: 56.65 W
- Jitter (Max/Min): 1.80× (non-deterministic)

---

## Prerequisites

- Linux (CentOS 7/8 or Ubuntu 18.04+)
- NVIDIA GPU (Tesla V100 or compatible)
- CUDA Toolkit 11.0+
- NVIDIA Driver 450+

---

## Quick Start

### Step 1: Clone Repository

```bash
git clone https://github.com/PRAYAG2000n/FPGA_Accelerated_Drone_Flight_Simulator.git
cd "FPGA_Accelerated_Drone_Flight_Simulator/Final_Project/GPU Implementation"
```

### Step 2: Verify GPU Availability

```bash
nvidia-smi
```

Expected output:
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 470.xxx       Driver Version: 470.xxx       CUDA Version: 11.x   |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  Tesla V100-SXM2...  On   | 00000000:00:04.0 Off |                    0 |
| N/A   32C    P0    24W / 300W |      0MiB / 32510MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+
```

### Step 3: Compile

```bash
nvcc -O3 -arch=sm_70 -o gpu_benchmark gpu_benchmark.cu -lnvidia-ml
```

**Note:** Change `-arch=sm_70` based on your GPU:
| GPU | Architecture |
|-----|--------------|
| Tesla V100 | sm_70 |
| RTX 2080 | sm_75 |
| RTX 3080 | sm_86 |
| RTX 4090 | sm_89 |

### Step 4: Run

```bash
./gpu_benchmark
```

---

## Expected Output

```
GPU Quadcopter Flight Controller Benchmark
  Samples: 100
GPU: Tesla V100-SXM2-32GB
SMs: 80

RESULTS
  Execution Time (mean):    5.68 us
  Execution Time (min):     5.12 us
  Execution Time (max):     9.22 us
  Execution Time (std):     0.62 us
  Latency (per sample):     0.0568 us
  Throughput:               17596714.00 samples/sec

RESOURCE UTILIZATION
  Grid Size:                1 blocks x 256 threads
  Active Threads:           100
  Memory Used:              11.33 KB

POWER CONSUMPTION
  Power:                    56.65 W
```

---

## Implementation Video
- **GPU Implementation**: https://www.loom.com/share/0ce497ae31e44db6b602b4741e565d4f

---
## Key Observation

GPU shows **1.80× timing jitter** (min: 5.12 µs, max: 9.22 µs), making it unsuitable for deterministic real-time flight control despite higher throughput.

---

## Links

- **GPU Implementation:** https://github.com/PRAYAG2000n/FPGA_Accelerated_Drone_Flight_Simulator/tree/main/Final_Project/GPU%20Implementation
