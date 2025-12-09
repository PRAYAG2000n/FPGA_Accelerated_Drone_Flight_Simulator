# Vitis Workflow - Quadcopter Flight Controller for Alveo U280

## Files

| File | Description |
|------|-------------|
| `quadcopter_system.xclbin` | Compiled FPGA bitstream (47 MB) - [Download from Releases](https://github.com/PRAYAG2000n/FPGA_Accelerated_Drone_Flight_Simulator/releases/tag/v1.0) |
| `host.cpp` | Host application source code |

## Build Details

- **Platform:** xilinx_u280_gen3x16_xdma_1_202211_1
- **Tool Version:** Vitis 2023.2
- **Build Time:** ~2 hours
- **Target:** Alveo U280 FPGA (CloudLab pc170)

---

## How to Run on pc170 (CloudLab U280)

### Step 1: Download files

```bash
# Download xclbin from GitHub release
wget https://github.com/PRAYAG2000n/FPGA_Accelerated_Drone_Flight_Simulator/releases/download/v1.0/quadcopter_system.xclbin

# Download host.cpp
wget https://raw.githubusercontent.com/PRAYAG2000n/FPGA_Accelerated_Drone_Flight_Simulator/main/Pre%20final%20Project%20directory/Vitis_workflow/host.cpp
```

### Step 2: Setup XRT environment

```bash
source /opt/xilinx/xrt/setup.sh
```

### Step 3: Compile host application

```bash
g++ -o host_app host.cpp \
    -I/opt/xilinx/xrt/include \
    -L/opt/xilinx/xrt/lib -lxrt_coreutil \
    -pthread -std=c++17
```

### Step 4: Run

```bash
./host_app quadcopter_system.xclbin
```

---

## Expected Output

```
Struct sizes - IMUData: 18, StateVector: 48, MotorSpeeds: 8
Device opened
XCLBIN loaded
Kernel obtained
Running kernel...
Kernel finished
========== THROTTLE UP PHASE (0-19) ==========
Sample 0: Roll=1.64413 Pitch=-2.41499 PosZ=50 Motors=[0, 0, 0, 0]
Sample 5: Roll=1.48608 Pitch=-2.18295 PosZ=50 Motors=[0, 0, 0, 0]
Sample 10: Roll=1.34323 Pitch=-1.97322 PosZ=50 Motors=[0, 0, 0, 0]
Sample 15: Roll=1.21411 Pitch=-1.78363 PosZ=50 Motors=[0, 0, 0, 0]
========== HOVER PHASE (20-49) ==========
Sample 20: Roll=1.0974 Pitch=-1.61227 PosZ=50 Motors=[0, 0, 0, 0]
Sample 30: Roll=0.896561 Pitch=-1.31737 PosZ=50 Motors=[0, 0, 0, 0]
Sample 40: Roll=0.732452 Pitch=-1.0764 PosZ=50 Motors=[0, 0, 0, 0]
========== ROLL RIGHT PHASE (50-74) ==========
Sample 50: Roll=0.598373 Pitch=-0.883514 PosZ=50 Motors=[0, 0, 0, 0]
Sample 60: Roll=0.488831 Pitch=-0.758514 PosZ=50 Motors=[0, 0, 0, 0]
Sample 70: Roll=0.399292 Pitch=-0.656387 PosZ=50 Motors=[0, 0, 0, 0]
========== PITCH FORWARD PHASE (75-99) ==========
Sample 75: Roll=0.364899 Pitch=-0.608551 PosZ=50 Motors=[0, 0, 0, 0]
Sample 85: Roll=0.334656 Pitch=-0.497299 PosZ=50 Motors=[0, 0, 0, 0]
Sample 95: Roll=0.309921 Pitch=-0.406387 PosZ=50 Motors=[0, 0, 0, 0]
========== TEST COMPLETE ==========
Final state: Roll=0.301346 Pitch=-0.374863 PosZ=50
```

---

## Results Interpretation

| Output | Meaning |
|--------|---------|
| `Device opened` | FPGA detected via PCIe  |
| `XCLBIN loaded` | Bitstream loaded onto U280  |
| `Kernel obtained` | quadcopter_system kernel ready  |
| `Kernel finished` | FPGA executed the design  |
| `PosZ=50` | Initial altitude maintained (matches C simulation)  |
| `Roll/Pitch changing` | Complementary filter processing IMU data  |
| `Motors=[0,0,0,0]` | Logic/tuning issue, not hardware failure |

---

## What This Proves

| Verification | Status |
|--------------|--------|
| HLS synthesis correct |  C++ converted to RTL |
| Vitis build correct |  xclbin generated |
| FPGA running design |  Kernel executing on U280 |
| PCIe communication |  Host ↔ FPGA transfer working |
| HBM memory access |  Data stored/retrieved |
| State estimation |  Roll/Pitch values match C simulation |

---

## Troubleshooting

### If wget fails:
```bash
# Use curl instead
curl -L -o quadcopter_system.xclbin https://github.com/PRAYAG2000n/FPGA_Accelerated_Drone_Flight_Simulator/releases/download/v1.0/quadcopter_system.xclbin
```

### If compilation fails:
```bash
# Check XRT is sourced
echo $XILINX_XRT
# Should show: /opt/xilinx/xrt

# If empty, source it again
source /opt/xilinx/xrt/setup.sh
```

### If device not found:
```bash
# Check FPGA is detected
xbutil examine

# Should show Alveo U280
```

---

## File Transfer (How files were copied to pc170)

### From fpga-tools to pc170:

```bash
# On fpga-tools build machine

# Transfer xclbin to pc170
scp ~/quadcopter_vitis/quadcopter_system.xclbin Prayag@pc170.cloudlab.umass.edu:~

# Transfer host application to pc170
scp ~/quadcopter_vitis/host/host_app Prayag@pc170.cloudlab.umass.edu:~
```

### On pc170 (after transfer):

```bash
# Verify files received
ls -la ~/quadcopter_system.xclbin
ls -la ~/host_app

# Setup XRT and run
source /opt/xilinx/xrt/setup.sh
./host_app quadcopter_system.xclbin
```

---

## Build Commands Reference (How xclbin was created)

### On fpga-tools build machine:

```bash
# 1. Source Vitis
source /tools/Xilinx/Vitis/2023.2/settings64.sh

# 2. Compile kernel (.xo)
v++ -c -t hw \
    --platform /opt/xilinx/platforms/xilinx_u280_gen3x16_xdma_1_202211_1/xilinx_u280_gen3x16_xdma_1_202211_1.xpfm \
    -DALVEO_U280_HW \
    -k quadcopter_system \
    -o quadcopter_system.xo \
    ./src/quadcopter_top.cpp \
    ./src/altitude_controller.cpp \
    ./src/attitude_controller.cpp \
    ./src/complementary_filter.cpp \
    ./src/flight_controller.cpp \
    ./src/keyboard_processor.cpp \
    ./src/motor_mixer.cpp \
    ./src/pid_controller.cpp \
    ./src/safety_monitor.cpp \
    -I./src

# 3. Link kernel (.xclbin) - takes ~2 hours
v++ -l -t hw \
    --platform /opt/xilinx/platforms/xilinx_u280_gen3x16_xdma_1_202211_1/xilinx_u280_gen3x16_xdma_1_202211_1.xpfm \
    -o quadcopter_system.xclbin \
    quadcopter_system.xo
```

---

## Author

**Prayag Sridhar**  
EECE 5698 - FPGA-based System Design  
December 2025
