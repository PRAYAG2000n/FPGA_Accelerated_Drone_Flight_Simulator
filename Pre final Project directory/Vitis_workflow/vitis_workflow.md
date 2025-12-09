# Vitis Workflow Output

## Compiled xclbin for Alveo U280

The `quadcopter_system.xclbin` file (47 MB) is available in the releases:

👉 **[Download xclbin from Releases](https://github.com/PRAYAG2000n/FPGA_Accelerated_Drone_Flight_Simulator/releases/tag/v1.0)**

### Build Details
- **Platform:** xilinx_u280_gen3x16_xdma_1_202211_1
- **Tool Version:** Vitis 2023.2
- **Build Time:** ~2 hours
- **Target:** Alveo U280 FPGA

### How to Use
```bash
# On CloudLab pc170 with U280
source /opt/xilinx/xrt/setup.sh
./host_app quadcopter_system.xclbin
```
