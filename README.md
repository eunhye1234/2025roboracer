# RoboRacer Sim Racing League @ ICRA 2025

This repository contains code for the **RoboRacer Sim Racing League @ ICRA 2025** competition.

🔗 [Competition Website](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-icra-2025/#resources)

---

## 🧩 Dependencies

- **OS**: Ubuntu 22.04
- **Python**: 3.10.12
- **ROS 2**: Humble Hawksbill
- **TensorFlow**: 2.16.1
- **CUDA**: 12.4

> 💡 **Check NVIDIA Driver**:  
> Run `nvidia-smi` to check your GPU status.  
> If nothing shows up:
> ```bash
> sudo ubuntu-drivers autoinstall
> ```

---

## 🚀 Setup Instructions

### 1. Pull and Run AutoDRIVE Simulator

**Pull the container**:
```bash
docker pull autodriveecosystem/autodrive_roboracer_sim:2025-icra-practice
