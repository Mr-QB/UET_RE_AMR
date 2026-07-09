# Hardware Documentation

This directory contains the electrical schematics, PCB layouts, and mechanical CAD designs for the UET AMR platform.

---

## Directory Structure

```
hardware/
├── schematics/      # Schematic drawings (KiCad format)
├── pcb/             # PCB layouts (KiCad format)
├── cad/             # Mechanical CAD models (STEP, Fusion360 formats)
└── BOM.xlsx         # Project Bill of Materials
```

---

## Bill of Materials

Refer to `BOM.xlsx` for a complete list of mechanical parts, fasteners, electronics, sensors, and purchasing links.

---

## Large Files and Git LFS

This repository uses Git Large File Storage (LFS) to manage large binary files such as 3D CAD assemblies and PCB models. Ensure Git LFS is installed and initialized before staging changes in this directory:

```bash
# Initialize Git LFS in your local environment
git lfs install

# Configured tracking filters:
# git lfs track "hardware/**/*.step"
# git lfs track "hardware/**/*.f3d"
# git lfs track "hardware/**/*.kicad_pcb"
```

---

## Printed Circuit Boards (PCBs)

The robot architecture uses three dedicated microcontrollers:

| Sub-system | Microcontroller | Primary Function | Source Directory |
|---|---|---|---|
| Base Controller | STM32F446RE | Motor control and encoder feedback | `firmware/base_controller` |
| Sensor Hub | ESP32 | IMU and ultrasonic sensor fusion | `firmware/sensor_hub` |
| Power Board | STM32F103 | Battery management system (BMS) and relays | `firmware/power_management` |
