# LMI_Ballbot_V2

This repository contains the source code, STL files, and experimental results developed during research on the Ballbot platform as part of the Master of Science program in Systems and Control at the Instituto Tecnológico de Aeronáutica (ITA). The work represents a second iteration and expansion of the initial Ballbot project proposed in the master’s thesis available at [Ballbot_LMI_V1](https://github.com/pedrophj/ballbot_LMI/tree/main).

## Repository Structure

| Directory | Description |
|------------|-------------|
| [**`/Data Bags`**](./Experimental%20Results/Data%20Bags/) | Provides the ROS bag files acquired during the experimental trials, offering the recorded topic data used for analysis and validation. |
| [**`/Videos`**](./Experimental%20Results/Videos/) | Provides video recordings of the experimental results obtained with the Ballbot prototype. The videos are also available online at [Google Drive](https://drive.google.com/drive/folders/1ocQBuRk292TQQt81lqkflchQvDIo9T61?usp=sharing). |
| [**`/Microcontroller`**](/Microcontroller/src) | PlatformIO project containing the ESP32 firmware. |
| [**`/STL`**](/STL/) | STL files for the mechanical structure of the Ballbot for 3D printing. |
| [**`/Simulations/2D`**](/Simulations/2D) | MATLAB simulations for the X–Z and X–Y planar models, used for control design and validation. |
| [**`/Simulations/3D`**](/Simulations/3D) | CoppeliaSim environment developed to validate the 2D model using a full 3D representation of the Ballbot. |

## MATLAB Simulations

The MATLAB simulation files for the Ballbot’s dynamic modeling and control are available in the [**`/Simulations/2D`**](/Simulations/2D) directory.  
These simulations cover both the X–Z and X–Y planar models, developed using **MATLAB version 2023b**.

## 3D Simulation Environment

The 3D simulation files are available in the [**`/Simulations/3D`**](/Simulations/3D) directory. The environment was developed using **CoppeliaSim Edu version 4.10**.

## Bill of Materials

The tables below list the electronic and structural components required to assemble and operate the Ballbot prototype, including specifications, quantities, and supply sources when applicable.

### Electronics

| Component | Model / Specification | Quantity |
|------------|-----------------------|:-------:|
| Microcontroller | ESP32-WROOM-32 | 1 |
| Microcomputer | Raspberry Pi 4 Model B (4GB) | 1 |
| Stepper Motors | NEMA 17 (17HS4401) | 3 |
| Motor Drivers | DRV8825 | 3 |
| IMU | Witmotion WT901C | 1 |
| LiDAR Sensor | RPLIDAR A2M8 | 1 |

### Structure Parts

| Part | Quantity |
|-----------------------|:-------:|
| Layer 0 | 1 |
| Layer 1 | 1 |
| Layer 2 | 1 |
| Layer 3 | 1 |
| Motor Support - Layer 0 Side | 3 |
| Motor Support - Motor Side | 3 |
| Support - Layer 1 | 3 |
| Support - Layer 2 | 3 |
| Support - Layer 3 | 3 |
| Support for Electronics - Layer 1 | 1 |
| Battery Support - BMS | 1 |
| Battery Support - Connectors | 1 |
| Wheel Assembly | 3 |

### Wheel Assembly

| Part | Quantity | Notes |
|-----------------------|:-------:|:-----------:|
| Fork | 6 | - |
| Roller - Small | 6 | - |
| Roller - Big | 6 | - |
| Motor Coupling | 1 | - |
| Motor Coupling Cap | 1 | - |
| MR52ZZ Spherical Bearing | 24 | Available at: [Aliexpress](https://pt.aliexpress.com/item/32492575669.html) |
