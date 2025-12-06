# Recommended Hardware Guide

## Overview
While this course focuses on simulation-based learning without requiring hands-on hardware assembly, understanding the recommended hardware components is valuable for those interested in deploying their projects to real robots.

## System Requirements

### Development Machine (RTX Workstation)
**Purpose**: Running simulations and training AI models

**Recommended Specifications**:
- **CPU**: High-core count processor (e.g., AMD Ryzen 9, Intel Core i9)
  - Minimum: 8 cores, Recommended: 16+ cores
  - Reasoning: Simulation and AI workloads are CPU-intensive

- **GPU**: NVIDIA RTX series (RTX 3090, RTX 4090, or RTX 6000)
  - Minimum: 24GB VRAM, Recommended: 48GB+ VRAM
  - Reasoning: Required for NVIDIA Isaac Sim and AI training

- **RAM**: System Memory
  - Minimum: 32GB, Recommended: 64GB+
  - Reasoning: Multiple simulations and models running simultaneously

- **Storage**: SSD
  - Minimum: 1TB NVMe SSD for OS and tools
  - Recommended: 2-4TB for datasets and models

**Cost Estimate**: $3,000 - $8,000

### Robot Platform (Jetson Orin Nano)
**Purpose**: Embedded AI and robot control

**Specifications**:
- **Processor**: NVIDIA Jetson Orin Nano
  - 8-core ARM CPU
  - 40 TOPS of AI performance
  - Ideal for edge computing on robots

- **Memory**:
  - 8GB or 12GB LPDDR5 RAM options
  - Sufficient for running ROS 2 and lightweight AI models

- **Power Consumption**: 5-15W
  - Low power for battery-operated robots

- **Interfaces**: USB, Ethernet, UART, GPIO, I2C, SPI
  - Allows connection to sensors and actuators

**Cost Estimate**: $200 - $400

### Perception System (Depth Camera)
**Purpose**: Visual perception for robot navigation and manipulation

**Recommended Sensors**:
- **Intel RealSense D435**:
  - RGB + Depth camera
  - Good for robotics applications
  - USB 3.0 interface
  - Cost: $150 - $200

- **Stereo Labs ZED**:
  - High-quality depth sensing
  - Built-in AI capabilities
  - Cost: $300 - $500

**Key Features**:
- Depth resolution: 640×480 or higher
- RGB resolution: 1280×720 or higher
- Frame rate: 30 FPS minimum
- USB or Ethernet connectivity

## Additional Components

### Actuation
- **Joint Motors**: Servo motors or stepper motors for articulation
- **Motor Controllers**: PWM or CAN bus motor controllers
- **Transmission**: Gears and mechanical linkages

### Power System
- **Battery**: Lithium-ion batteries (12V - 48V)
- **Power Management**: Battery management system (BMS)
- **Charging**: Appropriate chargers and docking stations

### Structural
- **Robot Frame**: Aluminum or composite materials
- **Fasteners**: Bolts, screws, connectors
- **Protective Housing**: Enclosures for electronics

## Budget Summary
- **Minimum Setup** (Simulation only): $2,000
- **Development Machine + Jetson**: $3,500 - $8,500
- **Full Robotics Platform**: $10,000 - $20,000+

## Notes
- This course focuses on simulation, so you don't need to purchase hardware immediately
- Start with simulation on your existing machine
- Gradually add hardware components as needed
- Consider budgeting over time rather than all at once

## Where to Buy
- **Development Machines**: Dell, NVIDIA, ASUS, HP (workstations)
- **Jetson Orin Nano**: NVIDIA Developer Store, resellers
- **Cameras**: Amazon, B&H Photo, official manufacturer websites
- **Components**: RobotShop, Adafruit, SparkFun

## Future Upgrades
Consider these upgrades as you progress:
1. Multi-GPU setups for faster training
2. Higher-end Jetson platforms (Orin NX, Orin AGX)
3. Professional-grade depth sensors
4. Multiple robots for multi-agent learning
