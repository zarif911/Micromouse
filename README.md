# Micromouse Maze Solver

An Arduino-based Micromouse robot that navigates and solves a 16x16 maze using the floodfill algorithm.

## Overview

This project implements a fully autonomous Micromouse robot that explores an unknown 16x16 maze, maps the walls, and finds the optimal path to the center using the floodfill algorithm. The robot uses IR sensors for wall detection, encoders for precise movement, and DC motors with PID control for accurate navigation.

## Features

- **Autonomous Maze Solving**: Uses floodfill algorithm to navigate from start (0,0) to center (7,7/8,8)
- **Wall Mapping**: Detects and stores wall positions using IR sensors
- **Precise Movement**: Encoder-based odometry for accurate distance control
- **Motor Control**: PID-controlled straight movements with adjustable parameters
- **Real-time Updates**: Continuously updates wall map and recalculates optimal path
- **Configurable**: Easy to adjust speed, threshold values, and maze parameters

## Hardware Requirements

### Components
- ESP32 or Arduino-compatible microcontroller
- 2x DC motors with encoders (12 CPR recommended)
- 2x Motor driver (L298N or similar)
- 3x IR sensors (for left, front, right wall detection)
- 2x Wheels (43mm diameter recommended)
- Power source (7.4V LiPo battery)
- 6x AA batteries (optional)

### Pin Connections

| Component | Pin | Function |
|-----------|-----|----------|
| IR Left | 34 | Analog input |
| IR Front | 35 | Analog input |
| IR Right | 32 | Analog input |
| Encoder Left A | 4 | Interrupt pin |
| Encoder Left B | 16 | Digital input |
| Encoder Right A | 17 | Interrupt pin |
| Encoder Right B | 5 | Digital input |
| Motor IN1 | 25 | Direction control |
| Motor IN2 | 26 | Direction control |
| Motor ENA | 27 | PWM enable (Left) |
| Motor IN3 | 14 | Direction control |
| Motor IN4 | 12 | Direction control |
| Motor ENB | 13 | PWM enable (Right) |

## Software Architecture

### Core Components

1. **Maze Solver**: Floodfill algorithm implementation
2. **Motor Control**: PID-based straight movement and precise turning
3. **Wall Detection**: IR sensor calibration and processing
4. **Path Planning**: Dynamic route recalculation

### Algorithm Flow

```mermaid
graph TD
    A[Start] --> B[Initialize Maze]
    B --> C[Check Sensors]
    C --> D[Update Wall Map]
    D --> E[Run Floodfill]
    E --> F[Find Best Move]
    F --> G[Execute Movement]
    G --> H[At Center?]
    H -->|No| C
    H -->|Yes| I[Stop and Complete]
