# Household Energy Monitoring and Control System

## Project Overview

This project implements a comprehensive household energy monitoring and control system using an STM32 Nucleo-L152RE microcontroller. The system integrates multiple sensors to measure various environmental parameters and energy consumption, communicates via Modbus protocol, and provides real-time data logging and control capabilities.

## Table of Contents

1. Hardware Components
2. Software Architecture
3. Sensor Integration
4. Modbus Communication
5. Real-Time Scheduling
6. Energy Control Logic
7. Data Logging and Visualization

## Hardware Components

- STM32 Nucleo-L152RE microcontroller board
- Sensors:
    - DHT22: Temperature and humidity sensor
    - LMT84LP: Freezer temperature sensor
    - NSL19M51: Light intensity sensor
    - HIH-4000-001: Bathroom humidity sensor
    - Grove-VOC-and-eCO2: Carbon dioxide and VOC sensor
    - 77DE-06-09: AC-line voltage sensor
    - L-934LGD: Energy measurement sensor
- MAX3485: RS-485 transceiver for Modbus communication
- Relay module for energy control

### Pin Assignments

- LMT84LP: PA1 (ADC channel 1)
- NSL19M51: PA2 (ADC channel 2)
- HIH-4000: PA3 (ADC channel 3)
- 77DE-06-09: PA4 (ADC channel 4)
- DHT22: PA6 (GPIO)
- Grove-VOC-and-eCO2: PA9 (SCL), PA10 (SDA)
- L-934LGD: PA7 (pulse input)
- Relay: PA8 (output)
- MAX3485: USART1 (PA9: TX, PA10: RX)
- Debug UART: USART2 (PA2: TX, PA3: RX)
- Status LED: PA5

## Software Architecture

The software is structured into several modules:

1. `main.c`: Core application logic and initialization
2. `sensors.c`: Sensor-specific implementations
3. `dht22.c`: DHT22 sensor communication
4. `delay.c`: Timing utilities
5. `include.h`: Common header file
6. `master.py`: Python script for Modbus master and data logging

## Sensor Integration

Each sensor is integrated using appropriate interfaces:

- ADC for analog sensors (LMT84LP, NSL19M51, HIH-4000, 77DE-06-09)
- Digital GPIO for DHT22
- I2C for Grove-VOC-and-eCO2
- Pulse counting for L-934LGD energy meter

Sensor readings are converted to appropriate units and stored in global variables for Modbus access.

## Modbus Communication

- Implemented using USART1 and MAX3485 transceiver
- Supports multiple input registers for different sensors
- Includes CRC checking and basic error handling
- Python master script queries all sensors sequentially

## Real-Time Scheduling

- TIM2 timer used to trigger sensor readings every second
- Interrupt-driven approach ensures timely updates for real-time monitoring

## Energy Control Logic

- Basic relay control implemented on PA8
- Threshold-based control using CO2 levels and energy consumption
- Expandable for more complex control strategies

## Data Logging and Visualization

- Python script logs data to IoT-Ticket platform
- Sensor data mapped to appropriate units and paths
- Real-time visualization available on IoT-Ticket dashboard
