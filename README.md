# 🌡️ Smart Thermostat — Raspberry Pi IoT Prototype

A hardware-software integrated thermostat built on 
Raspberry Pi 4B with real-time sensor data, 
state-driven control logic, and simulated cloud telemetry.

## Hardware
- Raspberry Pi 4B
- AHT20 temperature/humidity sensor (I2C)
- 16x2 LCD display
- LED indicators (heating/cooling states)

## How It Works
The system runs a state machine with three modes:
Off → Heat → Cool

Button input drives state transitions. 
The AHT20 sensor reads temperature and humidity via I2C, 
displays live data on the LCD, and fades LEDs to indicate 
active heating or cooling. UART output simulates 
cloud telemetry for remote monitoring.

## Stack
- Python
- GPIO / I2C protocols
- UART serial communication
- State machine architecture

## Key Engineering Decisions
- Chose I2C over SPI for sensor communication — 
  simpler wiring, sufficient speed for polling interval
- State machine pattern keeps control logic 
  predictable and testable
- UART output designed to be drop-in replaceable 
  with real cloud pipeline (MQTT/HTTP)
