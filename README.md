# Laser Shooting Simulation System (STM32F103C8T6)

![Status](https://img.shields.io/badge/status-active-brightgreen)
![Platform](https://img.shields.io/badge/platform-STM32F103C8T6-blue)
![Language](https://img.shields.io/badge/language-C%20(HAL%2FLL)-blue)

## Overview
A real-time laser shooting simulation system built on the **STM32F103C8t^** microcontroller.  
The system supports multiple firing modes and integrates an OLED display, vibration motor, and sound module.  
All firing logic is controlled using **EXTI interrupts**, **timers**, and a **finite state machine (FSM)** to ensure precision and stability.

---

## Features
- Single / Burst / Auto fire modes.
- Interrupt-driven control using EXTI.
- Timer-based shot timing and vibration feedback.
- SSD1306 OLED display for UI.
- FSM-based firing logic.
- Modular & easy-to-extend firmware architecture.

---

## Tech Stack
- **MCU:** STM32F103C8T6 (ARM Cortex-M3)  
- **Language: ** C (HAL/LL)  
- **Peripherals:
  + ** Module Laser KY-008 650nm
  + ** Speaker0.5W 8 Ohm 40mm
  + ** Solenoid JF-0730B 12VDC 1A
  + ** LCD OLED SSD1306 0.96 inch 128x64
  + ** Button
  + ** MOSFET IRLZ44N
  + ** Diode 1N5408
- **Tools:** STM32CubeIDE, ST-Link, Git, multimeter  

## Installation / Setup
**Connection diagram
<img width="639" height="344" alt="Screenshot (3)" src="https://github.com/user-attachments/assets/027bcd22-4f1e-4889-ab1f-0b3a608c1db9" />

1. Clone repo
2. Open the project in STM32CubeIDE
3. Build the project
4. Flash to board using ST-Link
