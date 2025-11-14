# Laser Shooting Simulation System

![Status](https://img.shields.io/badge/status-active-brightgreen)
![Platform](https://img.shields.io/badge/platform-STM32F103C8T6-blue)
![Language](https://img.shields.io/badge/language-C%20(HAL%2FLL)-blue)

## Overview
A real-time laser shooting simulation system built on the **STM32F103C8T6** microcontroller.  
The system supports multiple firing modes and integrates an OLED display, vibration motor, and sound module.  
All firing logic is managed using **EXTI interrupts**, **timers**, and a **finite state machine (FSM)** to ensure precise and stable operation.

---

## Features
- Single / Burst / Auto fire modes  
- Interrupt-driven control using EXTI  
- Timer-based shot timing and vibration feedback  
- SSD1306 OLED display for UI  
- FSM-based firing logic  
- Modular and easy-to-extend firmware architecture  

---

## Tech Stack
- **MCU:** STM32F103C8T6 (ARM Cortex-M3)  
- **Language:** C (HAL/LL)  
- **Peripherals:**  
  - Laser Module KY-008 (650 nm)  
  - Speaker 0.5W 8Ω  
  - Solenoid JF-0730B (12VDC, 1A)  
  - OLED SSD1306 0.96" (128×64)  
  - Push button  
  - MOSFET IRLZ44N  
  - Diode 1N5408  
- **Tools:** STM32CubeIDE, ST-Link, Git, Multimeter  

---

## Installation / Setup

<img width="639" height="344" alt="Schematic" src="https://github.com/user-attachments/assets/027bcd22-4f1e-4889-ab1f-0b3a608c1db9" />

1. Connect the **12V / 1A** power supply to the board.  
2. Clone the repository:  
   ```bash
   git clone https://github.com/your_username/LaserGun.git
3. Open the project in STM32CubeIDE.
4. Build the project.
5. Flash the firmware using ST-Link.
