# 📡 MC60 Dev Board Firmware

## 📖 Overview

Firmware for a Quectel MC60-based development board designed for IoT and embedded applications with rich sensor and connectivity options.

This codebase demonstrates a multi-tasking architecture using the MC60 SDK, integrating GNSS, sensors, OLED display, storage, and SIM management.

---

## 🛠 Hardware Features

- 📶 MC60 GSM/GPRS/GNSS/Bluetooth module  
- 🔋 Battery management with charging circuitry  
- 🖥 0.96" OLED display via I²C  
- 🛰 9-axis MEMS accelerometer/gyroscope (ICM series)  
- 🌡 Temperature & humidity sensor (SHT series)  
- 💾 SPI Flash storage  
- 📡 Active & passive GNSS antenna connectors  
- 📶 Dedicated BLE antenna  
- 💽 microSD card slot  
- 📲 Dual SIM card support  
- ⚡ Pin headers for expansion  

---

## 💡 Firmware Features

- 🧵 Multi-task design:
  - GNSS data acquisition (`proc_gnss`)
  - Sensor reading tasks (accelerometer, gyroscope, temperature, humidity)
  - OLED UI task with paging
  - SIM status handling
  - SPI Flash management

- 🔢 Pure-integer GNSS RMC parser — no floating-point usage  
- ⏱ Semaphore-driven synchronization between producer tasks and OLED consumer  
- 🖋 Optimized OLED updates to avoid flicker using fixed-width field formatting  

---

## 🖥 Development Environment

- **Toolchain**: MC60 SDK (GCC for ARM under MS-DOS environment)  
- **Build commands** (run in project root):

  ```bash
  make clean
  make new
