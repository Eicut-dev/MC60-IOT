📡 MC60 Dev Board Firmware
📖 Overview
Firmware for a Quectel MC60‑based development board designed for IoT and embedded applications with rich sensor and connectivity options.

This codebase demonstrates multi‑tasking firmware using the MC60 SDK, integrating GNSS, sensors, OLED display, storage, and SIM management.

🛠 Hardware Features
📶 MC60 GSM/GPRS/GNSS/Bluetooth module
🔋 Battery management with charging circuitry
🖥 0.96" OLED display via I²C
🛰 9‑axis MEMS accelerometer/gyroscope (ICM series)
🌡 Temperature & humidity sensor (SHT series)
💾 SPI Flash storage
📡 Active & passive GNSS antenna connectors
📶 Dedicated BLE antenna
💽 microSD card slot
📲 Dual SIM card support
⚡ Pin headers for expansion
💡 Firmware Features
🧵 Multi‑task design for:
GNSS data acquisition (proc_gnss)
Sensor reading tasks (ACC/Gyro, Temp/Humidity)
OLED UI task with paging
SIM status handling
SPI Flash management
🔢 Pure‑integer GNSS RMC parser — no floating‑point usage
⏱ Semaphore‑driven synchronization between producer tasks and OLED consumer
🖋 Optimized OLED updates to avoid flicker using fixed‑width field formatting
🖥 Development Environment
Toolchain: MC60 SDK (GCC for ARM under MS‑DOS environment)
Build commands (run in project root):
  make clean
  make new
Firmware flashing: Upload using Quectel QFlash tool
🔍 How It Works
On boot, tasks are created for each hardware block.
proc_oled listens for messages and updates the display according to the active page.
GNSS task proc_gnss fetches NMEA RMC sentences, parses coordinates into integer + fractional components, and sends them to the OLED task.
Sensor tasks push readings via message queues and release semaphores to coordinate peripheral access.
📦 Requirements
🛠 MC60 Developer Board hardware
🖥 QFlash & MC60 SDK
🖧 Serial terminal for debugging output (APP_DEBUG)
🚀 Example Usage
Connect board via USB/UART.
Compile firmware:
   make clean
   make new
Use QFlash to upload .cfg file to module.
Open terminal to monitor logs and view live sensor + GNSS updates on OLED.
📜 License
This project is released under the MIT License