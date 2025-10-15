# 🧠 WipeRite — Autonomous Whiteboard Writing & Erasing Robot

[![Build](https://img.shields.io/github/actions/workflow/status/wizwoz01/projects/ci.yml?label=build)](https://github.com/wizwoz01/projects/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Issues](https://img.shields.io/github/issues/wizwoz01/projects.svg)](https://github.com/wizwoz01/projects/issues)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-blue.svg)](CONTRIBUTING.md)
[![Release](https://img.shields.io/github/v/release/wizwoz01/projects)](https://github.com/wizwoz01/projects/releases)

> A compact, autonomous robot that magnetically attaches to whiteboards to **write** and **erase**—saving instructors time and improving classroom efficiency.

<p align="center">
  <img src="docs/media/hero.jpg" alt="WipeRite robot on a whiteboard" width="80%">
  <!-- Replace this image path with your actual robot photo or render -->
</p>

---

## 🧾 Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Architecture](#architecture)
- [Hardware](#hardware)
- [Firmware & Software](#firmware--software)
- [Getting Started](#getting-started)
- [Repository Layout](#repository-layout)
- [Roadmap](#roadmap)
- [Milestones / Demos](#milestones--demos)
- [Contributing](#contributing)
- [Team](#team)
- [License](#license)
- [Acknowledgments](#acknowledgments)

---

## 💡 Overview

**WipeRite** is an autonomous whiteboard robot that **writes and erases** notes automatically.  
It magnetically mounts to any standard whiteboard, maps the surface, and performs precise writing and erasing operations.

> Designed for educators, presenters, and workplaces to automate repetitive whiteboard tasks and improve focus during lessons.

---

## ⚙️ Key Features

- 🔩 **Magnetic Adhesion** — Securely attaches to standard whiteboards.
- 🧭 **Autonomous Navigation** — Maps the board and plans paths intelligently.
- ✍️ **Servo-Driven Writing** — Consistent, legible marker control.
- 🧽 **Motorized Erasing** — Clean surface after every pass.
- 📶 **Wireless Control** — Wi-Fi interface for commands and updates.
- 🧪 **Modular Design** — Independent hardware & software testing.
- 🛡️ **Safe & Reliable** — Watchdog timers and emergency-stop logic.

---

## 🧩 Architecture

<p align="center">
  <img src="docs/media/block-diagram.png" alt="WipeRite block diagram" width="80%">
  <!-- Replace with your actual system diagram -->
</p>

**Main Modules:**
- **Zynq-7000/MCU (ARM)** — Central control, motor logic, and sensor fusion  
- **Locomotion System** — Motor drivers, wheel/track movement  
- **Marker Module** — Servo-driven precision writing  
- **Eraser Module** — Motorized pad for automated cleaning  
- **Power System** — Li-ion battery & voltage regulation  
- **Comms** — Wi-Fi module for remote control and updates  

---

## 🧱 Hardware

- **Chassis:** 3D-printed modular frame (see `hardware/cad/`)  
- **Motors:** DC or stepper-based locomotion system  
- **Marker System:** Servo-actuated mount with controlled pressure  
- **Eraser System:** Lightweight wiping pad driven by small DC motor  
- **Sensors:** Edge detection (IR/TOF) and optional IMU  
- **Battery:** Rechargeable Li-ion pack with safety circuitry  

📂 *See [`hardware/BOM.csv`](hardware/BOM.csv) for component details.*(TBD)

---

## 💻 Firmware & Software

**Firmware (Zynq-7000/MCU)**
- Motor control loops (velocity, position)
- Marker/eraser actuator control
- Navigation & path planning
- Wireless comms (Wi-Fi, UART, SPI)

**Software (PC/Host)**
- CLI for control and telemetry  
- Optional Web UI for job uploads and calibration  

---

## 🚀 Getting Started

### 1️⃣ Clone Repository
```bash
git clone https://github.com/wizwoz01/projects.git
cd projects/T9-WIPERITE
Future Instrustions - TBD
