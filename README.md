# Quadruped RL Simulator — DDS-Based Robot Control Framework

[中文文档](doc/README_zh.md)

A real-time robot control framework for quadruped robots, built on **eProsima Fast-DDS** and designed to bridge reinforcement learning policy inference with low-level hardware actuation.

- **Data-centric architecture**: Decoupled perception, control, and actuation via DDS pub/sub
- **Multi-transport**: UDP, TCP, Shared Memory, Hybrid modes
- **Hardware drivers**: Serial (POSIX) + DM-LinkX CAN-over-EtherCAT
- **Extensible**: Plug-in controllers, simulators, and RL policy engines

---

## Table of Contents

1. [System Architecture](#1-system-architecture)
2. [Build & Run](#2-build--run)
3. [Dependencies](#3-dependencies)
4. [Project Structure](#4-project-structure)
5. [Transport Modes](#5-transport-modes)
6. [Documentation](#6-documentation)
7. [Status & Roadmap](#7-status--roadmap)
8. [License](#8-license)

---

## 1. System Architecture

```
┌──────────────────────────────────────────────────────┐
│                     Core Layer                        │
│    (main executable, Controller, Parameter Manager)   │
├──────────────────────────────────────────────────────┤
│               DDS Middleware (DMW)                     │
│     DdsNode · DdsPublisher<T> · DdsSubscription<T>    │
├─────────────────┬────────────────────────────────────┤
│  Driver Layer   │  Simulation & Policy Layer          │
│  Serial Driver  │  Sim (planned)                      │
│  DM-LinkX/ECAT  │  Policy (planned)                   │
└─────────────────┴────────────────────────────────────┘
```

**Data flow**: Hardware drivers publish sensor data to DDS topics → Control algorithms subscribe and compute commands → Actuation commands are published back to drivers.

## 2. Build & Run

```bash
cmake -B build
cmake --build build --target generate_dds_types
cmake --build build
./build/Core/core_app
```

## 3. Dependencies

| Package | Version | Role |
|---------|---------|------|
| [Fast-DDS](https://fast-dds.docs.eprosima.com/) | ≥ 3.0 | DDS middleware |
| [Fast-CDR](https://github.com/eProsima/Fast-CDR) | ≥ 2.0 | Serialization |
| [fastddsgen](https://fast-dds.docs.eprosima.com/en/latest/fastddsgen/introduction.html) | — | IDL codegen |
| [spdlog](https://github.com/gabime/spdlog) | — | Logging |
| [nlohmann-json](https://github.com/nlohmann/json) | ≥ 3.0 | JSON utility |
| [SOEM](https://github.com/OpenEtherCATsociety/SOEM) | — | EtherCAT master |

## 4. Project Structure

```
├── CMakeLists.txt         # Top-level build
├── idl/                   # DDS IDL definitions (torque.idl)
├── DMW/                   # DDS Middleware (header-only lib)
│   └── include/           # dds_node, dds_pub, dds_sub, utils
├── Drivers/               # Hardware abstraction
│   ├── Logger/            # spdlog-based logging
│   ├── Serial_Driver/     # POSIX serial port
│   └── DM_LinkX_Driver/   # EtherCAT/CAN gateway (submodule)
├── Core/                  # Application entry point
│   ├── main.cpp
│   ├── Controller/        # Placeholder
│   └── Parameter_Manager/ # Placeholder
├── Sim/                   # Placeholder
├── Policy/                # Placeholder
└── doc/                   # Extended documentation
```

## 5. Transport Modes

| Mode | Transport | Scenario |
|------|-----------|----------|
| 0 | UDP + SHM (default) | General purpose |
| 1 | Shared Memory only | Single-machine, low-latency |
| 2 | TCP built-in | Large data, cross-machine |
| 3 | TCP + UDP + SHM | Hybrid deployment |

## 6. Documentation

- [中文文档](doc/README_zh.md)

## 7. Status & Roadmap

- [x] DDS middleware (templated pub/sub)
- [x] IDL code generation pipeline
- [x] Serial driver, EtherCAT driver
- [x] Structured logging with rotation
- [x] ROS 2 compatibility utilities
- [ ] DDS Request/Reply services
- [ ] Control algorithm modules
- [ ] Physics simulation integration
- [ ] RL policy inference deployment

## 8. License

Contact the maintainer for licensing information.
