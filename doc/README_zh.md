# 四足机器人 RL Simulator — 基于 DDS 的机器人控制框架

[English](../README.md)

基于 **eProsima Fast-DDS** 的实时四足机器人控制框架，旨在连接强化学习策略推理与底层硬件驱动。

- **数据驱动架构**：通过 DDS 发布/订阅解耦感知、控制与执行
- **多传输模式**：UDP、TCP、共享内存、混合模式
- **硬件驱动**：串口 (POSIX) + DM-LinkX CAN-over-EtherCAT
- **可扩展**：可插拔控制器、仿真器与 RL 策略引擎

---

## 目录

1. [系统架构](#1-系统架构)
2. [构建与运行](#2-构建与运行)
3. [依赖项](#3-依赖项)
4. [项目结构](#4-项目结构)
5. [传输模式](#5-传输模式)
6. [文档](#6-文档)
7. [状态与路线图](#7-状态与路线图)
8. [许可证](#8-许可证)

---

## 1. 系统架构

```
┌──────────────────────────────────────────────────────┐
│                     Core Layer                        │
│    (main executable, Controller, Parameter Manager)   │
├──────────────────────────────────────────────────────┤
│               DDS Middleware (DMW)                     │
│     DdsNode · DdsPublisher<T> · DdsSubscription<T>    │
├─────────────────┬────────────────────────────────────┤
│  Driver Layer   │  Simulation & Policy Layer          │
│  Serial Driver  │  Sim（预留）                         │
│  DM-LinkX/ECAT  │  Policy（预留）                      │
└─────────────────┴────────────────────────────────────┘
```

**数据流**：硬件驱动发布传感器数据至 DDS 主题 → 控制算法订阅并计算指令 → 执行指令发布回驱动层。

## 2. 构建与运行

```bash
cmake -B build
cmake --build build --target generate_dds_types
cmake --build build
./build/Core/core_app
```

## 3. 依赖项

| 软件包 | 版本 | 作用 |
|--------|------|------|
| [Fast-DDS](https://fast-dds.docs.eprosima.com/) | ≥ 3.0 | DDS 中间件 |
| [Fast-CDR](https://github.com/eProsima/Fast-CDR) | ≥ 2.0 | 序列化 |
| [fastddsgen](https://fast-dds.docs.eprosima.com/en/latest/fastddsgen/introduction.html) | — | IDL 代码生成 |
| [spdlog](https://github.com/gabime/spdlog) | — | 日志 |
| [nlohmann-json](https://github.com/nlohmann/json) | ≥ 3.0 | JSON 工具 |
| [SOEM](https://github.com/OpenEtherCATsociety/SOEM) | — | EtherCAT 主站 |

## 4. 项目结构

```
├── CMakeLists.txt         # 顶层构建
├── idl/                   # DDS IDL 定义 (torque.idl)
├── DMW/                   # DDS 中间件 (header-only)
│   └── include/           # dds_node, dds_pub, dds_sub, utils
├── Drivers/               # 硬件抽象层
│   ├── Logger/            # spdlog 日志
│   ├── Serial_Driver/     # POSIX 串口
│   └── DM_LinkX_Driver/   # EtherCAT/CAN (子模块)
├── Core/                  # 应用入口
│   ├── main.cpp
│   ├── Controller/        # 预留
│   └── Parameter_Manager/ # 预留
├── Sim/                   # 预留
├── Policy/                # 预留
└── doc/                   # 扩展文档
```

## 5. 传输模式

| 模式 | 传输方式 | 场景 |
|------|----------|------|
| 0 | UDP + SHM (默认) | 通用 |
| 1 | 仅共享内存 | 单机低延迟 |
| 2 | TCP | 大数据量、跨机器 |
| 3 | TCP + UDP + SHM | 混合部署 |

## 6. 文档

- [English](../README.md)

## 7. 状态与路线图

- [x] DDS 中间件（模板化发布/订阅）
- [x] IDL 代码生成流水线
- [x] 串口驱动、EtherCAT 驱动
- [x] 带轮转的日志系统
- [x] ROS 2 兼容工具
- [ ] DDS 请求/回复服务
- [ ] 控制算法模块
- [ ] 物理仿真集成
- [ ] RL 策略推理部署

## 8. 许可证

请联系维护者获取许可信息。
