# Five-Propeller Underwater Robot | 五推进器水下机器人

[中文](#中文说明) · [English](#english-version) · [中文独立版 / Chinese-only version](README.zh-CN.md)

[![License: MIT](https://img.shields.io/badge/license-MIT-yellow.svg)](LICENSE)
![Platform](https://img.shields.io/badge/MCU-STM32F407-03234B)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-2C9F45)
![Thrusters](https://img.shields.io/badge/thrusters-3%20vertical%20%2B%202%20horizontal-167D9A)
![Status](https://img.shields.io/badge/status-course%20prototype-orange)

![SolidWorks 最终装配体 / Final SolidWorks assembly](docs/assets/cad-assembly.png)

## 中文说明

本项目是一款基于 STM32F407 和 FreeRTOS 的小型五推进器水下机器人课程设计。机器人采用三个垂直推进器控制升沉、俯仰和横滚，两个水平推进器控制前进、后退和偏航。仓库已将嵌入式固件、SolidWorks 机械设计、LoRa 上位机、Python 六自由度仿真、ROS1 模型、样机照片和工程说明整理为可直接浏览和审查的源文件。

> [!CAUTION]
> 本项目是教学与研究原型，不是经过认证的水下设备。当前代码中的深度换算结果可能约小 10 倍，主动控制模式也尚未实现 LoRa 指令丢失看门狗。在接通推进器或进行闭环水池试验前，请先阅读[安全说明与已知限制](docs/safety.md)，并核对接线、推进器方向和保护措施。

### 项目功能

- 五推进器驱动：三个垂直推进器控制升沉、俯仰和横滚，两个水平推进器控制纵向运动和偏航。
- 支持 `IDLE`、定深 `HOVER`、航向保持 `AUTO`、绝对航向转向 `TURN` 和定时方形轨迹 `SQUARE` 模式。
- 使用四个 PID 控制环、解析式推进器混控和按比例限幅。
- FreeRTOS 分离控制、压力采样、通信和心跳任务。
- 使用 H30 串口 IMU、MS5837-30BA 压力传感器、E22-400T30D LoRa 和五路 ESC PWM，并以 2 Hz 返回遥测数据。
- 提供基于 Python/pyserial 的指令终端、状态显示和 CSV 日志记录。
- 提供独立 Python 六自由度任务仿真和 ROS1 URDF/STL 可视化模型。

### 样机与水池试验

| 最终装配样机 | 方形轨迹试验画面 |
| --- | --- |
| ![最终装配阶段的样机](docs/assets/prototype-assembly.jpg) | ![水池方形轨迹试验画面](docs/assets/pool-square-test.png) |

课程报告记录了定深、直线航行、转向和定时方形轨迹等演示。这些试验可以说明原型方案具有可行性，但不能视为耐压等级、导航精度或安全认证依据。

### 系统组成

![电气与通信架构](docs/assets/system-architecture.png)

| 层级 | 主要组件 | 职责 |
| --- | --- | --- |
| 传感器 | H30 IMU、MS5837-30BA | 姿态、加速度、压力和深度测量 |
| 控制器 | STM32F407 + FreeRTOS | 状态机、PID、推进器混控和通信 |
| 执行机构 | 5 个 ESC 驱动推进器 | 升沉、俯仰、横滚、纵向运动和偏航 |
| 通信 | E22 LoRa（UART/DMA） | 指令接收和遥测发送 |
| 上位机 | Python + pyserial | 指令输入、状态显示和 CSV 日志 |
| 数值仿真 | Python + NumPy/Matplotlib | 六自由度模型和可复现实例 |
| 机器人可视化 | ROS1 URDF/STL | 几何模型和关节显示 |

### 硬件概况

实验报告列出的主要硬件包括 STM32F407、H30 IMU、MS5837-B30 压力传感器、E22-400T30D 无线模块、三星 21700-50S 电芯定制电池组、两个已安装的 Viocean T60 水平推进器、三个垂直推进器，以及直径 130 mm、长度 250 mm 的透明耐压舱。历史项目总预算为人民币 2,441.3 元。数量差异、备用推进器和缺失的电气资料请参见[硬件与预算说明](docs/hardware.md)。

### 仓库结构

```text
Five-Propeller-Underwater-Robot/
├── firmware/                 STM32F407 CubeMX/CMake 固件
├── tools/pc/                 Python LoRa 上位机与遥测记录工具
├── mechanical/solidworks/    SolidWorks 零件与装配体源文件
├── simulation/python/        独立六自由度数值仿真
├── simulation/ros/           ROS1 URDF/STL 可视化包
├── docs/                     架构、算法、协议、硬件和代码审查文档
├── .github/                  Issue 与 Pull Request 模板
├── README.md                 中英双语主页
├── README.zh-CN.md           独立中文说明
└── LICENSE
```

原有的两个 ZIP 压缩包已展开为源文件目录，因此可以直接在 GitHub 中浏览、搜索、比较和审查。

### 快速开始

#### 1. 查看或编译固件

需要 CMake 3.22+、Ninja 和 Arm GNU Toolchain（`arm-none-eabi-gcc`）。

```bash
cd firmware
cmake --preset Debug
cmake --build --preset Debug
```

可使用 STM32CubeMX 查看或重新生成 `firmware/unify_v1_0307.ioc`。烧录前必须结合实物核对引脚、接线、PWM 中值与范围、传感器设置、电机方向和安全限值。详见[固件说明](firmware/README.md)。

#### 2. 运行 LoRa 上位机

```bash
python -m venv .venv
# Windows PowerShell: .venv\Scripts\Activate.ps1
python -m pip install -r tools/pc/requirements.txt
python tools/pc/lora_pc.py COM6 9600
```

请将 `COM6` 和 `9600` 替换为实际串口和波特率。详见[上位机说明](tools/pc/README.md)和[通信协议](docs/protocol.md)。

#### 3. 运行数值仿真

```bash
cd simulation/python
python -m pip install -r requirements.txt
python scripts/smoke_test.py
python run_all_tasks.py
```

仿真参数目前属于工程假设，仿真误差（尤其是很小的深度 RMSE）不能作为实物精度。详见[仿真说明](simulation/python/README.md)。

#### 4. 查看 ROS 模型

将 `simulation/ros/simulatedrobot` 放入 ROS1 catkin 工作空间，然后运行：

```bash
roslaunch simulatedrobot display_fixed.launch
```

该包用于几何可视化，不是经过验证的水下 Gazebo 动力学模型。详见[ROS/URDF 说明](simulation/ros/README.md)。

### 固件控制模式

| 模式 | 行为 |
| --- | --- |
| `IDLE` | 输出中性 PWM，并清空所有 PID 状态 |
| `HOVER` | 保持目标深度和水平姿态，纵向推力为零 |
| `AUTO` | 保持目标深度和进入模式时的航向，同时施加设定纵向推力 |
| `TURN` | 停止纵向推进并转到指定偏航角；误差进入 ±3° 后切换为悬停 |
| `SQUARE` | 交替执行定时直线航段和 90° 转向，共完成四条边 |

任务时序、PID 参数、控制方程、混控映射、仿真差异和调参顺序见[控制算法说明](docs/control-algorithm.md)。

### 项目文档

- [系统架构与数据流](docs/architecture.md)
- [控制算法](docs/control-algorithm.md)
- [LoRa 通信协议](docs/protocol.md)
- [硬件与预算](docs/hardware.md)
- [静态代码审查结果](docs/code-review.md)
- [安全说明与已知限制](docs/safety.md)
- [机械设计文件](mechanical/README.md)

### 当前工程限制

- 必须修正压力到深度的换算，并通过实物试验标定。
- 指令和 IMU 解析器需要更严格的边界、长度和有限数值检查。
- 主动控制模式需要独立的指令超时看门狗。
- PID 参数和仿真水动力参数尚无提交到仓库的辨识数据集支撑。
- 缺少硬件原理图、连接器引脚表、保险丝、BMS 细节和经过验证的电流预算。
- URDF 仍是简化的可视化模型，没有水下动力学插件。

详细问题和修改建议记录在[代码审查结果](docs/code-review.md)中。

### 参与贡献与许可证

请阅读[贡献指南](CONTRIBUTING.md)。源代码、配置和文档应以普通 Git 文件提交，请勿重新提交编译目录、IDE 缓存、仿真输出、遥测日志或替代用 ZIP 压缩包。本仓库采用 [MIT License](LICENSE)；STM32 HAL、CMSIS、FreeRTOS 和其他第三方组件仍遵循各自源码目录中的许可声明。

---

## English version

This repository contains an STM32F407/FreeRTOS course project for a compact underwater robot with three vertical and two horizontal thrusters. It brings the firmware, SolidWorks design, PC LoRa tool, Python 6-DOF simulation, ROS1 model, prototype images, and engineering notes together as directly reviewable source files.

> [!CAUTION]
> This is a teaching and research prototype, not a certified underwater vehicle. The current depth conversion appears to be approximately 10× too small, and active modes do not yet have a LoRa command-loss watchdog. Read [Safety and known limitations](docs/safety.md), and verify wiring, motor direction, and protection measures before powering the thrusters or attempting closed-loop pool tests.

### Highlights

- Five-thruster actuation: three vertical units for heave, pitch, and roll, plus two horizontal units for surge and yaw.
- `IDLE`, depth `HOVER`, heading-hold `AUTO`, absolute-yaw `TURN`, and timed `SQUARE` modes.
- Four firmware PID loops with an analytic mixer and proportional saturation handling.
- FreeRTOS separation of control, pressure sampling, communication, and heartbeat work.
- H30 serial IMU, MS5837-30BA pressure sensor, E22-400T30D LoRa, five ESC PWM channels, and 2 Hz telemetry.
- Python/pyserial command console with live status and CSV logging.
- Standalone Python 6-DOF task simulation and ROS1 URDF/STL visualization.

### Prototype and pool test

The images in the [Chinese section](#样机与水池试验) show the final assembly and a square-path pool test. The course report records demonstrations of depth control, straight motion, turning, and a timed square sequence. These tests establish prototype feasibility; they do not establish a pressure rating, navigation accuracy, or safety certification.

### System overview

| Layer | Main components | Responsibility |
| --- | --- | --- |
| Sensors | H30 IMU, MS5837-30BA | Attitude, acceleration, pressure, and depth |
| Controller | STM32F407 + FreeRTOS | State machine, PID, mixing, and communication |
| Actuation | 5 ESC-driven thrusters | Heave, pitch, roll, surge, and yaw |
| Communication | E22 LoRa over UART/DMA | Commands and telemetry |
| Ground station | Python + pyserial | Command entry, display, and CSV logging |
| Numerical simulation | Python + NumPy/Matplotlib | 6-DOF model and repeatable task examples |
| Robot visualization | ROS1 URDF/STL | Geometry and joint display |

### Hardware snapshot

The report lists an STM32F407, H30 IMU, MS5837-B30 sensor, E22-400T30D radio, a custom Samsung 21700-50S battery pack, two installed Viocean T60 horizontal thrusters, three vertical thrusters, and a 130 mm × 250 mm transparent housing. Its historical project total is CNY 2,441.3. See [Hardware and budget](docs/hardware.md) for quantities, the spare-thruster discrepancy, and missing electrical documentation.

### Repository layout

The bilingual directory tree is shown in the [Chinese section](#仓库结构). The two original ZIP archives were replaced by source trees so GitHub can browse, search, diff, and review the project directly.

### Quick start

#### 1. Inspect or build the firmware

Requirements: CMake 3.22+, Ninja, and Arm GNU Toolchain (`arm-none-eabi-gcc`).

```bash
cd firmware
cmake --preset Debug
cmake --build --preset Debug
```

Use STM32CubeMX to inspect or regenerate `firmware/unify_v1_0307.ioc`. Pin assignments, wiring, PWM neutral/range, sensor configuration, motor direction, and safety limits must be verified against the actual hardware before flashing. See [Firmware notes](firmware/README.md).

#### 2. Run the PC LoRa console

```bash
python -m venv .venv
# Windows PowerShell: .venv\Scripts\Activate.ps1
python -m pip install -r tools/pc/requirements.txt
python tools/pc/lora_pc.py COM6 9600
```

Replace `COM6` and `9600` with the real port and configured baud rate. See [PC tool instructions](tools/pc/README.md) and the [protocol specification](docs/protocol.md).

#### 3. Run the numerical simulation

```bash
cd simulation/python
python -m pip install -r requirements.txt
python scripts/smoke_test.py
python run_all_tasks.py
```

Simulation parameters are provisional engineering assumptions. Numerical errors—especially very small depth RMSE values—must not be presented as measured vehicle accuracy. See [Simulation notes](simulation/python/README.md).

#### 4. View the ROS model

Place `simulation/ros/simulatedrobot` in a ROS1 catkin workspace and run:

```bash
roslaunch simulatedrobot display_fixed.launch
```

The package is a geometry visualization export, not a validated underwater Gazebo model. See [ROS/URDF notes](simulation/ros/README.md).

### Firmware control modes

| Mode | Behavior |
| --- | --- |
| `IDLE` | Output neutral PWM and reset all PID state |
| `HOVER` | Hold target depth, level pitch/roll, and use zero surge |
| `AUTO` | Hold depth and the entry heading while applying requested surge |
| `TURN` | Stop surge and rotate to a requested yaw; switch to hover within ±3° |
| `SQUARE` | Alternate timed straight legs and 90° yaw turns for four sides |

See [Control algorithm](docs/control-algorithm.md) for task timing, gains, equations, mixer mapping, simulation differences, and tuning order.

### Documentation

- [Architecture and data flow](docs/architecture.md)
- [Control algorithm](docs/control-algorithm.md)
- [LoRa protocol](docs/protocol.md)
- [Hardware and budget](docs/hardware.md)
- [Static code review findings](docs/code-review.md)
- [Safety and known limitations](docs/safety.md)
- [Mechanical design files](mechanical/README.md)

### Current engineering limitations

- The pressure-to-depth conversion must be corrected and physically calibrated.
- Command and IMU parsers need stricter bounds, exact-length, and finite-value validation.
- Active modes need an independent command-age watchdog.
- PID gains and simulator hydrodynamic parameters are not backed by a committed identification dataset.
- Hardware schematics, connector pinouts, fusing, BMS details, and a verified current budget are missing.
- The URDF remains a simplified visualization model without underwater dynamics plugins.

Details and suggested corrections are tracked in [Code review findings](docs/code-review.md).

### Contributing and license

See [CONTRIBUTING.md](CONTRIBUTING.md). Keep source, configuration, and documentation as normal Git files; do not recommit build directories, IDE caches, simulation outputs, telemetry logs, or replacement ZIP archives. This repository is released under the [MIT License](LICENSE). STM32 HAL, CMSIS, FreeRTOS, and other third-party components retain the notices included in their own source directories.
