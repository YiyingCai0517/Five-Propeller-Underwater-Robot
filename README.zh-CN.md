# 五推进器水下机器人

[English](README.md)

[![License: MIT](https://img.shields.io/badge/license-MIT-yellow.svg)](LICENSE)
![Platform](https://img.shields.io/badge/MCU-STM32F407-03234B)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-2C9F45)
![Thrusters](https://img.shields.io/badge/thrusters-3%20vertical%20%2B%202%20horizontal-167D9A)
![Status](https://img.shields.io/badge/status-course%20prototype-orange)

本项目是一套基于 STM32F407 和 FreeRTOS 的小型五推进器水下机器人课程设计。仓库将固件源码、SolidWorks 机械设计、LoRa 上位机、Python 六自由度仿真、ROS1 模型、实物图片和工程说明整理为可直接浏览、搜索和评审的源文件。

![最终 SolidWorks 总装模型](docs/assets/cad-assembly.png)

> [!CAUTION]
> 本项目是教学/研究原型，不是经过认证的水下装备。当前深度换算系数疑似小约 10 倍，主动控制模式也没有 LoRa 指令丢失看门狗。给推进器上电或进行闭环水池测试前，请先阅读[安全说明与已知限制](docs/safety.md)。

## 样机实现内容

- 三个垂直推进器控制升沉、俯仰和横滚，两个水平推进器控制前进和偏航。
- 支持 `IDLE`、定深 `HOVER`、航向保持 `AUTO`、指定航向 `TURN` 和定时 `SQUARE` 模式。
- 固件包含深度、俯仰、横滚、偏航四路 PID，以及解析式五推进器混控和比例限幅。
- FreeRTOS 将闭环控制、压力采集、通信和心跳拆分为独立任务。
- 集成 H30 串口 IMU、MS5837-30BA 深度传感器、E22-400T30D LoRa、五路 ESC PWM 和 2 Hz 遥测。
- Python 上位机支持指令输入、状态显示和 CSV 数据记录。
- 提供独立 Python 六自由度任务仿真，以及 ROS1 URDF/STL 可视化模型。

## 样机与水池测试

| 最终装配状态 | 方形轨迹测试画面 |
| --- | --- |
| ![样机最终装配状态](docs/assets/prototype-assembly.jpg) | ![方形轨迹水池测试](docs/assets/pool-square-test.png) |

课程报告记录了定深、直航、转向和定时方形轨迹演示。这些结果说明样机具备基本可行性，但不代表已经获得耐压等级、导航精度或安全认证。

## 系统概览

![电控与通信总体架构](docs/assets/system-architecture.png)

| 层级 | 主要组件 | 职责 |
| --- | --- | --- |
| 传感器 | H30 IMU、MS5837-30BA | 姿态、加速度、压力和深度 |
| 控制器 | STM32F407 + FreeRTOS | 状态机、PID、混控和通信 |
| 执行器 | 5 路 ESC 推进器 | 升沉、俯仰、横滚、前进和偏航 |
| 通信 | E22 LoRa + UART/DMA | 指令接收和遥测回传 |
| 上位机 | Python + pyserial | 命令、状态显示和 CSV 记录 |
| 数值仿真 | Python + NumPy/Matplotlib | 六自由度模型和可复现实验任务 |
| 机器人可视化 | ROS1 URDF/STL | 几何与关节显示 |

## 硬件概况

报告中的主要选型包括 STM32F407、H30 姿态传感器、MS5837-B30 深度传感器、E22-400T30D LoRa、三星 21700-50S 定制电池包、两台实际安装的 Viocean T60 水平推进器、三台垂直推进器和 130 mm × 250 mm 透明密封舱。历史总预算为人民币 2441.3 元。数量差异、备用推进器和尚缺少的电气资料见[硬件与预算清单](docs/hardware.md)。

## 仓库结构

```text
Five-Propeller-Underwater-Robot/
├─ firmware/                 STM32F407 CubeMX/CMake 固件
├─ tools/pc/                 Python LoRa 控制台与遥测记录
├─ mechanical/solidworks/    SolidWorks 原生零件和装配体
├─ simulation/python/        独立六自由度数值仿真
├─ simulation/ros/           ROS1 URDF/STL 可视化包
├─ docs/                     架构、算法、协议、硬件、代码审查
├─ .github/                  Issue 与 Pull Request 模板
├─ README.md
├─ README.zh-CN.md
└─ LICENSE
```

原有两个 ZIP 压缩包已由展开后的源码树替代，因此 GitHub 页面现在可以直接浏览、搜索、比较和评审项目内容。

## 快速开始

### 1. 查看或编译固件

需要 CMake 3.22+、Ninja 和 Arm GNU Toolchain（`arm-none-eabi-gcc`）。

```bash
cd firmware
cmake --preset Debug
cmake --build --preset Debug
```

可使用 STM32CubeMX 查看或重新生成 `firmware/unify_v1_0307.ioc`。烧录前必须根据实物核对引脚、接线、PWM 中位与范围、传感器配置、电机方向和安全限值。详见[固件说明](firmware/README.md)。

### 2. 运行 LoRa 上位机

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r tools\pc\requirements.txt
python tools\pc\lora_pc.py COM6 9600
```

请将 `COM6` 和 `9600` 改为实际端口与波特率。详见[上位机使用说明](tools/pc/README.md)和[通信协议](docs/protocol.md)。

### 3. 运行数值仿真

```powershell
cd simulation\python
python -m pip install -r requirements.txt
python scripts\smoke_test.py
python run_all_tasks.py
```

仿真参数包含尚未实测辨识的工程假设。数值仿真中的极小深度 RMSE 不能表述为样机实测精度。详见[仿真说明](simulation/python/README.md)。

### 4. 查看 ROS 模型

将 `simulation/ros/simulatedrobot` 放入 ROS1 catkin 工作空间，然后运行：

```bash
roslaunch simulatedrobot display_fixed.launch
```

该包主要用于几何可视化，不是已经验证的水下 Gazebo 动力学模型。详见 [ROS/URDF 说明](simulation/ros/README.md)。

## 固件工作模式

| 模式 | 行为 |
| --- | --- |
| `IDLE` | 五路 PWM 回中并清空 PID 状态 |
| `HOVER` | 保持目标深度和水平姿态，前进推力为零 |
| `AUTO` | 保持深度和进入模式时的航向，同时输出指定前进推力 |
| `TURN` | 停止前进并转到目标航向，进入 ±3° 后转为悬停 |
| `SQUARE` | 定时直行与 90° 转向交替，完成四条边 |

任务周期、PID 参数、混控公式、仿真差异和建议整定顺序见[控制算法说明](docs/control-algorithm.md)。

## 文档导航

- [系统架构与数据流](docs/architecture.md)
- [控制算法说明](docs/control-algorithm.md)
- [LoRa 通信协议](docs/protocol.md)
- [硬件与预算清单](docs/hardware.md)
- [静态代码审查结果](docs/code-review.md)
- [安全说明与已知限制](docs/safety.md)
- [机械设计文件说明](mechanical/README.md)

## 当前工程限制

- 必须修正压力到深度的换算并完成实测校准。
- LoRa 和 IMU 解析器需要更严格的边界、精确长度和有限值检查。
- 主动控制模式需要独立的指令超时看门狗。
- PID 参数和仿真水动力参数缺少仓库内可追溯的系统辨识数据。
- 仓库尚缺原理图、连接器针脚表、保险丝/BMS 说明和经过核对的电流预算。
- URDF 仍是简化可视化模型，不含经验证的水下动力学插件。

详细问题与建议修复方法记录在[代码审查结果](docs/code-review.md)中，便于后续贡献者逐项处理，而不是继续隐藏在二进制压缩包内。

## 参与贡献

请阅读 [CONTRIBUTING.md](CONTRIBUTING.md)。源码、配置和文档应作为普通 Git 文件维护，不要重新提交构建目录、IDE 缓存、仿真输出、遥测日志或替代 ZIP 压缩包。

## 许可证

本仓库采用 [MIT License](LICENSE)。STM32 HAL、CMSIS、FreeRTOS 和其他第三方组件继续遵循其自身源目录内的原有声明。
