# 原始 `simulatedrobot.zip` 模型整理说明

## 原始包的用途

`simulatedrobot.zip` 是 SolidWorks URDF Exporter 导出的 ROS 描述包，包含：

- `urdf/simulatedrobot.urdf`：机器人 link/joint/inertial/mesh 描述；
- `meshes/*.STL`：主体与 5 个推进器几何；
- `launch/display.launch` 和 `launch/gazebo.launch`：RViz/Gazebo 加载入口。

它可以用于模型显示，但原始包不包含水动力、推进器推力模型、控制器或轨迹任务。

## 本工程对原模型的使用方式

本工程把 URDF 中的 5 个连续关节解释为 5 个推进器安装位：

| 推进器 | 原 joint | 本工程功能 |
|---|---|---|
| front_vertical | `frontwheeljoint` | 垂向推进器，用于深度、俯仰控制 |
| rear_right_vertical | `rbj` | 垂向推进器，用于深度、横滚/俯仰控制 |
| rear_left_vertical | `lbj` | 垂向推进器，用于深度、横滚/俯仰控制 |
| left_horizontal | `lfj` | 水平推进器，用于前进和偏航控制 |
| right_horizontal | `rfj` | 水平推进器，用于前进和偏航控制 |

仿真坐标系定义为：

- body X：前进方向；
- body Y：左侧方向；
- body Z：上方；
- world Z：上方；
- depth = `-world_z`。

原 CAD/URDF 的主体网格大致范围为：`x=[-0.187, 0.156]`, `y=[-1.556, -1.0]`, `z=[-0.081, 0.089]`。因此本工程使用 `[-0.015387, -1.2703, 0.0053548]` 作为仿真 body 原点。

## 对 URDF 的修正

`robot_description/simulatedrobot/urdf/simulatedrobot_fixed.urdf` 做了两个必要修正：

1. `rfw` 原始质量和惯量为 0，已改为与其它小推进器一致，避免物理仿真报错或发散。
2. `base_link` 原始惯性中心疑似 Y/Z 坐标互换，已把 `xyz="-0.015387 0.0053548 1.2703"` 修正为 `xyz="-0.015387 -1.2703 0.0053548"`。

这个 fixed URDF 主要用于 RViz/Gazebo 显示。真正完成定深、转弯和矩形轨迹任务的是 Python 动力学仿真器。
