# Simulator Entrypoints

这个目录放两个最小可用的仿真入口：一个是 MuJoCo，一个是 Gazebo Sim。它们都会从 `descriptions` 下交互式选择机器人模型，并把选中的模型放进仿真环境。

## MuJoCo

依赖：

在当前 Python 虚拟环境里安装：

```bash
pip install mujoco numpy
```

启动：

先列出可用模型：

```bash
/home/ma/quadruped_rl/.venv/bin/python Sim/mujoco_model_selector.py --list
```

交互式选择并启动仿真：

```bash
/home/ma/quadruped_rl/.venv/bin/python Sim/mujoco_model_selector.py
```

直接指定模型并启动：

```bash
/home/ma/quadruped_rl/.venv/bin/python Sim/mujoco_model_selector.py --model go1_description
```

可选参数：

- `--x`, `--y`, `--z`：模型初始放置位置
- `--yaw`：绕 Z 轴初始朝向，单位弧度

## 说明

脚本读取的是各个描述包里已经生成好的 `urdf/robot.urdf`，并只使用其中的 collision 几何来构建 MuJoCo 场景。这样不依赖网格文件，也能在本地直接跑起来。

如果你已经激活了虚拟环境，也可以直接用 `python` 启动；不要直接用系统 `python3`，否则可能找不到 `mujoco` 依赖。

## Gazebo Sim

依赖：

- 系统里需要有 `gz` 命令可用
- 如果你已经配置好 ROS 2 / Gazebo 环境，通常直接在当前 shell 里启动即可

启动：

```bash
/home/ma/quadruped_rl/.venv/bin/python Sim/gazebo_model_selector.py --list
```

```bash
/home/ma/quadruped_rl/.venv/bin/python Sim/gazebo_model_selector.py
```

```bash
/home/ma/quadruped_rl/.venv/bin/python Sim/gazebo_model_selector.py --model go1_description
```

可选参数：

- `--x`, `--y`, `--z`：模型初始放置位置
- `--yaw`：绕 Z 轴初始朝向，单位弧度

脚本会先把选中的 URDF 转成 SDF world，再调用 `gz sim` 打开仿真。它同样支持 `--check-only`，用于只生成并验证 world，不真正启动 GUI。