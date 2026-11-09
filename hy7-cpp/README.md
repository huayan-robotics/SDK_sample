# HY7 C++ Samples

SDK 发布件: HuayanRobotLibrary-C++-V1.0.18.0
平台: multi-platform

## 目录结构

```
HY7_CPP_Samples/
├── CMakeLists.txt
├── build.py           # Windows / Linux 统一构建脚本
├── lib/               # MSVC、MinGW、Linux 全部库
└── samples/
    ├── HR_Pro.h
    └── ...
```

## 构建

Windows 和 Linux 使用同一个命令：

```bash
python build.py --config Release
python build.py --config Debug
```

CMake 会自动识别 MSVC、MinGW 或 Linux，并选择 `lib/` 中匹配的库。

## 案例

- `showcase_stateMachineManage` — state_machine_manage
- `showcase_robotErrorReset` — robot_error_reset
- `showcase_servoJ` — servo_j
- `showcase_servoP` — servo_p
- `showcase_servoJPro` — servo_j_pro
- `showcase_servoPPro` — servo_p_pro
- `showcase_speedJ` — speed_j
- `showcase_speedL` — speed_l
- `showcase_motion` — motion_abs
- `showcase_motionrel` — motion_rel
- `showcase_jog` — motion_jog
- `showcase_motionStatusManagement` — motion_status_management

## 连接机器人

案例中默认 IP 为 `192.168.0.10`，请根据实际机器人修改源码中的 IP 地址。
