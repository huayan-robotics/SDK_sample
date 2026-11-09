# HY7 Python Samples

SDK 发布件: HuayanRobotPythonSDK_V1.1.9.0
平台: multi-platform

## 目录结构

```
HY7_Python_Samples/
├── run.bat            # Windows 运行入口
├── run.sh             # Linux 运行入口
└── samples/
    ├── CPS.py
    └── ...
```

## 运行

### Windows

```bat
run.bat
```

### Linux

```bash
bash run.sh
```

或直接运行:

```bash
python samples/showcase_servoJ.py
```

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
