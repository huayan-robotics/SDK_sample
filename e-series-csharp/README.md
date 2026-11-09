# ESERIAL C# Samples

SDK 发布件: HuayanRobotC#SDK-V1.0.20.0
平台: multi-platform

## 目录结构

```
ESERIAL_CS_Samples/
├── build.bat
├── ESERIAL_CSHARP_Samples.sln
├── lib/
│   ├── RobotLibrarys.dll
│   └── Newtonsoft.Json.dll
├── bin/
│   ├── Debug/          # 所有 Debug 案例和一份 SDK DLL
│   └── Release/        # 所有 Release 案例和一份 SDK DLL
└── samples/
    └── ...
```

## 构建

可以直接用 Visual Studio 打开根目录中的 `.sln`，或执行：

```bat
build.bat
```

## 运行

```bat
cd samples\showcase_servoJ
dotnet run
```

## 案例

- `showcase_stateMachineManage` — state_machine_manage
- `showcase_constantforce` — constant_force_control
- `showcase_freedrive` — force_free_drive
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
- `showcase_TrajectoryPathJ` — trajectory_path_joint
- `showcase_TrajectoryPathL` — trajectory_path_space
- `showcase_TrajectoryPathManage` — trajectory_path_manage
- `showcase_MovePathJOL` — move_path_jol
- `showcase_motionStatusManagement` — motion_status_management

## 连接机器人

案例中默认 IP 为 `192.168.0.10`，请根据实际机器人修改源码中的 IP 地址。
