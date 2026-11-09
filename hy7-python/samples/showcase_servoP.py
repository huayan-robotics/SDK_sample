#!/usr/bin/env python3
    # -*- coding: utf-8 -*-
"""
ServoP (笛卡尔空间在线控制) 演示
"""

import math
import time
import sys
import os

# 导入 CPS SDK
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from CPS import CPSClient

# BoxID 和 RbtID 可以在此处进行修改
BOX_ID = 0
RBT_ID = 0   # 改为 1 即可控制右臂

def print_ret(cps, cmd, ret):
    if ret == 0:
        print("[OK]  " + cmd)
    else:
        err_msg = []
        lookup_ret = cps.HRIF_GetErrorCodeStr(BOX_ID, ret, err_msg)
        suffix = ": {}".format(err_msg[0]) if lookup_ret == 0 and err_msg else ""
        print("[ERR] {} -> {}{}".format(cmd, ret, suffix))

def print_pose(label, x, y, z, rx, ry, rz):
    print("{}: X={:.4f}, Y={:.4f}, Z={:.4f}, Rx={:.4f}, Ry={:.4f}, Rz={:.4f}".format(
        label, x, y, z, rx, ry, rz))

def main():
    print("=== ServoP Cartesian Online Control Example ===")

    # ----------------------------------------------------------
    # 1. 连接机器人
    # ----------------------------------------------------------
    # 需根据机器人的实际 IP 进行修改
    host = "192.168.0.10"
    port = 10003

    cps = CPSClient()
    ret = cps.HRIF_Connect(BOX_ID, host, port)
    print_ret(cps, "HRIF_Connect", ret)
    if ret != 0:
        return 1

    # ----------------------------------------------------------
    # 2. 确认机器人处于 Standby(33)
    # ----------------------------------------------------------
    fsm_result = []
    ret = cps.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, fsm_result)
    nFSM = int(fsm_result[0]) if fsm_result else 0
    print_ret(cps, "HRIF_ReadCurFSM (FSM={})".format(nFSM), ret)
    if ret != 0 or nFSM != 33:
        print("[ERR] Robot must be in Standby(33), FSM={}".format(nFSM))
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 3. 设置速度比并 MoveJ 运动到安全起始位姿
    # ----------------------------------------------------------
    safe_joints = [-90, -42.5, 0, 72.5, 0, -65, 0]
    points = [0.0] * 6   # dX~Rz (IsUseJoint=1 时忽略)

    ret = cps.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2)
    print_ret(cps, "HRIF_SetOverride", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    print("[INFO] Moving to a safe position with MoveJ...")
    ret = cps.HRIF_MoveJ_nJ(BOX_ID, RBT_ID, points, safe_joints,
                            "TCP", "Base", 30.0, 60.0, 0.0,
                            1, 0, 0, 0, "servoP_home")
    print_ret(cps, "HRIF_MoveJ_nJ", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    done = [False]
    deadline = time.monotonic() + 60.0
    while not done[0] and time.monotonic() < deadline:
        time.sleep(0.05)
        ret = cps.HRIF_IsMotionDone(BOX_ID, RBT_ID, done)
        if ret != 0:
            break
    if ret != 0 or not done[0]:
        if ret != 0:
            print_ret(cps, "HRIF_IsMotionDone", ret)
        else:
            print("[ERR] Timed out waiting for motion completion")
        cps.HRIF_DisConnect(BOX_ID)
        return 1
    print("[INFO] Safe position reached")

    # ----------------------------------------------------------
    # 4. 读取当前 TCP 位姿 (ServoP 的基准)
    # ----------------------------------------------------------
    tcp_result = []
    ret = cps.HRIF_ReadActTcpPos(BOX_ID, RBT_ID, tcp_result)
    print_ret(cps,"HRIF_ReadActTcpPos", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1
    dX, dY, dZ, dRx, dRy, dRz = [float(x) for x in tcp_result[:6]]
    print_pose("[INFO] Current TCP", dX, dY, dZ, dRx, dRy, dRz)

    # ----------------------------------------------------------
    # 5. 启动 Servo 模式
    # ----------------------------------------------------------
    SERVO_TIME     = 0.01   # 伺服周期 10ms
    LOOKAHEAD_TIME = 0.05   # 前瞻 50ms
    print("[INFO] Servo cycle={}s, lookahead={}s".format(SERVO_TIME, LOOKAHEAD_TIME))
    ret = cps.HRIF_StartServo(BOX_ID, RBT_ID, SERVO_TIME, LOOKAHEAD_TIME)
    print_ret(cps, "HRIF_StartServo", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # 轮询 FSM 确认进入 Moving(25)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        fsm_result = []
        ret = cps.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, fsm_result)
        if ret != 0:
            print_ret(cps, "HRIF_ReadCurFSMFromCPS", ret)
            cps.HRIF_GrpStop(BOX_ID, RBT_ID)
            cps.HRIF_DisConnect(BOX_ID)
            return 1
        nFSM = int(fsm_result[0]) if fsm_result else 0
        if nFSM == 25:
            break
        time.sleep(0.01)
    if nFSM != 25:
        print("[ERR] Timed out waiting for Moving(25), FSM={}".format(nFSM))
        cps.HRIF_GrpStop(BOX_ID, RBT_ID)
        cps.HRIF_DisConnect(BOX_ID)
        return 1
    print("[INFO] FSM after StartServo={}".format(nFSM))

    # ----------------------------------------------------------
    # 6. 推送首帧 (当前 TCP 位姿，避免突变)
    # ----------------------------------------------------------
    vec_coord = [dX, dY, dZ, dRx, dRy, dRz]
    vec_ucs   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # 默认 Base 坐标系
    vec_tcp   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # 默认 TCP 坐标系

    ret = cps.HRIF_PushServoP(BOX_ID, RBT_ID, vec_coord, vec_ucs, vec_tcp)
    print_ret(cps, "HRIF_PushServoP (sync)", ret)
    if ret != 0:
        cps.HRIF_GrpStop(BOX_ID, RBT_ID)
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 7. 生成笛卡尔轨迹并推送
    # ----------------------------------------------------------
    AMP            = 20.0   # 振幅 20mm
    FREQ           = 0.5    # 频率 0.5Hz
    SAMPLE_COUNT   = 200
    print("[INFO] Streaming a Cartesian trajectory (Z sine, +/-{} mm)...".format(AMP))
    t0 = time.monotonic()
    stream_failed = False
    for sample in range(1, SAMPLE_COUNT + 1):
        deadline = t0 + sample * SERVO_TIME
        time.sleep(max(0.0, deadline - time.monotonic()))
        t = sample * SERVO_TIME

        vec_coord[0] = dX
        vec_coord[1] = dY
        vec_coord[2] = dZ + AMP * math.sin(2.0 * math.pi * FREQ * t)
        vec_coord[3] = dRx
        vec_coord[4] = dRy
        vec_coord[5] = dRz

        ret = cps.HRIF_PushServoP(BOX_ID, RBT_ID, vec_coord, vec_ucs, vec_tcp)
        if ret != 0:
            print_ret(cps, "HRIF_PushServoP (sample {})".format(sample), ret)
            stream_failed = True
            break

    # ----------------------------------------------------------
    # 8. 停止并断开连接
    # ----------------------------------------------------------
    print("[INFO] ServoP completed; disconnecting")
    stop_ret = cps.HRIF_GrpStop(BOX_ID, RBT_ID)
    print_ret(cps, "HRIF_GrpStop", stop_ret)
    disconnect_ret = cps.HRIF_DisConnect(BOX_ID)
    print_ret(cps, "HRIF_DisConnect", disconnect_ret)
    return 1 if stream_failed or stop_ret != 0 or disconnect_ret != 0 else 0

if __name__ == "__main__":
    sys.exit(main())
