    # -*- coding: utf-8 -*-
"""Six-axis ServoPPro Cartesian stiffness-control example."""

import math
import sys
import time

from CPS import CPSClient

# BoxID 和 RbtID 可以在此处进行修改
BOX_ID = 0
RBT_ID = 0

def print_ret(cps, command, ret):
    if ret == 0:
        print("[OK]  " + command)
    else:
        error_result = []
        lookup_ret = cps.HRIF_GetErrorCodeStr(BOX_ID, ret, error_result)
        suffix = ": {}".format(error_result[0]) if lookup_ret == 0 and error_result else ""
        print("[ERR] {} -> {}{}".format(command, ret, suffix))

def print_pose(label, pose):
    print(
        "{}: X={:.3f}, Y={:.3f}, Z={:.3f}, "
        "Rx={:.3f}, Ry={:.3f}, Rz={:.3f}".format(label, *pose)
    )

def main():
    print("=== ServoP 6-Axis Cartesian Stiffness Control Example ===")

    # 需根据机器人的实际 IP 进行修改
    host = "192.168.0.10"
    port = 10003

    # ----------------------------------------------------------
    # 1. 连接机器人
    # ----------------------------------------------------------
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
    ret = cps.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2)
    print_ret(cps, "HRIF_SetOverride", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    safe_joints = [0, -90, 90, -90, -90, 0]
    pose = [0.0] * 6

    print("[INFO] Moving to a safe position with MoveJ...")
    ret = cps.HRIF_MoveJ(
        BOX_ID,
        RBT_ID,
        pose,
        safe_joints,
        "TCP",
        "Base",
        50.0,
        50.0,
        0.0,
        1,
        0,
        0,
        0,
        "servoP_home",
    )
    print_ret(cps, "HRIF_MoveJ", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    motion_done = []
    motion_deadline = time.monotonic() + 60.0
    while time.monotonic() < motion_deadline:
        ret = cps.HRIF_IsMotionDone(BOX_ID, RBT_ID, motion_done)
        if ret != 0:
            print_ret(cps, "HRIF_IsMotionDone", ret)
            cps.HRIF_DisConnect(BOX_ID)
            return 1
        if motion_done and motion_done[0]:
            break
        time.sleep(0.05)
    if not motion_done or not motion_done[0]:
        print("[ERR] Timed out waiting for motion completion")
        cps.HRIF_DisConnect(BOX_ID)
        return 1
    print("[INFO] Safe position reached")

    # ----------------------------------------------------------
    # 4. 读取当前 TCP 位姿 (ServoP 的基准)
    # ----------------------------------------------------------
    tcp_pose_result = []
    ret = cps.HRIF_ReadActTcpPos(BOX_ID, RBT_ID, tcp_pose_result)
    print_ret(cps, "HRIF_ReadActTcpPos", ret)
    if ret != 0 or len(tcp_pose_result) < 6:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    current_pose = [float(value) for value in tcp_pose_result[:6]]
    print_pose("[INFO] Current TCP", current_pose)

    ucs = [0.0] * 6
    tcp = [0.0] * 6  # 默认 TCP 坐标系

    # ----------------------------------------------------------
    # 5. 启动 Servo 刚度模式
    # ----------------------------------------------------------
    STIFFNESS_FACTOR = 8000
    LOOKAHEAD_TIME = 0.05
    print(
        "[INFO] Stiffness factor={}, lookahead={}s".format(
            STIFFNESS_FACTOR, LOOKAHEAD_TIME
        )
    )
    ret = cps.HRIF_StartServo(
        BOX_ID, RBT_ID, STIFFNESS_FACTOR, LOOKAHEAD_TIME
    )
    print_ret(cps, "HRIF_StartServo (stiffness)", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    fsm_result = []
    ret = cps.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, fsm_result)
    print_ret(cps, "HRIF_ReadCurFSMFromCPS", ret)
    if ret != 0:
        cps.HRIF_GrpStop(BOX_ID, RBT_ID)
        cps.HRIF_DisConnect(BOX_ID)
        return 1
    nFSM = int(fsm_result[0]) if fsm_result else 0
    print("[INFO] FSM after StartServo={}".format(nFSM))
    time.sleep(0.001)

    # ----------------------------------------------------------
    # 6. 推送首帧 (当前 TCP 位姿，避免突变)
    # ----------------------------------------------------------
    ret = cps.HRIF_PushServoP(BOX_ID, RBT_ID, current_pose, ucs, tcp)
    print_ret(cps, "HRIF_PushServoP (sync)", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 7. 生成笛卡尔轨迹并推送
    # ----------------------------------------------------------
    AMP = 20.0
    FREQ = 0.5
    DURATION = 2.0
    print(
        "[INFO] Streaming a Cartesian trajectory (stiffness={}, "
        "Z sine +/-{} mm, duration={}s)...".format(
            STIFFNESS_FACTOR, AMP, DURATION
        )
    )

    t0 = time.monotonic()
    stream_failed = False

    PUSH_PERIOD = 0.004  # 4 ms (250 Hz)
    deadline = t0 + DURATION
    i = 1
    while time.monotonic() < deadline:
        time.sleep(max(0.0, t0 + i * PUSH_PERIOD - time.monotonic()))
        t = i * PUSH_PERIOD
        i += 1

        target_pose = list(current_pose)
        target_pose[2] = current_pose[2] + AMP * math.sin(
            2.0 * math.pi * FREQ * t
        )

        ret = cps.HRIF_PushServoP(BOX_ID, RBT_ID, target_pose, ucs, tcp)
        if ret != 0:
            print_ret(cps, "HRIF_PushServoP (sample {})".format(i - 1), ret)
            stream_failed = True
            break

    # ----------------------------------------------------------
    # 8. 停止并断开连接
    # ----------------------------------------------------------
    stop_ret = cps.HRIF_GrpStop(BOX_ID, RBT_ID)
    print_ret(cps, "HRIF_GrpStop", stop_ret)
    disconnect_ret = cps.HRIF_DisConnect(BOX_ID)
    print_ret(cps, "HRIF_DisConnect", disconnect_ret)
    return 1 if stream_failed or stop_ret != 0 or disconnect_ret != 0 else 0

if __name__ == "__main__":
    sys.exit(main())
