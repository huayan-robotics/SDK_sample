#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Motion status management example for seven-axis robots."""

import sys
import time

from CPS import CPSClient


# BoxID 和 RbtID 可以在此处进行修改
BOX_ID = 0
RBT_ID = 0


def print_ret(cps, command, ret):
    if ret == 0:
        print("[OK]  " + command)
        return True

    message = []
    lookup_ret = cps.HRIF_GetErrorCodeStr(BOX_ID, ret, message)
    suffix = ": {}".format(message[0]) if lookup_ret == 0 and message else ""
    print("[ERR] {} -> {}{}".format(command, ret, suffix), file=sys.stderr)
    return False


def wait_motion_done(cps):
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        done = []
        ret = cps.HRIF_IsMotionDone(BOX_ID, RBT_ID, done)
        if ret != 0:
            print_ret(cps, "HRIF_IsMotionDone", ret)
            return False
        if done and done[0]:
            return True
        time.sleep(0.5)

    print("[ERR] Motion timeout (60s).", file=sys.stderr)
    return False


def main():
    print("======== Motion Status Management Example ========")
    cps = CPSClient()

    # ----------------------------------------------------------
    # 1. 连接机器人，确认处于 Standby(33) 并设置速度比为 0.2
    # ----------------------------------------------------------
    # 需根据机器人的实际 IP 进行修改
    host = "192.168.0.10"
    port = 10003

    ret = cps.HRIF_Connect(BOX_ID, host, port)
    print_ret(cps, "HRIF_Connect", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    result = []
    ret = cps.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, result)
    fsm = int(result[0]) if result else 0
    print_ret(cps, "HRIF_ReadCurFSM (FSM={})".format(fsm), ret)
    if ret != 0 or fsm != 33:
        print("[ERR] Robot must be in Standby(33), FSM={}".format(fsm), file=sys.stderr)
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    ret = cps.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2)
    print_ret(cps, "HRIF_SetOverride", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 2. MoveJ_nJ 运动到安全原点
    # ----------------------------------------------------------
    safe_joints = [-90, -42.5, 0, 72.5, 0, -65, 0]
    zero_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    ret = cps.HRIF_MoveJ_nJ(BOX_ID, RBT_ID, zero_pose, safe_joints, "TCP", "Base", 30.0, 60.0,
                            0.0, 1, 0, 0, 0, "moveJ_to_safepoints")
    print_ret(cps, "HRIF_MoveJ_nJ (safepoint)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 3. 连续下发用于演示暂停、继续和停止的 MoveJ_nJ 运动
    # ----------------------------------------------------------
    ret = cps.HRIF_SetBoxDO(BOX_ID, 0, 0)
    print_ret(cps, "HRIF_SetBoxDO (DO0=0)", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    safe_joints[0] += 10.0
    ret = cps.HRIF_MoveJ_nJ(BOX_ID, RBT_ID, zero_pose, safe_joints, "TCP", "Base", 30.0, 60.0,
                            0.0, 1, 0, 0, 0, "motion_status_interrupt")
    print_ret(cps, "HRIF_MoveJ_nJ (interrupt demo)", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    ret = cps.HRIF_cdsSetIO(BOX_ID, RBT_ID, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0.0, 0.0)
    print_ret(cps, "HRIF_cdsSetIO", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    safe_joints[0] -= 10.0
    ret = cps.HRIF_MoveJ_nJ(BOX_ID, RBT_ID, zero_pose, safe_joints, "TCP", "Base", 30.0, 60.0,
                            0.0, 1, 0, 0, 0, "motion_status_stop")
    print_ret(cps, "HRIF_MoveJ_nJ (stop demo)", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 4. 根据运动 ID 暂停、继续和停止，并通过输出 IO 判断运动是否完成
    # ----------------------------------------------------------
    command_id = ""
    while command_id != "motion_status_interrupt":
        result = []
        ret = cps.HRIF_ReadCurWaypointID(BOX_ID, RBT_ID, result)
        if ret != 0:
            print_ret(cps, "HRIF_ReadCurWaypointID", ret)
            cps.HRIF_DisConnect(BOX_ID)
            return 1
        command_id = str(result[0]) if result else ""
        time.sleep(0.1)
    print("[INFO] Current motion stage ID is: " + command_id)

    ret = cps.HRIF_GrpInterrupt(BOX_ID, RBT_ID)
    print_ret(cps, "HRIF_GrpInterrupt", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    time.sleep(1.0)
    ret = cps.HRIF_GrpContinue(BOX_ID, RBT_ID)
    print_ret(cps, "HRIF_GrpContinue", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    command_id = ""
    while command_id != "motion_status_stop":
        result = []
        ret = cps.HRIF_ReadCurWaypointID(BOX_ID, RBT_ID, result)
        if ret != 0:
            print_ret(cps, "HRIF_ReadCurWaypointID", ret)
            cps.HRIF_DisConnect(BOX_ID)
            return 1
        command_id = str(result[0]) if result else ""
        time.sleep(0.1)
    print("[INFO] Current motion stage ID is: " + command_id)

    ret = cps.HRIF_GrpStop(BOX_ID, RBT_ID)
    print_ret(cps, "HRIF_GrpStop", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    do0 = 0
    while do0 != 1:
        result = []
        ret = cps.HRIF_ReadBoxDO(BOX_ID, 0, result)
        if ret != 0:
            print_ret(cps, "HRIF_ReadBoxDO", ret)
            cps.HRIF_DisConnect(BOX_ID)
            return 1
        do0 = int(result[0]) if result else 0
        time.sleep(0.1)

    print("[Info] Motion Done! Disconnecting")
    ret = cps.HRIF_DisConnect(BOX_ID)
    print_ret(cps, "HRIF_DisConnect", ret)
    return 0 if ret == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
