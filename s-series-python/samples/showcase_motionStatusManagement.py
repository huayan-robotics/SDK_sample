#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Motion status management example."""

import sys
import time

from CPS import CPSClient


# BoxID 和 RbtID 可以在此处进行修改
box_id = 0
rbt_id = 0


def print_ret(cps, command, ret):
    if ret == 0:
        print("[OK]  " + command)
        return True

    error_message = []
    lookup_ret = cps.HRIF_GetErrorCodeStr(box_id, ret, error_message)
    suffix = ": {}".format(error_message[0]) if lookup_ret == 0 and error_message else ""
    print("[ERR] {} -> {}{}".format(command, ret, suffix), file=sys.stderr)
    return False


def wait_motion_done(cps):
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        done = []
        ret = cps.HRIF_IsMotionDone(box_id, rbt_id, done)
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
    # 1. 连接机器人，确认处于使能状态 Standby(33) 并设置速度比为 0.2
    # ----------------------------------------------------------
    # 需根据机器人的实际 IP 进行修改
    host = "192.168.0.10"
    port = 10003

    ret = cps.HRIF_Connect(box_id, host, port)
    print_ret(cps, "HRIF_Connect", ret)
    if ret != 0:
        cps.HRIF_DisConnect(box_id)
        return 1

    result = []
    ret = cps.HRIF_ReadCurFSMFromCPS(box_id, rbt_id, result)
    fsm = int(result[0]) if result else 0
    print_ret(cps, "HRIF_ReadCurFSM (FSM={})".format(fsm), ret)
    if ret != 0 or fsm != 33:
        print("[ERR] Robot must be in Standby(33), FSM={}".format(fsm), file=sys.stderr)
        cps.HRIF_DisConnect(box_id)
        return 1

    ret = cps.HRIF_SetOverride(box_id, rbt_id, 0.2)
    print_ret(cps, "HRIF_SetOverride", ret)
    if ret != 0:
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 2. MoveJ 运动到安全原点
    # ----------------------------------------------------------
    safe_joints = [0, -90, 90, -90, -90, 0]

    ret = cps.HRIF_MoveJ(box_id, rbt_id, [0, 0, 0, 0, 0, 0], safe_joints,
                         "TCP", "Base", 30.0, 60.0, 0.0, 1, 0, 0, 0, "moveJ_to_safepoints")
    print_ret(cps, "HRIF_MoveJ (safepoint)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(box_id)
        return 1
    
    # ----------------------------------------------------------
    # 3. 连续下发用于演示暂停继续和停止的 MoveJ 运动，并设置输出 IO 
    # ----------------------------------------------------------
    ret = cps.HRIF_SetBoxDO(box_id, 0, 0)
    print_ret(cps, "HRIF_SetBoxDO (DO0=0)", ret)
    if ret != 0:
        cps.HRIF_DisConnect(box_id)
        return 1

    dJ1, dJ2, dJ3, dJ4, dJ5, dJ6 = safe_joints
    ret = cps.HRIF_MoveJ(box_id, rbt_id, [0, 0, 0, 0, 0, 0], [dJ1 + 50.0, dJ2, dJ3, dJ4, dJ5, dJ6],
                         "TCP", "Base", 30.0, 60.0, 0.0, 1, 0, 0, 0, "motion_status_interrupt")
    print_ret(cps, "HRIF_MoveJ (interrupt demo)", ret)
    if ret != 0:
        cps.HRIF_DisConnect(box_id)
        return 1

    ret = cps.HRIF_cdsSetIO(box_id, rbt_id, 0, 0, 1, 1, 0, 0,
                            0, 1, 0, 1, 0.0, 0.0)
    print_ret(cps, "HRIF_cdsSetIO", ret)
    if ret != 0:
        cps.HRIF_DisConnect(box_id)
        return 1

    ret = cps.HRIF_MoveJ(box_id, rbt_id, [0, 0, 0, 0, 0, 0], [dJ1 - 50.0, dJ2, dJ3, dJ4, dJ5, dJ6],
                         "TCP", "Base", 30.0, 60.0, 0.0, 1, 0, 0, 0, "motion_status_stop")
    print_ret(cps, "HRIF_MoveJ (stop demo)", ret)
    if ret != 0:
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 4. 根据运动 ID 暂停、继续和停止，并通过输出 IO 判断运动是否完成
    # ----------------------------------------------------------
    commandId = ""
    while commandId != "motion_status_interrupt":
        result = []
        ret = cps.HRIF_ReadCurWaypointID(box_id, rbt_id, result)
        if ret != 0:
            print_ret(cps, "HRIF_ReadCurWaypointID", ret)
            cps.HRIF_DisConnect(box_id)
            return 1
        commandId = str(result[0]) if result else ""
        time.sleep(0.1)
    print("[INFO] Current motion stage ID is: " + commandId)

    ret = cps.HRIF_GrpInterrupt(box_id, rbt_id)
    print_ret(cps, "HRIF_GrpInterrupt", ret)
    if ret != 0:
        cps.HRIF_DisConnect(box_id)
        return 1

    time.sleep(1.0)
    ret = cps.HRIF_GrpContinue(box_id, rbt_id)
    print_ret(cps, "HRIF_GrpContinue", ret)
    if ret != 0:
        cps.HRIF_DisConnect(box_id)
        return 1

    commandId = ""
    while commandId != "motion_status_stop":
        result = []
        ret = cps.HRIF_ReadCurWaypointID(box_id, rbt_id, result)
        if ret != 0:
            print_ret(cps, "HRIF_ReadCurWaypointID", ret)
            cps.HRIF_DisConnect(box_id)
            return 1
        commandId = str(result[0]) if result else ""
        time.sleep(0.1)
    print("[INFO] Current motion stage ID is: " + commandId)

    ret = cps.HRIF_GrpStop(box_id, rbt_id)
    print_ret(cps, "HRIF_GrpStop", ret)
    if ret != 0:
        cps.HRIF_DisConnect(box_id)
        return 1

    DO0 = 0
    while DO0 != 1:
        result = []
        ret = cps.HRIF_ReadBoxDO(box_id, 0, result)
        if ret != 0:
            print_ret(cps, "HRIF_ReadBoxDO", ret)
            cps.HRIF_DisConnect(box_id)
            return 1
        DO0 = int(result[0]) if result else 0
        time.sleep(0.1)

    print("[Info] Motion Done! Disconnecting")
    ret = cps.HRIF_DisConnect(box_id)
    print_ret(cps, "HRIF_DisConnect", ret)
    return 0 if ret == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
