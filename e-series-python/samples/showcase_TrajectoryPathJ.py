#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Create and run a PathJ curve from points relative to a safe joint pose."""

import math
import sys
import time

from CPS import CPSClient


# BoxID 和 RbtID 可以在此处进行修改
BOX_ID = 0
RBT_ID = 0
PATH_NAME = "sdk_relative_path_j"
JOINT_COUNT = 6
COMPUTE_TIMEOUT = 600.0
MOTION_TIMEOUT = 120.0


def print_ret(cps, command, ret):
    if ret == 0:
        print("[OK]  " + command)
        return True

    message = []
    lookup_ret = cps.HRIF_GetErrorCodeStr(BOX_ID, ret, message)
    suffix = ": {}".format(message[0]) if lookup_ret == 0 and message else ""
    print("[ERR] {} -> {}{}".format(command, ret, suffix), file=sys.stderr)
    return False


def wait_motion(cps, timeout):
    deadline = time.monotonic() + timeout
    done = [False]
    while time.monotonic() < deadline:
        ret = cps.HRIF_IsMotionDone(BOX_ID, RBT_ID, done)
        if ret != 0:
            print_ret(cps, "HRIF_IsMotionDone", ret)
            return False
        if done[0]:
            return True
        time.sleep(0.05)

    print("[ERR] Motion timed out", file=sys.stderr)
    return False


def wait_path_computed(cps):
    deadline = time.monotonic() + COMPUTE_TIMEOUT
    last_state = None
    while time.monotonic() < deadline:
        state = []
        ret = cps.HRIF_ReadPathState(BOX_ID, RBT_ID, PATH_NAME, state)
        if ret != 0:
            print_ret(cps, "HRIF_ReadPathState", ret)
            return False
        if len(state) < 2:
            print("[ERR] HRIF_ReadPathState returned incomplete data", file=sys.stderr)
            return False

        state_j = int(state[0])
        if state_j != last_state:
            print("[INFO] PathJ state={}".format(state_j))
            last_state = state_j
        if state_j == 3:
            return True
        if state_j == 5:
            print("[ERR] PathJ computation failed, error={}".format(state[1]), file=sys.stderr)
            return False
        time.sleep(0.2)

    print("[ERR] PathJ computation timed out", file=sys.stderr)
    return False


def build_relative_curve(base):
    points = []
    point_count = 13
    for index in range(point_count):
        phase = -0.5 * math.pi + 2.0 * math.pi * index / (point_count - 1)
        point = list(base)
        point[0] += 5.0 * math.sin(phase)
        point[1] += 3.0 * (1.0 - math.cos(phase))
        points.extend(point)
    return points


def main():
    print("=== PathJ Relative Curve Example ===")

    # ----------------------------------------------------------
    # 1. 连接机器人并确认处于 Standby(33)
    # ----------------------------------------------------------
    # 需根据机器人的实际 IP 进行修改
    host = "192.168.0.10"
    port = 10003

    cps = CPSClient()
    ret = cps.HRIF_Connect(BOX_ID, host, port)
    if not print_ret(cps, "HRIF_Connect", ret):
        return 1

    result = 0
    motion_started = False
    try:
        fsm = []
        ret = cps.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, fsm)
        if not print_ret(cps, "HRIF_ReadCurFSMFromCPS", ret):
            return 1
        if not fsm or int(fsm[0]) != 33:
            print("[ERR] Robot must be in Standby(33), FSM={}".format(fsm), file=sys.stderr)
            return 1

        # ----------------------------------------------------------
        # 2. 设置速度比为0.2，运动到安全基准位姿，并以实际到达位置作为相对点位基准
        # ----------------------------------------------------------
        ret = cps.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2)
        print_ret(cps, "HRIF_SetOverride", ret)
        if ret != 0:
            cps.HRIF_DisConnect(BOX_ID)
            return 1

        safe_joints = [0, 0, 90, 0, 90, 0]
        pose = [0.0] * 6  # IsUseJoint=1 时忽略笛卡尔位姿。
        ret = cps.HRIF_MoveJ(
            BOX_ID, RBT_ID, pose, safe_joints, "TCP", "Base",
            20.0, 40.0, 0.0, 1, 0, 0, 0, "path_j_home"
        )
        if not print_ret(cps, "HRIF_MoveJ (safe pose)", ret):
            return 1
        motion_started = True
        if not wait_motion(cps, MOTION_TIMEOUT):
            return 1

        joints = []
        ret = cps.HRIF_ReadActJointPos(BOX_ID, RBT_ID, joints)
        if not print_ret(cps, "HRIF_ReadActJointPos", ret):
            return 1
        if len(joints) < JOINT_COUNT:
            print("[ERR] HRIF_ReadActJointPos returned incomplete data", file=sys.stderr)
            return 1
        base = [float(value) for value in joints[:JOINT_COUNT]]
        points = build_relative_curve(base)
        first_point = points[:JOINT_COUNT]

        # ----------------------------------------------------------
        # 3. 创建 PathJ，推送 ACS 原始点位并等待轨迹计算完成
        # ----------------------------------------------------------
        ret = cps.HRIF_InitPath(
            BOX_ID, RBT_ID, 0, PATH_NAME, 0.15, 5.0,
            30.0, 60.0, 500.0, "Base", "TCP"
        )
        if not print_ret(cps, "HRIF_InitPath", ret):
            return 1

        ret = cps.HRIF_PushPathPoints(BOX_ID, RBT_ID, PATH_NAME, points)
        if not print_ret(cps, "HRIF_PushPathPoints", ret):
            return 1

        ret = cps.HRIF_EndPushPathPoints(BOX_ID, RBT_ID, PATH_NAME)
        if not print_ret(cps, "HRIF_EndPushPathPoints", ret):
            return 1
        if not wait_path_computed(cps):
            return 1

        # ----------------------------------------------------------
        # 4. 先运动到轨迹初始点 P0，再运行 PathJ
        # ----------------------------------------------------------
        ret = cps.HRIF_MoveJ(
            BOX_ID, RBT_ID, pose, first_point, "TCP", "Base",
            20.0, 40.0, 0.0, 1, 0, 0, 0, "path_j_start"
        )
        if not print_ret(cps, "HRIF_MoveJ (P0)", ret):
            return 1
        if not wait_motion(cps, MOTION_TIMEOUT):
            return 1

        ret = cps.HRIF_MovePathJ(BOX_ID, RBT_ID, PATH_NAME)
        if not print_ret(cps, "HRIF_MovePathJ", ret):
            return 1
        if not wait_motion(cps, MOTION_TIMEOUT):
            return 1
    finally:
        if motion_started:
            stop_ret = cps.HRIF_GrpStop(BOX_ID, RBT_ID)
            if not print_ret(cps, "HRIF_GrpStop", stop_ret):
                result = 1
        disconnect_ret = cps.HRIF_DisConnect(BOX_ID)
        if not print_ret(cps, "HRIF_DisConnect", disconnect_ret):
            result = 1

    return result


if __name__ == "__main__":
    sys.exit(main())
