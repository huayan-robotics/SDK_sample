#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Create and run a PathL curve from points relative to a safe TCP pose."""

import math
import sys
import time

from CPS import CPSClient


# BoxID 和 RbtID 可以在此处进行修改
BOX_ID = 0
RBT_ID = 0
PATH_NAME = "sdk_relative_path_l"
POSE_SIZE = 6
COMPUTE_TIMEOUT = 120.0
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
        if len(state) < 4:
            print("[ERR] HRIF_ReadPathState returned incomplete data", file=sys.stderr)
            return False

        state_l = int(state[2])
        if state_l != last_state:
            print("[INFO] PathL state={}".format(state_l))
            last_state = state_l
        if state_l == 3:
            return True
        if state_l == 5:
            print("[ERR] PathL computation failed, error={}".format(state[3]), file=sys.stderr)
            return False
        time.sleep(0.2)

    print("[ERR] PathL computation timed out", file=sys.stderr)
    return False


def build_relative_curve(base):
    points = []
    point_count = 13
    for index in range(point_count):
        phase = -0.5 * math.pi + 2.0 * math.pi * index / (point_count - 1)
        point = list(base)
        point[1] += 20.0 * math.sin(phase)
        point[2] += 10.0 * (1.0 - math.cos(phase))
        points.extend(point)
    return points


def main():
    print("=== PathL Relative Curve Example ===")

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
        # 2. 设置速度比为0.2，运动到安全基准位姿，并读取实际 TCP 作为相对点位基准
        # ----------------------------------------------------------
        ret = cps.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2)
        print_ret(cps, "HRIF_SetOverride", ret)
        if ret != 0:
            cps.HRIF_DisConnect(BOX_ID)
            return 1

        safe_joints = [0, 0, 90, 0, 90, 0]
        ignored_pose = [0.0] * 6
        ret = cps.HRIF_MoveJ(
            BOX_ID, RBT_ID, ignored_pose, safe_joints, "TCP", "Base",
            20.0, 40.0, 0.0, 1, 0, 0, 0, "path_l_home"
        )
        if not print_ret(cps, "HRIF_MoveJ (safe pose)", ret):
            return 1
        motion_started = True
        if not wait_motion(cps, MOTION_TIMEOUT):
            return 1

        pose = []
        ret = cps.HRIF_ReadActTcpPos(BOX_ID, RBT_ID, pose)
        if not print_ret(cps, "HRIF_ReadActTcpPos", ret):
            return 1
        if len(pose) < POSE_SIZE:
            print("[ERR] HRIF_ReadActTcpPos returned incomplete data", file=sys.stderr)
            return 1
        base = [float(value) for value in pose[:POSE_SIZE]]
        points = build_relative_curve(base)
        first_point = points[:POSE_SIZE]

        # ----------------------------------------------------------
        # 3. 创建 PathL，推送 PCS 原始点位并等待轨迹计算完成
        # ----------------------------------------------------------
        ret = cps.HRIF_InitPath(
            BOX_ID, RBT_ID, 1, PATH_NAME, 0.15, 5.0,
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
        # 4. 以相同 UCS/TCP 运动到轨迹初始点 P0，再运行 PathL
        # ----------------------------------------------------------
        reference_joints = [0.0] * 6  # MoveL 不使用参考关节时传零。
        ret = cps.HRIF_MoveL(
            BOX_ID, RBT_ID, first_point, reference_joints, "TCP", "Base",
            20.0, 40.0, 0.0, 0, 0, 0, "path_l_start"
        )
        if not print_ret(cps, "HRIF_MoveL (P0)", ret):
            return 1
        if not wait_motion(cps, MOTION_TIMEOUT):
            return 1

        ret = cps.HRIF_MovePathL(BOX_ID, RBT_ID, PATH_NAME)
        if not print_ret(cps, "HRIF_MovePathL", ret):
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
