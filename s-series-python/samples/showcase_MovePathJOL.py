#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import sys
import time

from CPS import CPSClient

# BoxID 和 RbtID 可以在此处进行修改
BOX_ID = 0
RBT_ID = 0
BOX_DO_BIT = 2
BOX_DO_MASK = 1 << BOX_DO_BIT


def print_ret(cps, cmd, ret):
    if ret == 0:
        print("[OK]  " + cmd)
    else:
        err_msg = []
        lookup_ret = cps.HRIF_GetErrorCodeStr(BOX_ID, ret, err_msg)
        suffix = ": {}".format(err_msg[0]) if lookup_ret == 0 and err_msg else ""
        print("[ERR] {} -> {}{}".format(cmd, ret, suffix))


def main():
    print("=== MovePathJOL Online Path Example ===")

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
    # 2. 确认机器人处于 Standby(33) 状态
    # ----------------------------------------------------------
    fsm_result = []
    ret = cps.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, fsm_result)
    n_fsm = int(fsm_result[0]) if fsm_result else 0
    print_ret(cps, "HRIF_ReadCurFSM (FSM={})".format(n_fsm), ret)
    if ret != 0 or n_fsm != 33:
        print("[ERR] Robot must be in Standby(33), FSM={}".format(n_fsm))
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 3. 设置速度比，并运动到安全位置
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
        BOX_ID, RBT_ID, pose, safe_joints, "TCP", "Base",
        30.0, 60.0, 0.0, 1, 0, 0, 0, "MovePathJOL_home")
    print_ret(cps, "HRIF_MoveJ", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    done = [False]
    motion_deadline = time.monotonic() + 60.0
    while not done[0] and time.monotonic() < motion_deadline:
        time.sleep(0.1)
        ret = cps.HRIF_IsMotionDone(BOX_ID, RBT_ID, done)
        if ret != 0:
            break
    if ret != 0 or not done[0]:
        if ret != 0:
            print_ret(cps, "HRIF_IsMotionDone", ret)
        else:
            print("[ERR] MoveJ timed out")
        cps.HRIF_DisConnect(BOX_ID)
        return 1
    print("[INFO] Safe position reached")

    # ----------------------------------------------------------
    # 4. 准备并一次性下发 10 个点
    # ----------------------------------------------------------
    point_count = 10
    raw_acs_points = []
    n_is_set_io = [0] * point_count
    n_end_do_mask = [0] * point_count
    n_end_do_val = [0] * point_count
    n_box_do_mask = [0] * point_count
    n_box_do_val = [0] * point_count
    n_box_co_mask = [0] * point_count
    n_box_co_val = [0] * point_count
    n_box_aoch0_mask = [0] * point_count
    n_box_aoch0_mode = [0] * point_count
    n_box_aoch1_mask = [0] * point_count
    n_box_aoch1_mode = [0] * point_count
    box_aoch0_val = [0.0] * point_count
    box_aoch1_val = [0.0] * point_count

    for i in range(point_count):
        angle = 2.0 * math.pi * i / (point_count - 1)
        point = list(safe_joints)
        point[0] += 1.5 * math.sin(angle)
        point[1] += 0.8 * (1.0 - math.cos(angle))
        raw_acs_points.extend(point)

    n_is_set_io[4] = 1
    n_box_do_mask[4] = BOX_DO_MASK
    n_box_do_val[4] = BOX_DO_MASK
    n_is_set_io[9] = 1
    n_box_do_mask[9] = BOX_DO_MASK
    n_box_do_val[9] = 0

    ret = cps.HRIF_MovePathJOL(
        BOX_ID, RBT_ID, 20, 20, 20,
        raw_acs_points, n_is_set_io, n_end_do_mask, n_end_do_val,
        n_box_do_mask, n_box_do_val, n_box_co_mask, n_box_co_val,
        n_box_aoch0_mask, n_box_aoch0_mode,
        n_box_aoch1_mask, n_box_aoch1_mode,
        box_aoch0_val, box_aoch1_val)
    print_ret(cps, "HRIF_MovePathJOL (10 points)", ret)
    if ret != 0:
        cps.HRIF_GrpStop(BOX_ID, RBT_ID)
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 5. 第 5 个点执行完成后读取 DO2 状态
    # ----------------------------------------------------------
    path_index = [0, 0]
    path_deadline = time.monotonic() + 60.0
    while time.monotonic() < path_deadline:
        time.sleep(0.1)
        ret = cps.HRIF_GetMovePathJOLIndex(RBT_ID, BOX_ID, path_index)
        if ret != 0 or int(path_index[0]) >= 6:
            break
    if ret != 0 or int(path_index[0]) < 6:
        if ret != 0:
            print_ret(cps, "HRIF_GetMovePathJOLIndex", ret)
        else:
            print("[ERR] Wait for MovePathJOL point 5 completion timed out")
        cps.HRIF_GrpStop(BOX_ID, RBT_ID)
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    do_result = []
    ret = cps.HRIF_ReadBoxDO(BOX_ID, BOX_DO_BIT, do_result)
    do_value = int(do_result[0]) if do_result else 0
    print_ret(cps, "HRIF_ReadBoxDO (point 5, DO2={})".format(do_value), ret)
    if ret != 0:
        cps.HRIF_GrpStop(BOX_ID, RBT_ID)
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 6. 等待第 10 个点完成，并再次读取 DO2 状态
    # ----------------------------------------------------------
    path_deadline = time.monotonic() + 60.0
    while time.monotonic() < path_deadline:
        time.sleep(0.1)
        ret = cps.HRIF_GetMovePathJOLIndex(RBT_ID, BOX_ID, path_index)
        if ret != 0 or int(path_index[0]) >= 10:
            break
    if ret != 0 or int(path_index[0]) < 10:
        if ret != 0:
            print_ret(cps, "HRIF_GetMovePathJOLIndex", ret)
        else:
            print("[ERR] Wait for MovePathJOL point 10 timed out")
        cps.HRIF_GrpStop(BOX_ID, RBT_ID)
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    done = [False]
    motion_deadline = time.monotonic() + 60.0
    while not done[0] and time.monotonic() < motion_deadline:
        time.sleep(0.1)
        ret = cps.HRIF_IsMotionDone(BOX_ID, RBT_ID, done)
        if ret != 0:
            break
    if ret != 0 or not done[0]:
        if ret != 0:
            print_ret(cps, "HRIF_IsMotionDone", ret)
        else:
            print("[ERR] MovePathJOL completion timed out")
        cps.HRIF_GrpStop(BOX_ID, RBT_ID)
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    do_result = []
    ret = cps.HRIF_ReadBoxDO(BOX_ID, BOX_DO_BIT, do_result)
    do_value = int(do_result[0]) if do_result else 0
    print_ret(cps, "HRIF_ReadBoxDO (completed, DO2={})".format(do_value), ret)

    # ----------------------------------------------------------
    # 7. 停止运动并断开连接
    # ----------------------------------------------------------
    print("[INFO] MovePathJOL completed; disconnecting")
    stop_ret = cps.HRIF_GrpStop(BOX_ID, RBT_ID)
    print_ret(cps, "HRIF_GrpStop", stop_ret)
    disconnect_ret = cps.HRIF_DisConnect(BOX_ID)
    print_ret(cps, "HRIF_DisConnect", disconnect_ret)
    return 0 if ret == 0 and stop_ret == 0 and disconnect_ret == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
