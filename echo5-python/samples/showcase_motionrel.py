#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""relative motion example."""

import sys
import time

from CPS import CPSClient


# BoxID 和 RbtID 可以在此处进行修改
BOX_ID = 0
RBT_ID = 0


def print_ret(cps, cmd, ret):
    if ret == 0:
        print("[OK]  " + cmd)
        return
    err_msg = []
    lookup_ret = cps.HRIF_GetErrorCodeStr(BOX_ID, ret, err_msg)
    suffix = ": {}".format(err_msg[0]) if lookup_ret == 0 and err_msg else ""
    print("[ERR] {} -> {}{}".format(cmd, ret, suffix), file=sys.stderr)


def wait_motion_done(cps):
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        time.sleep(0.5)
        done = []
        ret = cps.HRIF_IsMotionDone(BOX_ID, RBT_ID, done)
        if ret != 0:
            print_ret(cps, "HRIF_IsMotionDone", ret)
            return False
        if done and done[0]:
            return True
    print("[ERR] Motion timeout (60s).", file=sys.stderr)
    return False


def main():
    print("======== Relative motion example started. ========")
    cps = CPSClient()

    # ----------------------------------------------------------
    # 1. 连接机器人
    # ----------------------------------------------------------
    # 需根据机器人的实际 IP 进行修改
    host = "192.168.0.10"
    port = 10003
    ret = cps.HRIF_Connect(BOX_ID, host, port)
    print_ret(cps, "HRIF_Connect", ret)
    if ret != 0:
        return 1

    # ----------------------------------------------------------
    # 2. 确认机器人处于 Standby(33)
    # ----------------------------------------------------------
    fsm_result = []
    ret = cps.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, fsm_result)
    n_fsm = int(fsm_result[0]) if fsm_result else 0
    print_ret(cps, "HRIF_ReadCurFSM (FSM={})".format(n_fsm), ret)
    if ret != 0 or n_fsm != 33:
        print("[ERR] Robot must be in Standby(33), FSM={}".format(n_fsm), file=sys.stderr)
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 3. 设置全局速度比为 0.2
    # ----------------------------------------------------------
    ret = cps.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2)
    print_ret(cps, "HRIF_SetOverride", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 4. MoveJ 运动到安全起点
    # ----------------------------------------------------------
    safe_joints = [90, 45, 0, -80, 0, 35, 0]
    zero_pose = [0.0] * 6
    ret = cps.HRIF_MoveJ_nJ(BOX_ID, RBT_ID, zero_pose, safe_joints, "TCP", "Base", 30.0, 60.0,
                            0.0, 1, 0, 0, 0, "Jog_start")
    print_ret(cps, "HRIF_MoveJ_nJ", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 5. 单轴关节相对运动：J2 轴正向旋转 10°
    # ----------------------------------------------------------
    axis_id = 1
    direction = 1
    distance = 10.0
    ret = cps.HRIF_MoveRelJ(BOX_ID, RBT_ID, axis_id, direction, distance)
    print_ret(cps, "HRIF_MoveRelJ(J2 +10 deg)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(BOX_ID)
        return 1

   # ----------------------------------------------------------
    # 6. MoveRelL：沿当前 UCS X 反方向移动 30 mm
    # ----------------------------------------------------------
    tool_motion = 0
    axis_id = 0
    direction = 0
    distance = 30.0
    ret = cps.HRIF_MoveRelL(BOX_ID, RBT_ID, axis_id, direction, distance, tool_motion)
    print_ret(cps, "HRIF_MoveRelL(X -30 mm)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 7. WayPointRel：叠加值模式，X正向、Y反向移动 30 mm
    # ----------------------------------------------------------
    move_type = 1
    point_list = 0
    relative_type = 1
    point_joints = [0.0] * 7
    axis_mask = [1, 1, 0, 0, 0, 0]
    targets = [30.0, -30.0, 0.0, 0.0, 0.0, 0.0]
    ret = cps.HRIF_WayPointRel_nJ(BOX_ID, RBT_ID, move_type, point_list, zero_pose, point_joints, relative_type,
                                  axis_mask, targets, "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPointRel_XY1")
    print_ret(cps, "HRIF_WayPointRel_nJ(X +30 mm, Y -30 mm)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 8. WayPointRel：叠加值模式，X 反向、Y 正向移动 30 mm
    # ----------------------------------------------------------
    targets = [-30.0, 30.0, 0.0, 0.0, 0.0, 0.0]
    ret = cps.HRIF_WayPointRel_nJ(BOX_ID, RBT_ID, move_type, point_list, zero_pose, point_joints, relative_type,
                                  axis_mask, targets, "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPointRel_XY2")
    print_ret(cps, "HRIF_WayPointRel_nJ(X -30 mm, Y +30 mm)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 9. WayPointRel：叠加值模式，J1、J2 同时旋转 10°
    # ----------------------------------------------------------
    move_type = 0
    axis_mask = [1, 1, 0, 0, 0, 0, 0]
    targets = [10.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ret = cps.HRIF_WayPointRel_nJ(BOX_ID, RBT_ID, move_type, point_list, zero_pose, point_joints, relative_type,
                                  axis_mask, targets, "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPointRel_J1_J2")
    print_ret(cps, "HRIF_WayPointRel_nJ(J1 +10 deg, J2 +10 deg)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 10. 断开连接
    # ----------------------------------------------------------
    print("[OK] Relative motion completed.")
    ret = cps.HRIF_DisConnect(BOX_ID)
    print_ret(cps, "HRIF_DisConnect", ret)
    return 0 if ret == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
