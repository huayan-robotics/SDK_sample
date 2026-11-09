#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Relative motion example."""

import sys
import time

from CPS import CPSClient


# BoxID 和 RbtID 可以在此处进行修改
box_id = 0
rbt_id = 0


def print_ret(cps, cmd, ret):
    if ret == 0:
        print("[OK]  " + cmd)
    else:
        err_msg = []
        lookup_ret = cps.HRIF_GetErrorCodeStr(box_id, ret, err_msg)
        suffix = ": {}".format(err_msg[0]) if lookup_ret == 0 and err_msg else ""
        print("[ERR] {} -> {}{}".format(cmd, ret, suffix), file=sys.stderr)


def wait_motion_done(cps):
    deadline = time.monotonic() + 60.0

    while time.monotonic() < deadline:
        time.sleep(0.5)
        done = []
        ret = cps.HRIF_IsMotionDone(box_id, rbt_id, done)
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

    ret = cps.HRIF_Connect(box_id, host, port)
    print_ret(cps, "HRIF_Connect", ret)
    if ret != 0:
        return 1

    # ----------------------------------------------------------
    # 2. 确认机器人处于 Standby(33)
    # ----------------------------------------------------------
    fsm_result = []
    ret = cps.HRIF_ReadCurFSMFromCPS(box_id, rbt_id, fsm_result)
    n_fsm = int(fsm_result[0]) if fsm_result else 0
    print_ret(cps, "HRIF_ReadCurFSM (FSM={})".format(n_fsm), ret)
    if ret != 0 or n_fsm != 33:
        print("[ERR] Robot must be in Standby(33), FSM={}".format(n_fsm), file=sys.stderr)
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 3. 设置全局速度比为 0.2
    # ----------------------------------------------------------
    ret = cps.HRIF_SetOverride(box_id, rbt_id, 0.2)
    print_ret(cps, "HRIF_SetOverride", ret)
    if ret != 0:
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 4. MoveJ 运动到安全起点
    # ----------------------------------------------------------
    safe_joints = [0, 0, 90, 0, 90, 0]
    ret = cps.HRIF_MoveJ(box_id, rbt_id, [0, 0, 0, 0, 0, 0],
                         safe_joints, "TCP", "Base",
                         30.0, 60.0, 0.0, 1, 0, 0, 0, "Jog_start")
    print_ret(cps, "HRIF_MoveJ", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 5. 单轴关节相对运动：J1 轴正向旋转 10°
    # ----------------------------------------------------------
    n_axis_id = 0       # 0~5 对应 J1~J6
    n_direction = 1    # 1：正方向；0：负方向
    d_distance = 10.0  # 相对距离

    ret = cps.HRIF_MoveRelJ(box_id, rbt_id, n_axis_id, n_direction, d_distance)
    print_ret(cps, "HRIF_MoveRelJ(J1 +10 deg)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 6. MoveRelL：沿当前 UCS X 正方向移动 30 mm
    # ----------------------------------------------------------
    n_tool_motion = 0  # 0：相对 UCS；1：相对 TCP
    n_axis_id = 0      # 0~5 对应 X、Y、Z、Rx、Ry、Rz
    n_direction = 1
    d_distance = 30.0

    ret = cps.HRIF_MoveRelL(
        box_id, rbt_id, n_axis_id, n_direction, d_distance, n_tool_motion
    )
    print_ret(cps, "HRIF_MoveRelL(X +30 mm)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 7. WayPointRel：叠加值模式，X、Y 同时正向移动 30 mm
    # ----------------------------------------------------------
    n_type = 1          # 1：直线运动
    n_point_list = 0    # 不使用点位列表
    n_rel_move_type = 1  # 使用叠加相对值

    ret = cps.HRIF_WayPointRel(box_id, rbt_id, n_type, n_point_list, [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0],
                               n_rel_move_type, [1, 1, 0, 0, 0, 0], [30.0, 30.0, 0, 0, 0, 0],
                               "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPointRel_XY_with_radius")
    print_ret(cps, "HRIF_WayPointRel(X +30 mm, Y +30 mm)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 8. WayPointRel：叠加值模式，X 反向、Y 正向移动 30 mm
    # ----------------------------------------------------------
    ret = cps.HRIF_WayPointRel(box_id, rbt_id, n_type, n_point_list, [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0],
                               n_rel_move_type, [1, 1, 0, 0, 0, 0], [-30.0, 30.0, 0, 0, 0, 0],
                               "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPointRel_XY_without_radius")
    print_ret(cps, "HRIF_WayPointRel(X -30 mm, Y +30 mm)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 9. WayPointRel：叠加值模式，J1、J4 同时旋转 10°
    # ----------------------------------------------------------
    n_type = 0  # 关节运动
    ret = cps.HRIF_WayPointRel(box_id, rbt_id, n_type, n_point_list, [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0],
                               n_rel_move_type, [1, 0, 0, 1, 0, 0], [10.0, 0, 0, 10.0, 0, 0],
                               "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPointRel_J1_J2")
    print_ret(cps, "HRIF_WayPointRel(J1 +10 deg, J4 +10 deg)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 10. 断开连接
    # ----------------------------------------------------------
    print("[OK] Relative motion completed.")
    ret = cps.HRIF_DisConnect(box_id)
    print_ret(cps, "HRIF_DisConnect", ret)
    return 0 if ret == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
