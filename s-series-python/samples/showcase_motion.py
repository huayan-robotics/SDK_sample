#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Absolute motion example."""

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
    print("======== Waypoint Motion Example Start ========")
    cps = CPSClient()

    # ----------------------------------------------------------
    # 1. 连接机器人
    # ----------------------------------------------------------
    # 需根据机器人的实际 IP 进行修改
    ip = "192.168.0.10"
    port = 10003

    ret = cps.HRIF_Connect(box_id, ip, port)
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
    # 4. MoveJ 运动到安全关节位置，无过渡
    # ----------------------------------------------------------
    safe_joints = [0, -90, 90, -90, -90, 0]
    ret = cps.HRIF_MoveJ(box_id, rbt_id, [0, 0, 0, 0, 0, 0],
                         safe_joints, "TCP", "Base",
                         30.0, 60.0, 0.0, 1, 0, 0, 0, "absolute_MoveJ_home")
    print_ret(cps, "HRIF_MoveJ(Point A)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 5. 读取 TCP 当前位置，用于后续路点运动
    # 从安全点 A 开始绘制云朵形状：
    #                         M
    #             Waypoint ╱     ╲ WaypointEx
    #                    C         D
    #                  ╭             ╮
    #               MoveC          WayPoint2
    #                  ╰             ╯
    #                    B ◄─MoveL── A
    # ----------------------------------------------------------
    d_straight_length = 80.0 #直线长度
    d_arc_radius = 20.0  #圆弧半径
    d_x = 0.0
    d_y = 0.0
    d_z = 0.0
    d_rx = 0.0
    d_ry = 0.0
    d_rz = 0.0

    tcp_result = []
    ret = cps.HRIF_ReadActTcpPos(box_id, rbt_id, tcp_result)
    print_ret(cps, "HRIF_ReadActPos", ret)
    if ret != 0:
        cps.HRIF_DisConnect(box_id)
        return 1
    if len(tcp_result) < 6:
        print("[ERR] HRIF_ReadActTcpPos returned incomplete data", file=sys.stderr)
        cps.HRIF_DisConnect(box_id)
        return 1
    d_x, d_y, d_z, d_rx, d_ry, d_rz = map(float, tcp_result[:6])

    # ----------------------------------------------------------
    # 6. 从 A 点 MoveL 直线运动到点 B，无过渡
    # ----------------------------------------------------------
    d_x += d_straight_length
    ret = cps.HRIF_MoveL(box_id, rbt_id, [d_x, d_y, d_z, d_rx, d_ry, d_rz],
                         [0, 0, 0, 0, 0, 0], "TCP", "Base", 30.0, 60.0,
                         0.0, 0, 0, 0, "absolute_MoveL_approach")
    print_ret(cps, "HRIF_MoveL(Point B)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 7. MoveC：经过辅助点执行 B -> C 圆弧，无过渡
    # ----------------------------------------------------------
    ret = cps.HRIF_MoveC(box_id, rbt_id, [d_x, d_y, d_z, d_rx, d_ry, d_rz],
                         [d_x + d_arc_radius, d_y + d_arc_radius, d_z, d_rx, d_ry, d_rz], [d_x, d_y + 2 * d_arc_radius, d_z, d_rx, d_ry, d_rz],
                         0, 1, 0.0, 30.0, 60.0, 0.0, "TCP", "Base", "absolute_MoveC_BC")
    print_ret(cps, "HRIF_MoveC(B->C)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 8. WayPoint：C -> M，有过渡，过渡半径为 20.0 mm
    # ----------------------------------------------------------
    n_move_type = 1
    d_x -= d_straight_length / 2.0
    d_y += 2.0 * d_arc_radius + 50

    ret = cps.HRIF_WayPoint(box_id, rbt_id, n_move_type, [d_x, d_y, d_z, d_rx, d_ry, d_rz],
                            [0, 0, 0, 0, 0, 0], "TCP", "Base", 30.0, 60.0,
                            20.0, 0, 0, 0, 0, "WayPoint_C_M")
    print_ret(cps, "HRIF_WayPoint(C->M)", ret)
    if ret != 0:
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 9. WayPointEx：M -> D，有过渡，过渡半径为 20.0 mm
    # ----------------------------------------------------------
    d_x -= d_straight_length / 2.0
    d_y -= 50

    ret = cps.HRIF_WayPointEx(box_id, rbt_id, n_move_type, [d_x, d_y, d_z, d_rx, d_ry, d_rz],
                              [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0],
                              30.0, 60.0, 20.0, 0, 0, 0, 0, "WayPointEx_M_D")
    print_ret(cps, "HRIF_WayPointEx(M->D)", ret)
    if ret != 0:
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 10. WayPoint2：D -> A，绘制圆弧，无过渡
    # ----------------------------------------------------------
    n_move_type = 2
    ret = cps.HRIF_WayPoint2(box_id, rbt_id, n_move_type, [d_x, d_y - 2 * d_arc_radius, d_z, d_rx, d_ry, d_rz],
                             [d_x - d_arc_radius, d_y - d_arc_radius, d_z, d_rx, d_ry, d_rz], [0, 0, 0, 0, 0, 0],
                             "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPoint2_D_A")
    print_ret(cps, "HRIF_WayPoint2(D->A)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 11. 断开连接
    # ----------------------------------------------------------
    print("[OK] Absolute motion completed.")
    ret = cps.HRIF_DisConnect(box_id)
    print_ret(cps, "HRIF_DisConnect", ret)
    return 0 if ret == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
