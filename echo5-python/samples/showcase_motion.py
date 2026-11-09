#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""absolute motion example."""

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
    print("======== Waypoint Motion Example Start ========")
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
    # 4. MoveJ 运动到安全关节位置，无过渡
    # ----------------------------------------------------------
    safe_joints = [90, 45, 0, -80, 0, 35, 0]
    zero_pose = [0.0] * 6
    ret = cps.HRIF_MoveJ_nJ(BOX_ID, RBT_ID, zero_pose, safe_joints, "TCP", "Base", 30.0, 60.0,
                            0.0, 1, 0, 0, 0, "absolute_MoveJ_home")
    print_ret(cps, "HRIF_MoveJ_nJ(Point A)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(BOX_ID)
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
    straight_length = 30.0
    arc_radius = 10.0
    tcp_result = []
    ret = cps.HRIF_ReadActTcpPos(BOX_ID, RBT_ID, tcp_result)
    print_ret(cps, "HRIF_ReadActPos", ret)
    if ret != 0 or len(tcp_result) < 6:
        if ret == 0:
            print("[ERR] HRIF_ReadActTcpPos returned incomplete data", file=sys.stderr)
        cps.HRIF_DisConnect(BOX_ID)
        return 1
    x, y, z, rx, ry, rz = map(float, tcp_result[:6])

    # ----------------------------------------------------------
    # 6. 从 A 点 MoveL 直线运动到点 B，无过渡
    # ----------------------------------------------------------
    z -= straight_length
    pose = [x, y, z, rx, ry, rz]
    ret = cps.HRIF_MoveL_nJ(BOX_ID, RBT_ID, pose, safe_joints, "TCP", "Base", 30.0, 60.0,
                            0.0, 0, 0, 0, "absolute_MoveL_approach")
    print_ret(cps, "HRIF_MoveL_nJ(Point B)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 7. MoveC：经过辅助点执行 B -> C 圆弧，无过渡
    # ----------------------------------------------------------
    aux_pose = [x + arc_radius, y, z - arc_radius, rx, ry, rz]
    end_pose = [x + 2.0 * arc_radius, y, z, rx, ry, rz]
    ret = cps.HRIF_WayPoint2_nJ(BOX_ID, RBT_ID, 2, end_pose, aux_pose, safe_joints, "TCP", "Base",
                                30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPoint2_B_C")
    print_ret(cps, "HRIF_WayPoint2_nJ(B->C)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 8. WayPoint：C -> M，有过渡，过渡半径为 20.0 mm
    # ----------------------------------------------------------
    move_type = 1
    z += straight_length / 4.0
    x += 2.0 * arc_radius + 30.0
    pose = [x, y, z, rx, ry, rz]
    ret = cps.HRIF_WayPoint_nJ(BOX_ID, RBT_ID, move_type, pose, safe_joints, "TCP", "Base", 50.0,
                               50.0, 20.0, 0, 0, 0, 0, "WayPoint_C_M")
    print_ret(cps, "HRIF_WayPoint_nJ(C->M)", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 9. WayPointEx：M -> D，有过渡，过渡半径为 20.0 mm
    # ----------------------------------------------------------
    z += straight_length / 4.0
    x -= 30.0
    pose = [x, y, z, rx, ry, rz]
    ret = cps.HRIF_WayPointEx_nJ(BOX_ID, RBT_ID, move_type, pose, safe_joints, zero_pose, zero_pose,
                                 30.0, 60.0, 20.0, 0, 0, 0, 0, "WayPointEx_M_D")
    print_ret(cps, "HRIF_WayPointEx_nJ(M->D)", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 10. WayPoint2：D -> A，绘制圆弧，无过渡
    # ----------------------------------------------------------
    move_type = 2
    end_pose = [x - 2.0 * arc_radius, y, z, rx, ry, rz]
    aux_pose = [x - arc_radius / 2.0, y, z + arc_radius / 2.0, rx, ry, rz]
    ret = cps.HRIF_WayPoint2_nJ(BOX_ID, RBT_ID, move_type, end_pose, aux_pose, safe_joints, "TCP", "Base",
                                30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPoint2_D_A")
    print_ret(cps, "HRIF_WayPoint2_nJ(D->A)", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    print("[OK] Absolute motion completed.")

    # ----------------------------------------------------------
    # 11. 断开连接
    # ----------------------------------------------------------
    ret = cps.HRIF_DisConnect(BOX_ID)
    print_ret(cps, "HRIF_DisConnect", ret)
    return 0 if ret == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
