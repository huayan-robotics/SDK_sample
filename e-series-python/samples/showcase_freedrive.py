#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Force Free Drive Example."""

import sys
import time

from CPS import CPSClient


# BoxID 和 RbtID 可以在此处修改。
box_id = 0
rbt_id = 0


def print_ret(cps, cmd, ret):
    if ret == 0:
        print("[OK]  " + cmd)
        return True
    err_msg = []
    lookup_ret = cps.HRIF_GetErrorCodeStr(box_id, ret, err_msg)
    suffix = ": " + str(err_msg[0]) if lookup_ret == 0 and err_msg else ""
    print("[ERR] {} -> {}{}".format(cmd, ret, suffix), file=sys.stderr)
    return False


def fail(cps):
    ret = cps.HRIF_DisConnect(box_id)
    print_ret(cps, "HRIF_DisConnect", ret)
    return 1


def main():
    print("======== Force Free Drive Example ========")
    cps = CPSClient()
    # ----------------------------------------------------------
    # 1. 连接机器人
    # ----------------------------------------------------------
    ip = "192.168.0.10"
    port = 10003
    ret = cps.HRIF_Connect(box_id, ip, port)
    if not print_ret(cps, "HRIF_Connect", ret):
        return 1

    # ----------------------------------------------------------
    # 2. 确认机器人处于 Standby(33)
    # ----------------------------------------------------------
    result = []
    ret = cps.HRIF_ReadCurFSMFromCPS(box_id, rbt_id, result)
    n_fsm = int(result[0]) if ret == 0 and result else -1
    if not print_ret(cps, "HRIF_ReadCurFSMFromCPS", ret) or n_fsm != 33:
        print("[ERR] Robot must be in Standby(33), FSM={}".format(n_fsm), file=sys.stderr)
        return fail(cps)

    # ----------------------------------------------------------
    # 3. 仅开启 Z 平移自由度
    # ----------------------------------------------------------
    ret = cps.HRIF_SetFreeDriveMotionFreedom(box_id, [0, 0, 1, 0, 0, 0], rbt_id)
    if not print_ret(cps, "HRIF_SetFreeDriveMotionFreedom", ret):
        return fail(cps)

    # ----------------------------------------------------------
    # 4. 设置启动阈值、速度和柔顺度
    # ----------------------------------------------------------
    d_force_threshold = 5.0
    d_torque_threshold = 0.4

    ret = cps.HRIF_SetFTWrenchThresholds(box_id, rbt_id, d_force_threshold, d_torque_threshold)
    if not print_ret(cps, "HRIF_SetFTWrenchThresholds", ret):
        return fail(cps)

    n_speed_mode = 1

    ret = cps.HRIF_SetFTFreeDriveSpeedMode(box_id, rbt_id, n_speed_mode)
    if not print_ret(cps, "HRIF_SetFTFreeDriveSpeedMode", ret):
        return fail(cps)

    d_max_linear_velocity = 10.0
    d_max_angular_velocity = 5.0

    ret = cps.HRIF_SetMaxFreeDriveVel(box_id, rbt_id, d_max_linear_velocity, d_max_angular_velocity)
    if not print_ret(cps, "HRIF_SetMaxFreeDriveVel", ret):
        return fail(cps)

    d_linear = 50.0
    d_angular = 50.0

    ret = cps.HRIF_SetFTFreeFactor(box_id, d_linear, d_angular, rbt_id)
    if not print_ret(cps, "HRIF_SetFTFreeFactor", ret):
        return fail(cps)

    # ----------------------------------------------------------
    # 5. 开启自由驱动
    # ----------------------------------------------------------
    ret = cps.HRIF_SetForceFreeDriveMode(box_id, rbt_id, 1)
    if not print_ret(cps, "HRIF_SetForceFreeDriveMode(open)", ret):
        return fail(cps)

    time.sleep(1.0)

    result = []
    ret = cps.HRIF_ReadForceControlState(box_id, rbt_id, result)
    n_force_state = int(result[0]) if ret == 0 and result else -1
    if not print_ret(cps, "HRIF_ReadForceControlState", ret) or n_force_state != 3:
        print("[ERR] Expected force state 3, actual={}".format(n_force_state), file=sys.stderr)
        return fail(cps)

    # ----------------------------------------------------------
    # 6. 保持自由驱动 5 秒
    # ----------------------------------------------------------
    n_duration_seconds = 5
    print("[INFO] Free drive enabled for {} seconds.".format(n_duration_seconds))
    time.sleep(n_duration_seconds)

    # ----------------------------------------------------------
    # 7. 关闭自由驱动并断开连接
    # ----------------------------------------------------------
    ret = cps.HRIF_SetForceFreeDriveMode(box_id, rbt_id, 0)
    if not print_ret(cps, "HRIF_SetForceFreeDriveMode(close)", ret):
        return fail(cps)

    ret = cps.HRIF_DisConnect(box_id)
    print_ret(cps, "HRIF_DisConnect", ret)
    return 0 if ret == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
