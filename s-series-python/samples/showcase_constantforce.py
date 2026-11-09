#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Constant Force Control Example."""

import sys
import time

from CPS import CPSClient


# BoxID 和 RbtID 可以在此处修改。
box_id = 0
rbt_id = 0

FORCE_CONTROL_STATE_CLOSED = 0
FORCE_CONTROL_STATE_SEARCH = 1
FORCE_CONTROL_STATE_FOLLOW = 2
FORCE_CONTROL_STATE_FREEDRIVE = 3


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


def wait_blending_done(cps):
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        time.sleep(0.5)
        done = []
        ret = cps.HRIF_IsBlendingDone(box_id, rbt_id, done)
        if ret != 0:
            return False
        if done and done[0]:
            print("[ok] Motion Done ")
            return True
    print("[ERR] Motion timeout (60s).", file=sys.stderr)
    return False


def main():
    print("======== Constant Force Control Example ========")
    cps = CPSClient()

    # ----------------------------------------------------------
    # 1. 连接机器人
    # ----------------------------------------------------------
    host = "192.168.0.10"
    port = 10003
    ret = cps.HRIF_Connect(box_id, host, port)
    if not print_ret(cps, "HRIF_Connect", ret):
        return 1

    # ----------------------------------------------------------
    # 2. 确认机器人处于 Standby(33)，并确认力控关闭
    # ----------------------------------------------------------
    result = []
    ret = cps.HRIF_ReadCurFSMFromCPS(box_id, rbt_id, result)
    n_fsm = int(result[0]) if ret == 0 and result else -1
    if not print_ret(cps, "HRIF_ReadCurFSMFromCPS", ret) or n_fsm != 33:
        print("[ERR] Robot must be in Standby(33), FSM={}".format(n_fsm), file=sys.stderr)
        return fail(cps)

    result = []
    ret = cps.HRIF_ReadForceControlState(box_id, rbt_id, result)
    n_force_state = int(result[0]) if ret == 0 and result else -1
    if not print_ret(cps, "HRIF_ReadForceControlState", ret) or n_force_state != 0:
        print("[ERR] Expected force state 0, actual={}".format(n_force_state), file=sys.stderr)
        return fail(cps)

    # ----------------------------------------------------------
    # 3. 设置低速倍率，运动到现场确认的安全探寻起点
    # ----------------------------------------------------------
    ret = cps.HRIF_SetOverride(box_id, rbt_id, 0.2)
    if not print_ret(cps, "HRIF_SetOverride", ret):
        return fail(cps)

    # 请替换为初始探寻关节角度。
    safe_joints = [0, -90, 90, -90, -90, 0]
    ret = cps.HRIF_MoveJ(box_id, rbt_id, [0, 0, 0, 0, 0, 0], safe_joints,
                         "TCP", "Base", 10.0, 20.0, 0.0, 1, 0, 0, 0, "force_search_start")
    if not print_ret(cps, "HRIF_MoveJ(search start)", ret) or not wait_blending_done(cps):
        return fail(cps)

    # ----------------------------------------------------------
    # 4. 选择恒力模式
    # ----------------------------------------------------------
    n_strategy = 0

    ret = cps.HRIF_SetForceControlStrategy(box_id, rbt_id, n_strategy)
    if not print_ret(cps, "HRIF_SetForceControlStrategy", ret):
        return fail(cps)

    # ----------------------------------------------------------
    # 5. 仅开启 Z 平移自由度，设置目标力与异常力保护
    # ----------------------------------------------------------
    ret = cps.HRIF_SetControlFreedom(box_id, rbt_id, [0, 0, 1, 0, 0, 0])
    if not print_ret(cps, "HRIF_SetControlFreedom", ret):
        return fail(cps)

    d_target_fz = -10.0

    ret = cps.HRIF_SetForceControlGoal(box_id, rbt_id, [0, 0, d_target_fz, 0, 0, 0])
    if not print_ret(cps, "HRIF_SetForceControlGoal", ret):
        return fail(cps)

    ret = cps.HRIF_SetForceDataLimit(box_id, rbt_id,
                                     [500, 500, 500, 50, 50, 50], [300, 300, 300, 30, 30, 30])
    if not print_ret(cps, "HRIF_SetForceDataLimit", ret):
        return fail(cps)

    # ----------------------------------------------------------
    # 6. 设置探寻速度、探寻边界和跟随边界
    # ----------------------------------------------------------
    ret = cps.HRIF_SetMaxSearchVelocities(box_id, rbt_id, 5.0, 1.0)
    if not print_ret(cps, "HRIF_SetMaxSearchVelocities", ret):
        return fail(cps)

    ret = cps.HRIF_SetMaxSearchDistance(box_id, rbt_id, 100, 100, 100, 20, 20, 20)
    if not print_ret(cps, "HRIF_SetMaxSearchDistance", ret):
        return fail(cps)

    ret = cps.HRIF_SetSteadyContactDeviationRange(box_id, rbt_id, 300, 300, 300, 20, 20, 20,
                                                 -100, -100, -100, -20, -20, -20)
    if not print_ret(cps, "HRIF_SetSteadyContactDeviationRange", ret):
        return fail(cps)

    # ----------------------------------------------------------
    # 7. 设置动态响应参数
    # ----------------------------------------------------------
    # 设置力和力矩控制的质量/惯量参数和阻尼参数（此处设置为推荐的默认值）。
    ret = cps.HRIF_SetMassParams(box_id, rbt_id, [40.0, 40.0, 40.0, 10.0, 10.0, 10.0])
    if not print_ret(cps, "HRIF_SetMassParams", ret):
        return fail(cps)

    ret = cps.HRIF_SetDampParams(box_id, rbt_id, [800.0, 800.0, 800.0, 40.0, 40.0, 40.0])
    if not print_ret(cps, "HRIF_SetDampParams", ret):
        return fail(cps)

    # ----------------------------------------------------------
    # 8. 开启力控，等待探寻完成
    # ----------------------------------------------------------
    ret = cps.HRIF_SetForceControlState(box_id, rbt_id, 1)
    if not print_ret(cps, "HRIF_SetForceControlState(open)", ret):
        return fail(cps)

    deadline = time.monotonic() + 60.0
    state = -1
    timeout = True
    while time.monotonic() < deadline:
        time.sleep(0.5)
        result = []
        ret = cps.HRIF_ReadForceControlState(box_id, rbt_id, result)
        if ret != 0:
            break
        state = int(result[0]) if result else state
        if state == FORCE_CONTROL_STATE_FOLLOW:  # 代表力控探寻完成状态。
            print("[OK] Force state = {}".format(state))
            timeout = False
            break

    if ret != 0 or timeout:
        print("[ERR] Expected state !={}, actual={}".format(FORCE_CONTROL_STATE_FOLLOW, state), file=sys.stderr)
        ret = cps.HRIF_SetForceControlState(box_id, rbt_id, 0)
        print_ret(cps, "HRIF_SetForceControlState", ret)
        return fail(cps)
    
    tcp_result = []
    ret = cps.HRIF_ReadActTcpPos(box_id, rbt_id, tcp_result)
    print_ret(cps,"HRIF_ReadActTcpPos", ret)
    if ret != 0:
        cps.HRIF_DisConnect(box_id)
        return 1
    dX, dY, dZ, dRx, dRy, dRz = [float(x) for x in tcp_result[:6]]
    # ----------------------------------------------------------
    # 9. 在探寻完成状态下，沿 UCS X 正方向进行空间直线运动
    # ----------------------------------------------------------
    ret = cps.HRIF_MoveL(box_id, rbt_id, [dX, dY + 50.0 , dZ, dRx, dRy, dRz],
                         [0, 0, 0, 0, 0, 0], "TCP", "Base", 30.0, 60.0,
                         0.0, 0, 0, 0, "absolute_MoveL_approach")
    if not print_ret(cps, "HRIF_MoveL(Y + 50)", ret) or not wait_blending_done(cps):
        return fail(cps)
    # ----------------------------------------------------------
    # 10. 关闭力控，断开连接
    # ----------------------------------------------------------
    ret = cps.HRIF_SetForceControlState(box_id, rbt_id, 0)
    if not print_ret(cps, "HRIF_SetForceControlState(close)", ret):
        return fail(cps)

    print("[OK] Constant force control completed.")

    ret = cps.HRIF_DisConnect(box_id)
    print_ret(cps, "HRIF_DisConnect", ret)
    return 0 if ret == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
