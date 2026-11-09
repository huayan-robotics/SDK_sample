#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""jogging example."""

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
    print("======== Jogging example started. ========")
    cps = CPSClient()

    # ----------------------------------------------------------
    # 1. 连接机器人，需根据机器人的实际 IP 进行修改
    # ----------------------------------------------------------
    host = "192.168.0.10"
    port = 10003
    ret = cps.HRIF_Connect(BOX_ID, host, port)
    print_ret(cps, "HRIF_Connect", ret)
    if ret != 0:
        return 1

    # ----------------------------------------------------------
    # 2. 确认机器人处于 Standby(33) 就绪状态
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
    safe_joints = [-90, -42.5, 0, 72.5, 0, -65, 0]
    zero_pose = [0.0] * 6
    ret = cps.HRIF_MoveJ_nJ(BOX_ID, RBT_ID, zero_pose, safe_joints, "TCP", "Base", 30.0, 60.0,
                            0.0, 1, 0, 0, 0, "Jog_start")
    print_ret(cps, "HRIF_MoveJ_nJ", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 5. 进行关节短点动，J3 轴反向运动 10°
    # ----------------------------------------------------------
    axis_id = 2
    direction = 0
    for _ in range(5):
        ret = cps.HRIF_ShortJogJ(BOX_ID, RBT_ID, axis_id, direction)
        print_ret(cps, "HRIF_ShortJogJ", ret)
        if ret != 0 or not wait_motion_done(cps):
            cps.HRIF_DisConnect(BOX_ID)
            return 1

    # ----------------------------------------------------------
    # 6. 进行空间短点动，X 轴反向运动 20 mm
    # ----------------------------------------------------------
    axis_id = 0
    for _ in range(10):
        ret = cps.HRIF_ShortJogL(BOX_ID, RBT_ID, axis_id, direction)
        print_ret(cps, "HRIF_ShortJogL", ret)
        if ret != 0 or not wait_motion_done(cps):
            cps.HRIF_DisConnect(BOX_ID)
            return 1

    # ----------------------------------------------------------
    # 7. 进行空间长点动，X 反向运动；停止发送事件后等待停止
    # ----------------------------------------------------------
    state = 1
    ret = cps.HRIF_LongJogL(BOX_ID, RBT_ID, axis_id, direction, state)
    print_ret(cps, "HRIF_LongJogL", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    for index in range(5):
        time.sleep(0.1)
        ret = cps.HRIF_LongMoveEvent(BOX_ID, RBT_ID)
        print_ret(cps, "HRIF_LongMoveEvent" if index < 4 else "HRIF_LongMoveEvent(stop)", ret)
        if ret != 0:
            cps.HRIF_DisConnect(BOX_ID)
            return 1
    if not wait_motion_done(cps):
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 8. 进行关节长点动，J4 轴正向运动；主动发送 n_state=0 停止
    # ----------------------------------------------------------
    axis_id = 3
    direction = 1
    ret = cps.HRIF_LongJogJ(BOX_ID, RBT_ID, axis_id, direction, state)
    print_ret(cps, "HRIF_LongJogJ", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    start = time.monotonic()
    while True:
        ret = cps.HRIF_LongMoveEvent(BOX_ID, RBT_ID)
        print_ret(cps, "HRIF_LongMoveEvent", ret)
        if ret != 0:
            cps.HRIF_DisConnect(BOX_ID)
            return 1
        time.sleep(0.1)
        if time.monotonic() - start > 2.5:
            state = 0
            ret = cps.HRIF_LongJogJ(BOX_ID, RBT_ID, axis_id, direction, state)
            print_ret(cps, "HRIF_LongJogJ(stop)", ret)
            if ret != 0 or not wait_motion_done(cps):
                cps.HRIF_DisConnect(BOX_ID)
                return 1
            break

    # ----------------------------------------------------------
    # 9. 断开连接
    # ----------------------------------------------------------
    print("[OK] Jogging Motion Completed.")
    ret = cps.HRIF_DisConnect(BOX_ID)
    print_ret(cps, "HRIF_DisConnect", ret)
    return 0 if ret == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
