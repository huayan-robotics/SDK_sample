#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Jogging example."""

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
    print("======== Jogging example started. ========")
    cps = CPSClient()

    # ----------------------------------------------------------
    # 1. 连接机器人，需根据机器人的实际 IP 进行修改
    # ----------------------------------------------------------
    host = "192.168.0.10"
    port = 10003

    ret = cps.HRIF_Connect(box_id, host, port)
    print_ret(cps, "HRIF_Connect", ret)
    if ret != 0:
        return 1

    # ----------------------------------------------------------
    # 2. 确认机器人处于 Standby(33) 就绪状态
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
    safe_joints = [0, -90, 90, -90, -90, 0]
    ret = cps.HRIF_MoveJ(box_id, rbt_id, [0, 0, 0, 0, 0, 0],
                         safe_joints, "TCP", "Base",
                         30.0, 60.0, 0.0, 1, 0, 0, 0, "Jog_start")
    print_ret(cps, "HRIF_MoveJ", ret)
    if ret != 0 or not wait_motion_done(cps):
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 5. 进行关节短点动，J1 轴正向运动 10°
    # ----------------------------------------------------------
    n_axis_id = 0      # 指定旋转轴，0~5 对应 J1~J6
    n_direction = 1   # 指定运动方向，0：反向；1：正向
    for i in range(5):
        ret = cps.HRIF_ShortJogJ(box_id, rbt_id, n_axis_id, n_direction)
        print_ret(cps, "HRIF_ShortJogJ", ret)
        if ret != 0 or not wait_motion_done(cps):
            cps.HRIF_DisConnect(box_id)
            return 1

    # ----------------------------------------------------------
    # 6. 进行空间短点动，Z 轴正向运动 20 mm
    # ----------------------------------------------------------
    n_axis_id = 2  # 指定空间偏移，0~5 对应 X/Y/Z/RX/RY/RZ
    for i in range(10):
        ret = cps.HRIF_ShortJogL(box_id, rbt_id, n_axis_id, n_direction)
        print_ret(cps, "HRIF_ShortJogL", ret)
        if ret != 0 or not wait_motion_done(cps):
            cps.HRIF_DisConnect(box_id)
            return 1

    # ----------------------------------------------------------
    # 7. 进行空间长点动，Z 轴反向运动；停止发送事件后等待停止
    # ----------------------------------------------------------
    n_state = 1       # 指定长点动运动状态，0：关闭；1：开启
    n_direction = 0  # 指定运动方向，0：反向；1：正向

    ret = cps.HRIF_LongJogL(box_id, rbt_id, n_axis_id, n_direction, n_state)
    print_ret(cps, "HRIF_LongJogL", ret)
    if ret != 0:
        cps.HRIF_DisConnect(box_id)
        return 1

    # 持续发送 HRIF_LongMoveEvent 维持长点动，采用 100 ms 周期。
    for i in range(5):
        time.sleep(0.1)
        ret = cps.HRIF_LongMoveEvent(box_id, rbt_id)
        cmd = "HRIF_LongMoveEvent" if i < 4 else "HRIF_LongMoveEvent(stop)"
        print_ret(cps, cmd, ret)
        if ret != 0:
            cps.HRIF_DisConnect(box_id)
            return 1

    # 等待最后一次 HRIF_LongMoveEvent 完成。
    if not wait_motion_done(cps):
        cps.HRIF_DisConnect(box_id)
        return 1

    # ----------------------------------------------------------
    # 8. 进行关节长点动，J5 轴反向运动；主动发送 n_state=0 停止
    # ----------------------------------------------------------
    n_axis_id = 4
    n_direction = 0
    ret = cps.HRIF_LongJogJ(box_id, rbt_id, n_axis_id, n_direction, n_state)
    print_ret(cps, "HRIF_LongJogJ", ret)
    if ret != 0:
        cps.HRIF_DisConnect(box_id)
        return 1

    # 持续发送 HRIF_LongMoveEvent 维持长点动，采用 100 ms 周期。
    t_start = time.monotonic()
    while True:
        ret = cps.HRIF_LongMoveEvent(box_id, rbt_id)
        print_ret(cps, "HRIF_LongMoveEvent", ret)
        if ret != 0:
            cps.HRIF_DisConnect(box_id)
            return 1
        time.sleep(0.1)

        t_now = time.monotonic()
        total_time = t_now - t_start
        if total_time > 2:
            n_state = 0
            ret = cps.HRIF_LongJogJ(box_id, rbt_id, n_axis_id, n_direction, n_state)
            print_ret(cps, "HRIF_LongJogJ(stop)", ret)
            if ret != 0 or not wait_motion_done(cps):
                cps.HRIF_DisConnect(box_id)
                return 1
            break

    # ----------------------------------------------------------
    # 9. 断开连接
    # ----------------------------------------------------------
    print("[OK] Jogging Motion Completed.")
    ret = cps.HRIF_DisConnect(box_id)
    print_ret(cps, "HRIF_DisConnect", ret)
    return 0 if ret == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
