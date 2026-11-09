    # -*- coding: utf-8 -*-
import math
import time
import sys

from CPS import CPSClient

# BoxID 和 RbtID 可以在此处进行修改
BOX_ID = 0
RBT_ID = 0 

def print_ret(cps, cmd, ret):
    if ret == 0:
        print("[OK]  " + cmd)
    else:
        err_msg = []
        lookup_ret = cps.HRIF_GetErrorCodeStr(BOX_ID, ret, err_msg)
        suffix = ": {}".format(err_msg[0]) if lookup_ret == 0 and err_msg else ""
        print("[ERR] {} -> {}{}".format(cmd, ret, suffix))

def main():
    print("=== ServoJ 6-Axis Joint-Space Online Control Example ===")

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
    # 2. 确认机器人处于 Standby(33)
    # ----------------------------------------------------------
    fsm_result = []
    ret = cps.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, fsm_result)
    nFSM = int(fsm_result[0]) if fsm_result else 0
    print_ret(cps, "HRIF_ReadCurFSM (FSM={})".format(nFSM), ret)
    if ret != 0 or nFSM != 33:
        print("[ERR] Robot must be in Standby(33), FSM={}".format(nFSM))
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 3. 读取当前关节位置
    # ----------------------------------------------------------
    JOINT_COUNT    = 6
    cur_joints = []
    ret = cps.HRIF_ReadActJointPos(BOX_ID, RBT_ID, cur_joints)
    print_ret(cps, "HRIF_ReadActJointPos", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1
    cur_joints = [float(x) for x in cur_joints[:JOINT_COUNT]]

    ret = cps.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2)
    print_ret(cps, "HRIF_SetOverride", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 4. 设置速度比并 MoveJ 运动到安全起始位姿
    # ----------------------------------------------------------
    safe_joints = [0, -90, 90, -90, -90, 0]
    points = [0.0] * 6   # dX~Rz (IsUseJoint=1 时忽略)

    print("[INFO] Moving to a safe position with MoveJ...")
    ret = cps.HRIF_MoveJ(BOX_ID, RBT_ID, points, safe_joints,
                            "TCP", "Base",
                            30.0, 60.0, 0.0,    # velocity, acc, radius
                            1, 0, 0, 0, "servo_home")  # IsUseJoint=1
    print_ret(cps, "HRIF_MoveJ", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    done = [False]
    motion_deadline = time.monotonic() + 60.0
    while not done[0] and time.monotonic() < motion_deadline:
        time.sleep(0.05)
        ret = cps.HRIF_IsMotionDone(BOX_ID, RBT_ID, done)
        if ret != 0:
            print_ret(cps, "HRIF_IsMotionDone", ret)
            cps.HRIF_DisConnect(BOX_ID)
            return 1
    if not done[0]:
        print("[ERR] MoveJ timed out")
        cps.HRIF_DisConnect(BOX_ID)
        return 1
    print("[INFO] Safe position reached")

    # ----------------------------------------------------------
    # 5. 读取当前关节位置 (MoveJ 后的基准)
    # ----------------------------------------------------------
    cur_joints = []
    ret = cps.HRIF_ReadActJointPos(BOX_ID, RBT_ID, cur_joints)
    print_ret(cps, "HRIF_ReadActJointPos (after MoveJ)", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1
    cur_joints = [float(x) for x in cur_joints[:JOINT_COUNT]]

    # ----------------------------------------------------------
    # 6. 启动 Servo 模式 伺服周期模式
    # ----------------------------------------------------------
    SERVO_TIME     = 0.01   # 伺服周期 10ms
    LOOKAHEAD_TIME = 0.05   # 前瞻 50ms
    print("[INFO] Servo cycle={}s, lookahead={}s".format(SERVO_TIME, LOOKAHEAD_TIME))
    ret = cps.HRIF_StartServo(BOX_ID, RBT_ID, SERVO_TIME, LOOKAHEAD_TIME)
    print_ret(cps, "HRIF_StartServo", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    time.sleep(0.001)

    # ----------------------------------------------------------
    # 7. 先推送当前关节位置 (同步，避免首帧突变)
    # ----------------------------------------------------------
    ret = cps.HRIF_PushServoJ(BOX_ID, RBT_ID, cur_joints)
    print_ret(cps, "HRIF_PushServoJ (sync)", ret)
    if ret != 0:
        cps.HRIF_GrpStop(BOX_ID, RBT_ID)
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 8. 基于当前关节位置生成正弦轨迹并推送
    # ----------------------------------------------------------
    AMP            = 5.0    # 振幅 5°
    FREQ           = 0.5    # 频率 0.5Hz
    DURATION       = 2.0    # 持续 2 秒
    direction = [1, 1, 1, 1, 1, 0]
    total = int(DURATION / SERVO_TIME)

    print("[INFO] Streaming {} points (multi-joint sine, +/-{} deg)...".format(total, AMP))
    t0 = time.perf_counter()
    max_push_ms = 0.0
    sum_push_ms = 0.0
    ok_cnt = 0
    err_cnt = 0

    for i in range(1, total + 1):
        # HRIF_PushServoJ 会立即返回，需要主动保持 10ms 的推送周期。
        next_time = t0 + i * SERVO_TIME
        time.sleep(max(0.0, next_time - time.perf_counter()))
        t = i * SERVO_TIME

        offset = AMP * math.sin(2.0 * math.pi * FREQ * t)
        target = [cur_joints[j] + direction[j] * offset for j in range(JOINT_COUNT)]

        push_start = time.perf_counter()
        ret = cps.HRIF_PushServoJ(BOX_ID, RBT_ID, target)
        push_ms = (time.perf_counter() - push_start) * 1000.0

        if push_ms > max_push_ms:
            max_push_ms = push_ms
        sum_push_ms += push_ms

        if ret != 0:
            err_cnt += 1
            if err_cnt <= 3:
                print("[ERR] Point {} failed, code={}, push time={:.2f}ms".format(i, ret, push_ms))
        else:
            ok_cnt += 1

    avg_ms = sum_push_ms / (ok_cnt + err_cnt) if (ok_cnt + err_cnt) > 0 else 0
    print("[INFO] Completed: succeeded={}, failed={}, average push time={:.2f}ms, maximum push time={:.2f}ms".format(
        ok_cnt, err_cnt, avg_ms, max_push_ms))

    # ----------------------------------------------------------
    # 9. 停止并断开连接
    # ----------------------------------------------------------
    print("[INFO] ServoJ completed; disconnecting")
    stop_ret = cps.HRIF_GrpStop(BOX_ID, RBT_ID)
    print_ret(cps, "HRIF_GrpStop", stop_ret)
    disconnect_ret = cps.HRIF_DisConnect(BOX_ID)
    print_ret(cps, "HRIF_DisConnect", disconnect_ret)
    return 0 if err_cnt == 0 and stop_ret == 0 and disconnect_ret == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
