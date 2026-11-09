import sys
import math
import time

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
    cps = CPSClient()
    print("======== SpeedJ Online Joint Velocity Control Example ========")

    # ----------------------------------------------------------
    # 1. 连接机器人
    # ----------------------------------------------------------
    # 需根据机器人的实际 IP 进行修改
    host = "192.168.0.10"
    port = 10003
    ret = cps.HRIF_Connect(BOX_ID, host, port)
    print_ret(cps,"HRIF_Connect", ret)
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
    # 3. 设置速度比并 MoveJ 运动到安全起始位姿
    # ----------------------------------------------------------
    ret = cps.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2)
    print_ret(cps, "HRIF_SetOverride", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1
    safeJoints =  [-90, -42.5, 0, 72.5, 0, -65, 0]
    points = [0.0] * 6 
    ret = cps.HRIF_MoveJ_nJ(BOX_ID, RBT_ID,
        points, safeJoints,
        "TCP", "Base", 30.0, 60.0, 0.0, 1, 0, 0, 0, "SpeedJ_start")
    print_ret(cps,"HRIF_MoveJ_nJ", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    motion_done = []
    motion_deadline = time.monotonic() + 60.0
    while time.monotonic() < motion_deadline:
        ret = cps.HRIF_IsMotionDone(BOX_ID, RBT_ID, motion_done)
        if ret != 0:
            print_ret(cps,"HRIF_IsMotionDone", ret)
            cps.HRIF_DisConnect(BOX_ID)
            return 1
        if motion_done and motion_done[0]:
            break
        time.sleep(0.02)
    if not motion_done or not motion_done[0]:
        print("[ERR] Timed out waiting for motion completion")
        cps.HRIF_GrpStop(BOX_ID, RBT_ID)
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 4. 设置 SpeedJ 轨迹参数
    # ----------------------------------------------------------
    period = 8.0
    amplitude = 10.0
    dt = 0.05
    omega = 2.0 * math.pi / period
    joint_acc = 50.0
    runtime = dt * 2.0
    sample_count = round(period / dt)

    # ----------------------------------------------------------
    # 5. 连续下发关节速度，绘制一个圆形轨迹
    # ----------------------------------------------------------
    print("[INFO] Starting continuous SpeedJ command streaming.")
    next_send_time = time.perf_counter()

    for sample in range(sample_count):
        t = sample * dt
        j1_velocity = amplitude * omega * math.cos(omega * t)
        j2_velocity = -amplitude * omega * math.sin(omega * t)
        velocity = [j1_velocity, j2_velocity, 0.0, 0.0, 0.0, 0.0, 0.0]

        ret = cps.HRIF_SpeedJ_nJ(BOX_ID, RBT_ID, velocity, joint_acc, runtime)
        if ret != 0:
            print_ret(cps,f"HRIF_SpeedJ(sample {sample + 1})", ret)
            cps.HRIF_GrpStop(BOX_ID, RBT_ID)
            cps.HRIF_DisConnect(BOX_ID)
            return 1

        next_send_time += dt
        time.sleep(max(0.0, next_send_time - time.perf_counter()))

    # ----------------------------------------------------------
    # 6. 发送零关节速度，使机器人平稳停止。
    # ----------------------------------------------------------
    stop_runtime = 0.5
    ret = cps.HRIF_SpeedJ_nJ( BOX_ID, RBT_ID,
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        joint_acc, stop_runtime)
    print_ret(cps,"HRIF_SpeedJ(stop)", ret)
    time.sleep(stop_runtime)

    print("[OK] SpeedJ motion completed.")
    stop_ret = cps.HRIF_GrpStop(BOX_ID, RBT_ID)
    print_ret(cps, "HRIF_GrpStop", stop_ret)
    disconnect_ret = cps.HRIF_DisConnect(BOX_ID)
    print_ret(cps, "HRIF_DisConnect", disconnect_ret)
    return 0 if ret == 0 and stop_ret == 0 and disconnect_ret == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
