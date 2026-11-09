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
    speed_override = 0.2
    ret = cps.HRIF_SetOverride(BOX_ID, RBT_ID, speed_override)
    print_ret(cps, "HRIF_SetOverride", ret)
    if ret != 0:
        cps.HRIF_DisConnect(BOX_ID)
        return 1
    safeJoints = [0, 0, 90, 0, 90, 0]
    ret = cps.HRIF_MoveJ(BOX_ID, RBT_ID,
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], safeJoints,
        "TCP", "Base", 30.0, 60.0, 0.0, 1, 0, 0, 0, "SpeedJ_start")
    print_ret(cps,"HRIF_MoveJ", ret)
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
    # 4. SpeedJ J1、J5 联动，近似在基坐标系 XY 水平面画小圆
    # ----------------------------------------------------------
    j1_amplitude = 12.0  # J1 角度波形幅值，deg
    j5_amplitude = 12.0  # J5 角度波形幅值，deg
    period = 7.5  # 一圈的运行时间，s
    command_interval = 0.05  # SpeedJ 指令更新周期，s
    sample_count = round(period / command_interval)
    dt = period / sample_count
    omega = 2.0 * math.pi / period
    joint_acc = 20.0  # 关节加速度，deg/s^2
    runtime = dt * 2.0  # 指令有效时间，需大于 dt

    # ----------------------------------------------------------
    # 5. 连续下发 J1、J5 正余弦速度
    # ----------------------------------------------------------
    print(f"[INFO] Starting SpeedJ approximate XY circle: period={period} s, "
          f"J1 amplitude={j1_amplitude} deg, J5 amplitude={j5_amplitude} deg, "
          f"peak J1/J5 speed={j1_amplitude * omega}/{j5_amplitude * omega} deg/s")
    next_send_time = time.perf_counter()

    for sample in range(sample_count):
        t = sample * dt
        j1_velocity = j1_amplitude * omega * math.cos(omega * t)
        j5_velocity = j5_amplitude * omega * math.sin(omega * t)
        velocity = [j1_velocity, 0.0, 0.0, 0.0, j5_velocity, 0.0]

        ret = cps.HRIF_SpeedJ(BOX_ID, RBT_ID, velocity, joint_acc, runtime)
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
    ret = cps.HRIF_SpeedJ( BOX_ID, RBT_ID,
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
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
