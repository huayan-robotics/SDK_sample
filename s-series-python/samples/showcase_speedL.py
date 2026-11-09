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
    print("======== SpeedL Online Cartesian Velocity Control Example  ========")

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
    safeJoints = [0, -90, 90, -90, -90, 0]
    ret = cps.HRIF_MoveJ(BOX_ID,RBT_ID, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], safeJoints,
                        "TCP", "Base", 30.0, 60.0, 0.0, 1, 0, 0, 0, "SpeedL_start")
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

    ret = cps.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2)
    print_ret(cps, "HRIF_SetOverride", ret)
    if ret != 0:
        cps.HRIF_GrpStop(BOX_ID, RBT_ID)
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 4. 设置 SpeedL 轨迹参数
    # ----------------------------------------------------------
    linear_speed = 25.0
    linear_acc = 50.0
    angular_acc = 100.0
    direction_hold_time = 4.0
    command_runtime = 5.0
    sqrt3_over_2 = math.sqrt(3.0) / 2.0
    directions = [
        (1.0, 0.0),
        (0.5, sqrt3_over_2),
        (-0.5, sqrt3_over_2),
        (-1.0, 0.0),
        (-0.5, -sqrt3_over_2),
        (0.5, -sqrt3_over_2),
    ]

    # ----------------------------------------------------------
    # 5. 连续下发空间速度
    # ----------------------------------------------------------
    print("[INFO] Starting continuous SpeedL command streaming.")
    next_direction_time = time.perf_counter()

    for index, (direction_x, direction_y) in enumerate(directions, start=1):
        vx = linear_speed * direction_x
        vy = linear_speed * direction_y
        velocity = [vx, vy, 0.0, 0.0, 0.0, 0.0]

        print(f"[INFO] direction {index}: Vx={vx:.3f}, Vy={vy:.3f} mm/s")

        ret = cps.HRIF_SpeedL(BOX_ID,RBT_ID, velocity,linear_acc, angular_acc,command_runtime)

        print_ret(cps,f"HRIF_SpeedL(direction {index})", ret)
        if ret != 0:
            cps.HRIF_GrpStop(BOX_ID, RBT_ID)
            cps.HRIF_DisConnect(BOX_ID)
            return 1

        next_direction_time += direction_hold_time
        time.sleep(max(0.0, next_direction_time - time.perf_counter()))

    # ----------------------------------------------------------
    # 6. 发送零空间速度，使机器人平稳停止。
    # ----------------------------------------------------------
    stop_runtime = 1.0
    ret = cps.HRIF_SpeedL(BOX_ID, RBT_ID, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                             linear_acc, angular_acc,stop_runtime,)
    
    print_ret(cps,"HRIF_SpeedL(stop)", ret)
    time.sleep(stop_runtime)

    print("[OK] SpeedL motion completed.")
    stop_ret = cps.HRIF_GrpStop(BOX_ID, RBT_ID)
    print_ret(cps, "HRIF_GrpStop", stop_ret)
    disconnect_ret = cps.HRIF_DisConnect(BOX_ID)
    print_ret(cps, "HRIF_DisConnect", disconnect_ret)
    return 0 if ret == 0 and stop_ret == 0 and disconnect_ret == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
