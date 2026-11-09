#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read controller diagnostics, request one reset, and verify the FSM result."""

import sys
import time

from CPS import CPSClient

# BoxID 和 RbtID 可以在此处进行修改
BOX_ID = 0
RBT_ID = 0

def print_error_code(cps: CPSClient, source, error_code):
    # Preserve the original code and append a readable description when available.
    if error_code == 0:
        return
    message = []
    ret = cps.HRIF_GetErrorCodeStr(BOX_ID, error_code, message)
    suffix = ": {}".format(message[0]) if ret == 0 and message else ""
    print("[ERROR] {} -> {}{}".format(source, error_code, suffix), file=sys.stderr)

def error_category(error_code):
    # Use the leading digit only for quick grouping, not as a full diagnosis.
    return {
        1: "robot body",
        2: "controller internal",
        3: "external device communication",
        4: "controller internal",
        6: "specific function or device",
    }.get(error_code // 10000, "other")

def is_error_state(state):
    # Check whether the FSM is in an error or safety-event state covered here.
    return state in {5, 10, 12, 17, 21, 22, 40, 52}

def read_diagnostics(cps: CPSClient) -> bool:
    # ReadRobotState returns an error summary plus motion, safety, and power states.
    robot_state = []
    ret = cps.HRIF_ReadRobotState(BOX_ID, RBT_ID, robot_state)
    if ret != 0:
        print_error_code(cps, "HRIF_ReadRobotState", ret)
        return False
    if len(robot_state) < 13:
        print("[ERROR] HRIF_ReadRobotState returned incomplete data", file=sys.stderr)
        return False

    error_state = int(robot_state[2])
    error_code = int(robot_state[3])
    error_axis = int(robot_state[4])
    emergency_stop = int(robot_state[7])
    safe_guard = int(robot_state[8])
    print(
        "[STATE] error={}, code={}, category={}, axis={}, emergencyStop={}, safeGuard={}".format(
            error_state, error_code, error_category(error_code), error_axis,
            emergency_stop, safe_guard
        )
    )
    print_error_code(cps, "controller", error_code)

    # Per-axis codes supplement errorAxis and identify the affected joint.
    axis_errors = []
    ret = cps.HRIF_ReadAxisErrorCode(BOX_ID, RBT_ID, axis_errors)
    if ret != 0:
        print_error_code(cps, "HRIF_ReadAxisErrorCode", ret)
        return False
    if axis_errors:
        print_error_code(cps, "axis group", int(axis_errors[0]))
        for index, code in enumerate(axis_errors[1:7], 1):
            if int(float(code)) != 0:
                print_error_code(cps, "J{}".format(index), int(float(code)))

    # Observe channel mismatches and physical safety signals separately.
    emergency_info = []
    ret = cps.HRIF_ReadEmergencyInfo(BOX_ID, emergency_info, RBT_ID)
    if ret != 0:
        print_error_code(cps, "HRIF_ReadEmergencyInfo", ret)
        return False
    if len(emergency_info) < 4:
        print("[ERROR] HRIF_ReadEmergencyInfo returned incomplete data", file=sys.stderr)
        return False
    print(
        "[SAFETY] emergencyCircuitError={}, emergencySignal={}, "
        "safeguardCircuitError={}, safeguardSignal={}".format(*emergency_info[:4])
    )
    return True

def main():
    # This example does not create faults; use it only for a safely recoverable alarm.

    # ----------------------------------------------------------
    # 1. 连接机器人
    # ----------------------------------------------------------
    # 需根据机器人的实际 IP 进行修改
    host = "192.168.0.10"
    port = 10003

    cps = CPSClient()
    ret = cps.HRIF_Connect(BOX_ID, host, port)
    if ret != 0:
        print("[ERROR] HRIF_Connect -> {}".format(ret), file=sys.stderr)
        return 1

    # ----------------------------------------------------------
    # 2. 读取状态机并判断是否需要复位
    # ----------------------------------------------------------
    fsm = []
    ret = cps.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, fsm)
    if ret != 0:
        print_error_code(cps, "HRIF_ReadCurFSM", ret)
        cps.HRIF_DisConnect(BOX_ID)
        return 1
    state = int(fsm[0])
    print("[FSM] {}".format(state))
    if not is_error_state(state):
        print("[INFO] The state machine is not in an error state handled by this example; no reset is required.")
        cps.HRIF_DisConnect(BOX_ID)
        return 0

    # ----------------------------------------------------------
    # 3. 读取错误及安全诊断信息
    # ----------------------------------------------------------
    if not read_diagnostics(cps):
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 4. 确认故障原因已排除
    # ----------------------------------------------------------
    print(
        "Clear the emergency stop, safeguard, collision, communication, or "
        "equipment fault indicated by the diagnostics."
    )
    input("After confirming site safety and clearing the cause, press Enter to request one reset...")

    # ----------------------------------------------------------
    # 5. 请求复位机器人
    # ----------------------------------------------------------
    ret = cps.HRIF_GrpReset(BOX_ID, RBT_ID)
    if ret != 0:
        print_error_code(cps, "HRIF_GrpReset", ret)
        cps.HRIF_DisConnect(BOX_ID)
        return 1

    # ----------------------------------------------------------
    # 6. 轮询状态机并确认复位结果
    # ----------------------------------------------------------
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        fsm = []
        ret = cps.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, fsm)
        if ret != 0:
            print_error_code(cps, "HRIF_ReadCurFSM", ret)
            cps.HRIF_DisConnect(BOX_ID)
            return 1
        state = int(fsm[0])
        print("[FSM] {}".format(state))
        if not is_error_state(state):
            print("[OK] The state machine has left the error state. Resume operation through the state-machine flow.")
            cps.HRIF_DisConnect(BOX_ID)
            return 0
        time.sleep(0.2)

    print(

        "[ERROR] The state machine remains in an error state after reset; "

        "inspect the diagnostics or contact technical support.",

        file=sys.stderr,

    )
    cps.HRIF_DisConnect(BOX_ID)
    return 1

if __name__ == "__main__":
    sys.exit(main())
