#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Run the customer workflow from connection to safe shutdown."""

import sys
import time
from enum import IntEnum

from CPS import CPSClient

# BoxID 和 RbtID 可以在此处进行修改
BOX_ID = 0
RBT_ID = 0
TRANSITION_TIMEOUT_SECONDS = 60

# This is intentionally not a complete controller FSM definition.
class FsmState(IntEnum):
    BOX_DISCONNECTED = 2
    EMERGENCY_STOPPED = 5
    BLACKOUT = 7
    SAFETY_IO_ERROR = 10
    SAFEGUARDING = 12
    CONTROLLER_DISCONNECTED = 14
    CONTROLLER_VERSION_ERROR = 16
    ETHERCAT_ERROR = 17
    MANUAL_GUIDING = 20
    COLLISION_STOP = 21
    ROBOT_ERROR = 22
    DISABLED = 24
    MOVING = 25
    LONG_JOG_MOVING = 26
    FREE_DRIVE = 31
    PAUSED = 32
    STANDBY = 33
    PROGRAM_RUNNING = 34
    PROGRAM_HELD = 36
    PLUGIN_DISCONNECTED = 39
    PLUGIN_ERROR = 40
    LOW_TEMPERATURE = 43
    SAFEMOVE_ERROR = 52


BUSINESS_ACTIVE_STATES = {
    FsmState.MOVING,
    FsmState.LONG_JOG_MOVING,
    FsmState.PAUSED,
    FsmState.PROGRAM_RUNNING,
    FsmState.PROGRAM_HELD,
}
ABNORMAL_STATE_GUIDANCE = {
    FsmState.EMERGENCY_STOPPED: "safety stop; restore and verify the external safety circuit first",
    FsmState.SAFETY_IO_ERROR: "safety stop; restore and verify the external safety circuit first",
    FsmState.SAFEGUARDING: "safety stop; restore and verify the external safety circuit first",
    FsmState.CONTROLLER_VERSION_ERROR: "maintenance condition; operator or service handling is required",
    FsmState.ETHERCAT_ERROR: "controller error; diagnose the cause before requesting Reset",
    FsmState.COLLISION_STOP: "collision stop; inspect the robot and work area before requesting Reset",
    FsmState.ROBOT_ERROR: "controller error; diagnose the cause before requesting Reset",
    FsmState.PLUGIN_DISCONNECTED: "maintenance condition; operator or service handling is required",
    FsmState.PLUGIN_ERROR: "controller error; diagnose the cause before requesting Reset",
    FsmState.LOW_TEMPERATURE: "maintenance condition; operator or service handling is required",
    FsmState.SAFEMOVE_ERROR: "controller error; diagnose the cause before requesting Reset",
}


def print_ret(cps, command, ret):
    # Print call results consistently and return whether the call succeeded.
    if ret == 0:
        print("[OK]   {}".format(command))
        return True
    if ret == 39500:
        print("[ERR]  {} -> 39500: Robot not connected".format(command), file=sys.stderr)
        return False
    message = []
    lookup_ret = cps.HRIF_GetErrorCodeStr(BOX_ID, ret, message)
    suffix = ": {}".format(message[0]) if lookup_ret == 0 and message else ""
    print("[ERR]  {} -> {}{}".format(command, ret, suffix), file=sys.stderr)
    return False


def read_state(cps):
    fsm = []
    ret = cps.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, fsm)
    if ret == 0:
        return int(fsm[0])
    print_ret(cps, "HRIF_ReadCurFSMFromCPS", ret)
    print("[DIAG] State read failed; no transition decision was made", file=sys.stderr)
    return None


def report_abnormal_state(state):
    guidance = ABNORMAL_STATE_GUIDANCE.get(state)
    if guidance is None:
        return False
    print("[ERR]  FSM={}: {}".format(state, guidance), file=sys.stderr)
    return True


def start_robot(cps):
    deadline = time.monotonic() + TRANSITION_TIMEOUT_SECONDS
    previous_state = -1

    while time.monotonic() < deadline:
        state = read_state(cps)
        if state is None:
            return False
        if state == FsmState.STANDBY:
            print("[READY] Robot reached Standby(33)")
            return True
        if report_abnormal_state(state):
            return False

        if state != previous_state:
            actions = {
                FsmState.BOX_DISCONNECTED: ("HRIF_Connect2Box", lambda: cps.HRIF_Connect2Box(BOX_ID)),
                FsmState.BLACKOUT: ("HRIF_Electrify", lambda: cps.HRIF_Electrify(BOX_ID)),
                FsmState.CONTROLLER_DISCONNECTED: (
                    "HRIF_Connect2Controller", lambda: cps.HRIF_Connect2Controller(BOX_ID)
                ),
                FsmState.DISABLED: ("HRIF_GrpEnable", lambda: cps.HRIF_GrpEnable(BOX_ID, RBT_ID)),
                FsmState.MANUAL_GUIDING: ("HRIF_MoveToSS", lambda: cps.HRIF_MoveToSS(BOX_ID, RBT_ID)),
                FsmState.FREE_DRIVE: (
                    "HRIF_GrpCloseFreeDriver",
                    lambda: cps.HRIF_GrpCloseFreeDriver(BOX_ID, RBT_ID),
                ),
            }
            action = actions.get(state)
            # Wait for the controller to complete the current transition.
            if action and not print_ret(cps, action[0], action[1]()):
                print("[DIAG] Startup transition failed at FSM={}".format(state), file=sys.stderr)
                return False

        previous_state = state
        time.sleep(0.1)

    print(
        "[ERR]  Robot did not reach Standby(33) within {} seconds".format(
            TRANSITION_TIMEOUT_SECONDS
        ),
        file=sys.stderr,
    )
    return False


def shutdown_robot(cps):
    deadline = time.monotonic() + TRANSITION_TIMEOUT_SECONDS
    previous_state = -1

    while time.monotonic() < deadline:
        state = read_state(cps)
        if state is None:
            return False
        if state == FsmState.BLACKOUT:
            print("[SAFE]  Robot reached Blackout(7)")
            return True
        if report_abnormal_state(state):
            return False

        if state != previous_state:
            action = None
            if state in BUSINESS_ACTIVE_STATES:
                action = ("HRIF_GrpStop", lambda: cps.HRIF_GrpStop(BOX_ID, RBT_ID))
            elif state == FsmState.STANDBY:
                action = ("HRIF_GrpDisable", lambda: cps.HRIF_GrpDisable(BOX_ID, RBT_ID))
            elif state in {FsmState.DISABLED, FsmState.CONTROLLER_DISCONNECTED}:
                action = ("HRIF_BlackOut", lambda: cps.HRIF_BlackOut(BOX_ID))
            elif state == FsmState.MANUAL_GUIDING:
                action = ("HRIF_MoveToSS", lambda: cps.HRIF_MoveToSS(BOX_ID, RBT_ID))
            elif state == FsmState.FREE_DRIVE:
                action = (
                    "HRIF_GrpCloseFreeDriver",
                    lambda: cps.HRIF_GrpCloseFreeDriver(BOX_ID, RBT_ID),
                )
            # Wait for the controller to complete the current transition.
            if action and not print_ret(cps, action[0], action[1]()):
                print("[DIAG] Shutdown transition failed at FSM={}".format(state), file=sys.stderr)
                return False

        previous_state = state
        time.sleep(0.1)

    print(
        "[ERR]  Robot did not reach Blackout(7) within {} seconds".format(
            TRANSITION_TIMEOUT_SECONDS
        ),
        file=sys.stderr,
    )
    return False


def run_business_task():
    print("[BUSINESS] Execute the application task here")
    return True


def main():
    # ----------------------------------------------------------
    # 1. Connect：连接机器人
    # ----------------------------------------------------------
    # 需根据机器人的实际 IP 进行修改
    host = "192.168.0.10"
    port = 10003

    print("=== Controller State Machine Workflow Example ===")
    cps = CPSClient()
    ret = cps.HRIF_Connect(BOX_ID, host, port)
    print_ret(cps, "HRIF_Connect", ret)
    if ret != 0:
        return 1

    result = 0

    # ----------------------------------------------------------
    # 2. Standby：上电、初始化并使能机器人
    # ----------------------------------------------------------
    if not start_robot(cps):
        result = 1

    # ----------------------------------------------------------
    # 3. Business：执行业务任务
    # ----------------------------------------------------------
    if result == 0 and not run_business_task():
        print("[DIAG] Business task failed; state transition did not fail", file=sys.stderr)
        result = 1

    # ----------------------------------------------------------
    # 4. Shutdown：停止、去使能并下电
    # ----------------------------------------------------------
    if result == 0 and not shutdown_robot(cps):
        result = 1

    # ----------------------------------------------------------
    # 5. Disconnect：断开机器人连接
    # ----------------------------------------------------------
    ret = cps.HRIF_DisConnect(BOX_ID)
    if not print_ret(cps, "HRIF_DisConnect", ret):
        result = 1
    return result


if __name__ == "__main__":
    sys.exit(main())
