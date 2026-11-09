#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Create, inspect, rename, and delete a temporary trajectory object."""

import sys

from CPS import CPSClient


# BoxID 和 RbtID 可以在此处进行修改
BOX_ID = 0
RBT_ID = 0
ORIGINAL_NAME = "sdk_manage_path_temp"
RENAMED_NAME = "sdk_manage_path_renamed"


def print_ret(cps, command, ret):
    if ret == 0:
        print("[OK]  " + command)
        return True

    message = []
    lookup_ret = cps.HRIF_GetErrorCodeStr(BOX_ID, ret, message)
    suffix = ": {}".format(message[0]) if lookup_ret == 0 and message else ""
    print("[ERR] {} -> {}{}".format(command, ret, suffix), file=sys.stderr)
    return False


def delete_path(cps, path_name, label):
    ret = cps.HRIF_DelPath(BOX_ID, RBT_ID, path_name)
    return print_ret(cps, label, ret)


def main():
    print("=== Trajectory Path Management Example ===")

    # ----------------------------------------------------------
    # 1. 连接机器人并确认处于 Standby(33)
    # ----------------------------------------------------------
    # 需根据机器人的实际 IP 进行修改
    host = "192.168.0.10"
    port = 10003

    cps = CPSClient()
    ret = cps.HRIF_Connect(BOX_ID, host, port)
    if not print_ret(cps, "HRIF_Connect", ret):
        return 1

    result = 0
    active_name = None
    try:
        fsm = []
        ret = cps.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, fsm)
        if not print_ret(cps, "HRIF_ReadCurFSMFromCPS", ret):
            return 1
        if not fsm or int(fsm[0]) != 33:
            print("[ERR] Robot must be in Standby(33), FSM={}".format(fsm), file=sys.stderr)
            return 1

        # ----------------------------------------------------------
        # 2. 确认临时名称未被占用，再创建专用于本案例的轨迹对象
        # ----------------------------------------------------------
        initial_paths = []
        ret = cps.HRIF_ReadPathList(BOX_ID, RBT_ID, initial_paths)
        if not print_ret(cps, "HRIF_ReadPathList (before create)", ret):
            return 1
        path_text = ",".join(str(value) for value in initial_paths)
        if ORIGINAL_NAME in path_text or RENAMED_NAME in path_text:
            print("[ERR] Temporary trajectory name is already in use", file=sys.stderr)
            return 1

        ret = cps.HRIF_InitPath(
            BOX_ID, RBT_ID, 0, ORIGINAL_NAME, 0.1, 5.0,
            30.0, 60.0, 500.0, "Base", "TCP"
        )
        if not print_ret(cps, "HRIF_InitPath", ret):
            return 1
        active_name = ORIGINAL_NAME

        # ----------------------------------------------------------
        # 3. 读取创建后的轨迹列表和临时轨迹详细信息
        # ----------------------------------------------------------
        path_list = []
        ret = cps.HRIF_ReadPathList(BOX_ID, RBT_ID, path_list)
        if not print_ret(cps, "HRIF_ReadPathList", ret):
            return 1
        print("[INFO] Path list: {}".format(path_list))

        path_info = []
        ret = cps.HRIF_ReadPathInfo(BOX_ID, RBT_ID, ORIGINAL_NAME, path_info)
        if not print_ret(cps, "HRIF_ReadPathInfo", ret):
            return 1
        print("[INFO] {} info: {}".format(ORIGINAL_NAME, path_info))

        # ----------------------------------------------------------
        # 4. 重命名轨迹并读取新名称下的信息
        # ----------------------------------------------------------
        ret = cps.HRIF_UpdatePathName(BOX_ID, RBT_ID, ORIGINAL_NAME, RENAMED_NAME)
        if not print_ret(cps, "HRIF_UpdatePathName", ret):
            return 1
        active_name = RENAMED_NAME

        renamed_info = []
        ret = cps.HRIF_ReadPathInfo(BOX_ID, RBT_ID, RENAMED_NAME, renamed_info)
        if not print_ret(cps, "HRIF_ReadPathInfo (renamed)", ret):
            return 1
        print("[INFO] {} info: {}".format(RENAMED_NAME, renamed_info))

        # ----------------------------------------------------------
        # 5. 删除临时轨迹，避免影响控制器中的业务轨迹
        # ----------------------------------------------------------
        if not delete_path(cps, RENAMED_NAME, "HRIF_DelPath"):
            return 1
        active_name = None
    finally:
        if active_name is not None:
            if not delete_path(cps, active_name, "HRIF_DelPath (cleanup)"):
                result = 1
        disconnect_ret = cps.HRIF_DisConnect(BOX_ID)
        if not print_ret(cps, "HRIF_DisConnect", disconnect_ret):
            result = 1

    return result


if __name__ == "__main__":
    sys.exit(main())
