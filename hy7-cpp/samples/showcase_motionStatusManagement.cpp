#include "HR_Pro.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

// BoxID 和 RbtID 可以在此处进行修改
static const unsigned int BOX_ID = 0;
static const unsigned int RBT_ID = 0;

static bool print_ret(const std::string& command, int ret)
{
    if (ret == 0) {
        std::cout << "[OK]  " << command << std::endl;
        return true;
    }

    std::string message;
    const int lookupRet = HRIF_GetErrorCodeStr(BOX_ID, ret, message);
    std::cerr << "[ERR] " << command << " -> " << ret;
    if (lookupRet == 0) std::cerr << ": " << message;
    std::cerr << std::endl;
    return false;
}

static bool wait_motion_done()
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    while (std::chrono::steady_clock::now() < deadline)
    {
        bool done = false;
        const int ret = HRIF_IsMotionDone(BOX_ID, RBT_ID, done);
        if (ret != 0)
        {
            print_ret("HRIF_IsMotionDone", ret);
            return false;
        }
        if (done) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cerr << "[ERR] Motion timeout (60s)." << std::endl;
    return false;
}

int main()
{
    std::cout << "======== Motion Status Management Example ========" << std::endl;
    // ----------------------------------------------------------
    // 1. 连接机器人，确认处于使能状态 Standby(33) 并设置速度比为 0.2
    // ----------------------------------------------------------
    // 需根据机器人的实际 IP 进行修改
    const std::string host = "192.168.0.10";
    const unsigned int port = 10003;

    int ret = HRIF_Connect(BOX_ID, host.c_str(), port);
    print_ret("HRIF_Connect", ret);
    if (ret != 0)
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    int fsm = 0;
    ret = HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, fsm);
    print_ret("HRIF_ReadCurFSM (FSM=" + std::to_string(fsm) + ")", ret);
    if (ret != 0 || fsm != 33)
    {
        std::cerr << "[ERR] Robot must be in Standby(33), FSM=" << fsm << std::endl;
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    ret = HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
    print_ret("HRIF_SetOverride", ret);
    if (ret != 0)
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }
    // ----------------------------------------------------------
    // 2. MoveJ运动到安全原点
    // ----------------------------------------------------------
    JointsData safeJoints;
    safeJoints.dJoint[0] = -90;  safeJoints.dJoint[1] = -42.5;
    safeJoints.dJoint[2] = 0;   safeJoints.dJoint[3] = 72.5;
    safeJoints.dJoint[4] = 0;   safeJoints.dJoint[5] = -65;
    safeJoints.dJoint[6] = 0;

    ret = HRIF_MoveJ_nJ(BOX_ID, RBT_ID, 0, 0, 0, 0, 0, 0, safeJoints, "TCP", "Base",
                        30.0, 60.0, 0.0, 1, 0, 0, 0, "moveJ_to_safepoints");
    print_ret("HRIF_MoveJ_nJ (safepoint)", ret);
    if (ret != 0 || !wait_motion_done())
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }
    // ----------------------------------------------------------
    // 3. 连续下发用于演示暂停继续和停止的 MoveJ 运动
    // ----------------------------------------------------------
    ret = HRIF_SetBoxDO(BOX_ID, 0, 0);
    print_ret("HRIF_SetBoxDO (DO0=0)", ret);
    if (ret != 0)
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    safeJoints.dJoint[0] += 10.0;
    ret = HRIF_MoveJ_nJ(BOX_ID, RBT_ID, 0, 0, 0, 0, 0, 0, safeJoints, "TCP", "Base",
                        30.0, 60.0, 0.0, 1, 0, 0, 0, "motion_status_interrupt");
    print_ret("HRIF_MoveJ_nJ (interrupt demo)", ret);
    if (ret != 0)
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }
    
    ret = HRIF_cdsSetIO(BOX_ID, RBT_ID, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0.0, 0.0);
    print_ret("HRIF_cdsSetIO", ret);
    if (ret != 0)
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    safeJoints.dJoint[0] -= 10.0;
    ret = HRIF_MoveJ_nJ(BOX_ID, RBT_ID, 0, 0, 0, 0, 0, 0, safeJoints, "TCP", "Base",
                        30.0, 60.0, 0.0, 1, 0, 0, 0, "motion_status_stop");
    print_ret("HRIF_MoveJ_nJ (stop demo)", ret);
    if (ret != 0)
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }
    // ----------------------------------------------------------
    // 4. 识别到当前的运动ID，进行暂停继续以及停止的示例，并且最终根据输出 IO 判断运动是否完成
    // ----------------------------------------------------------
    std::string commandId;
    while (commandId != "motion_status_interrupt")
    {
        ret = HRIF_ReadCurWaypointID(BOX_ID, RBT_ID, commandId);
        if (ret != 0)
        {
            print_ret("HRIF_ReadCurWaypointID", ret);
            HRIF_DisConnect(BOX_ID);
            return EXIT_FAILURE;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "[INFO] Current motion stage ID is: " << commandId << std::endl;

    ret = HRIF_GrpInterrupt(BOX_ID, RBT_ID);
    print_ret("HRIF_GrpInterrupt", ret);
    if (ret != 0)
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    ret = HRIF_GrpContinue(BOX_ID, RBT_ID);
    print_ret("HRIF_GrpContinue", ret);
    if (ret != 0)
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    commandId.clear();
    while (commandId != "motion_status_stop")
    {
        ret = HRIF_ReadCurWaypointID(BOX_ID, RBT_ID, commandId);
        if (ret != 0)
        {
            print_ret("HRIF_ReadCurWaypointID", ret);
            HRIF_DisConnect(BOX_ID);
            return EXIT_FAILURE;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "[INFO] Current motion stage ID is: " << commandId << std::endl;

    ret = HRIF_GrpStop(BOX_ID, RBT_ID);
    print_ret("HRIF_GrpStop", ret);
    if (ret != 0)
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    int DO0 = 0;
    while (DO0 != 1)
    {
        ret = HRIF_ReadBoxDO(BOX_ID, 0, DO0);
        if (ret != 0)
        {
            print_ret("HRIF_ReadBoxDO", ret);
            HRIF_DisConnect(BOX_ID);
            return EXIT_FAILURE;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "[Info] Motion Done! Disconnecting" << std::endl;
    ret = HRIF_DisConnect(BOX_ID);
    print_ret("HRIF_DisConnect", ret);
    return ret == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
