#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <cstdlib>

#include "HR_Pro.h"
// BoxID 和 RbtID 可以在此处进行修改
static const unsigned int BOX_ID = 0;
static const unsigned int RBT_ID = 0;

void print_ret(const std::string &cmd, int ret)
{
    if (ret == 0)
    {
        std::cout << "[OK]  " << cmd << std::endl;
    }
    else
    {
        std::string errMsg;
        const int lookupRet = HRIF_GetErrorCodeStr(BOX_ID, ret, errMsg);
        std::cout << "[ERR] " << cmd << " -> " << ret;
        if (lookupRet == 0)
            std::cout << ": " << errMsg;
        std::cout << std::endl;
    }
}

bool wait_motion_done()
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);

    while (std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        bool bDone = false;
        int ret = HRIF_IsMotionDone(BOX_ID, RBT_ID, bDone);
        if (ret != 0)
        {
            print_ret("HRIF_IsMotionDone", ret);
            return false;
        }
        if(bDone) 
        {
            return true;
        }
    }
    std::cerr << "[ERR] Motion timeout (60s)." << std::endl;
    return false;
}

int main()
{
    std::cout << "======== Waypoint Motion Example Start ========" << std::endl;

    // ----------------------------------------------------------
    // 1. 连接机器人
    // ----------------------------------------------------------
    // 需根据机器人的实际 IP 进行修改
    const std::string host = "192.168.0.10";
    const unsigned int port = 10003;

    int ret = HRIF_Connect(BOX_ID, host.c_str(), port);
    print_ret("HRIF_Connect", ret);
    if (ret != 0)
    {
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 2. 确认机器人处于 Standby(33)
    // ----------------------------------------------------------
    int nFSM = 0;
    ret = HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, nFSM);
    print_ret("HRIF_ReadCurFSM (FSM=" + std::to_string(nFSM) + ")", ret);
    if (ret != 0 || nFSM != 33)
    {
        std::cerr << "[ERR] Robot must be in Standby(33), FSM=" << nFSM << std::endl;
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 3. 设置全局速度比为 0.2
    // ----------------------------------------------------------
    ret = HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
    print_ret("HRIF_SetOverride", ret);
    if (ret != 0)
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 4. MoveJ运动到安全关节位置, 无过渡，过渡半径为 0.0 mm；
    // 若设置了过渡半径，并且用HRIF_IsMotionDone命令判断此条路点运动是否完成，则过渡不生效
    // ----------------------------------------------------------
    JointsData safeJoints;
    safeJoints.dJoint[0] = 90;   safeJoints.dJoint[1] = 45;
    safeJoints.dJoint[2] = 0;     safeJoints.dJoint[3] = -80;
    safeJoints.dJoint[4] = 0;     safeJoints.dJoint[5] = 35;
    safeJoints.dJoint[6] = 0;

    ret = HRIF_MoveJ_nJ(BOX_ID, RBT_ID, 0, 0, 0, 0, 0, 0, safeJoints, "TCP", "Base",
                        30.0, 60.0, 0.0, 1, 0, 0, 0, "absolute_MoveJ_home");

    print_ret("HRIF_MoveJ_nJ(Point A)", ret);
    if (ret != 0 || !wait_motion_done()) 
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 5.读取TCP当前位置，用于后续路点运动
    // 从安全点 A 开始在 XZ 平面绘制云朵形状，A 到 B 沿 Z 负方向，云朵向 X 正方向展开，Y 保持不变
    //                         M
    //             Waypoint ╱     ╲ WaypointEx
    //                    C         D
    //                  ╭             ╮
    //            WayPoint2          WayPoint2
    //                  ╰             ╯
    //                    B ◄─MoveL── A
    // ----------------------------------------------------------
    const double dStraightLength = 30.0; //胶囊形直线长度
    const double dArcRadius = 10.0; //胶囊圆弧半径
    double dX = 0.0; double dY = 0.0; double dZ = 0.0; 
    double dRx = 0.0; double dRy = 0.0; double dRz = 0.0;

    ret = HRIF_ReadActTcpPos(BOX_ID,RBT_ID,dX, dY, dZ,dRx, dRy, dRz);
    print_ret("HRIF_ReadActPos", ret);
    if (ret != 0) {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }
    // ----------------------------------------------------------
    // 6. 从 A 点沿 Z 负方向 MoveL 直线运动到点 B，无过渡
    // ----------------------------------------------------------
    dZ -= dStraightLength;
    ret = HRIF_MoveL_nJ(BOX_ID, RBT_ID, dX, dY, dZ, dRx, dRy, dRz, safeJoints,
                        "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, "absolute_MoveL_approach");

    print_ret("HRIF_MoveL_nJ(Point B)", ret);
    if (ret != 0 || ! wait_motion_done()) 
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 7. WayPoint2：经过辅助点执行 B -> C 圆弧，无过渡
    // ----------------------------------------------------------
    double dAuxPos_X = dX + dArcRadius;
    double dAuxPos_Z = dZ - dArcRadius;

    double dEndPos_X = dX + 2 * dArcRadius;

    /* HRIF_MoveC_nJ 暂不使用。
    ret = HRIF_MoveC_nJ(BOX_ID, RBT_ID, dX, dY, dZ, dRx, dRy, dRz,
                        dAuxPos_X, dY, dAuxPos_Z, dRx, dRy, dRz,
                        dEndPos_X, dY, dZ, dRx, dRy, dRz,
                        0, 1, 0.0, 30.0, 60.0, 0.0, "TCP", "Base", "absolute_MoveC_BC", safeJoints, 0);
    */

    ret = HRIF_WayPoint2_nJ(BOX_ID, RBT_ID, 2, dEndPos_X, dY, dZ, dRx, dRy, dRz,
                            dAuxPos_X, dY, dAuxPos_Z, dRx, dRy, dRz, safeJoints,
                            "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPoint2_B_C");

    print_ret("HRIF_WayPoint2_nJ(B->C)", ret);
    if (ret != 0 || ! wait_motion_done()) 
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }
    // ----------------------------------------------------------
    // 8. WayPoint：C -> M，有过渡，过渡半径为 20.0 mm
    // ----------------------------------------------------------
    int nMoveType = 1;

    dZ += dStraightLength / 4.0;
    dX += 2.0 * dArcRadius + 30;

    ret = HRIF_WayPoint_nJ(BOX_ID, RBT_ID, nMoveType, dX, dY, dZ, dRx, dRy, dRz, safeJoints,
                           "TCP", "Base", 30.0, 60.0, 20.0, 0, 0, 0, 0, "WayPoint_C_M");

    print_ret("HRIF_WayPoint_nJ(C->M)", ret);
    if (ret != 0 ) 
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 9. WayPointEx：M -> D，，有过渡，过渡半径为 20.0 mm
    // ----------------------------------------------------------
    dZ += dStraightLength / 4.0;
    dX -= 30;

    ret = HRIF_WayPointEx_nJ(BOX_ID, RBT_ID, nMoveType, dX, dY, dZ, dRx, dRy, dRz, safeJoints,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             30.0, 60.0, 20.0, 0, 0, 0, 0, "WayPointEx_M_D");

    print_ret("HRIF_WayPointEx_nJ(M->D)", ret);
    if (ret != 0 ) 
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 10. WayPoint2：D -> A，绘制圆弧，无过渡
    // ----------------------------------------------------------
    nMoveType = 2;

    dEndPos_X = dX - 2 * dArcRadius;

    dAuxPos_X = dX - dArcRadius/2.0;
    dAuxPos_Z = dZ + dArcRadius/2.0;

    ret = HRIF_WayPoint2_nJ(BOX_ID, RBT_ID, nMoveType, dEndPos_X, dY, dZ, dRx, dRy, dRz,
                            dAuxPos_X, dY, dAuxPos_Z, dRx, dRy, dRz, safeJoints,
                            "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPoint2_D_A");

    print_ret("HRIF_WayPoint2_nJ(D->A)", ret);
    if (ret != 0 || ! wait_motion_done()) 
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 11. 断开连接
    // ----------------------------------------------------------
    std::cout << "[OK] Absolute motion completed."<< std::endl;
    ret = HRIF_DisConnect(BOX_ID);
    print_ret("HRIF_DisConnect", ret);
    return ret == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
