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
    const std::string ip = "192.168.0.10";
    const unsigned int port = 10003;

    int ret = HRIF_Connect(BOX_ID, ip.c_str(), port);
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
    // 若设置了过渡半径，并且用HRIF_IsMotionDone命令判断路点运动是否完成，则过渡不生效
    // ----------------------------------------------------------
    double dJ1 = 0; double dJ2 = -90; double dJ3 = 90;
    double dJ4 = -90; double dJ5 = -90; double dJ6 = 0;

    ret = HRIF_MoveJ(BOX_ID, RBT_ID, 0, 0, 0, 0, 0, 0,
                     dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, "TCP", "Base",
                     30.0, 60.0, 0.0, 1, 0, 0, 0, "absolute_MoveJ_home");
    
                     print_ret("HRIF_MoveJ(Point A)", ret);
    if (ret != 0 || !wait_motion_done()) 
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 5.读取TCP当前位置，用于后续路点运动
    //从安全点A开始绘制云朵形状,如下所示
    //                         M
    //             Waypoint ╱     ╲ WaypointEx
    //                    C         D
    //                  ╭             ╮
    //               MoveC          WayPoint2
    //                  ╰             ╯
    //                    B ◄─MoveL── A
    // ----------------------------------------------------------
    const double dStraightLength = 80.0; //直线长度
    const double dArcRadius = 20.0; //圆弧半径
    double dX = 0.0; double dY = 0.0; double dZ = 0.0; 
    double dRx = 0.0; double dRy = 0.0; double dRz = 0.0;

    ret = HRIF_ReadActTcpPos(BOX_ID,RBT_ID,dX, dY, dZ,dRx, dRy, dRz);
    print_ret("HRIF_ReadActPos", ret);
    if (ret != 0) {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }
    // ----------------------------------------------------------
    // 6. 从A点 MoveL 直线运动到点B，无过渡
    // ----------------------------------------------------------
    dX += dStraightLength;
    ret = HRIF_MoveL(BOX_ID, RBT_ID, dX, dY, dZ, dRx, dRy, dRz,
                     0, 0, 0, 0, 0, 0, "TCP", "Base",
                     30.0, 60.0, 0.0, 0, 0, 0, "absolute_MoveL_approach");
    
                     print_ret("HRIF_MoveL(Point B)", ret);
    if (ret != 0 || ! wait_motion_done()) 
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 7. MoveC：经过辅助点执行 B -> C 圆弧，无过渡
    // ----------------------------------------------------------
    double dAuxPos_X = dX + dArcRadius;
    double dAuxPos_Y = dY + dArcRadius;

    double dEndPos_Y = dY + 2 * dArcRadius;

    ret = HRIF_MoveC(BOX_ID, RBT_ID, dX, dY, dZ, dRx, dRy, dRz,
                     dAuxPos_X, dAuxPos_Y, dZ, dRx, dRy, dRz,
                     dX, dEndPos_Y, dZ, dRx, dRy, dRz,
                     0, 1, 0.0, 30.0, 60.0, 0.0,"TCP", "Base", "absolute_MoveC_BC");
    
                     print_ret("HRIF_MoveC(B->C)", ret);
    if (ret != 0 || ! wait_motion_done()) 
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }
    // ----------------------------------------------------------
    // 8. WayPoint：C -> M，有过渡，过渡半径为 20.0 mm
    // ----------------------------------------------------------
    int nMoveType = 1;

    dX -= dStraightLength / 2.0 ;
    dY += 2.0 * dArcRadius + 50;

    ret = HRIF_WayPoint(BOX_ID, RBT_ID, nMoveType, dX, dY, dZ, dRx, dRy, dRz, 0, 0, 0, 0, 0, 0,
                        "TCP", "Base", 30.0, 60.0, 20.0, 0, 0, 0, 0, "WayPoint_C_M");
    
                        print_ret("HRIF_WayPoint(C->M)", ret);
    if (ret != 0 ) 
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 9. WayPointEx：M -> D，，有过渡，过渡半径为 20.0 mm
    // ----------------------------------------------------------
    dX -= dStraightLength / 2.0;
    dY -= 50;

    ret = HRIF_WayPointEx(BOX_ID, RBT_ID, nMoveType, dX, dY, dZ, dRx, dRy, dRz,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          30.0, 60.0, 20.0, 0, 0, 0, 0, "WayPointEx_M_D");

    print_ret("HRIF_WayPointEx(M->D)", ret);
    if (ret != 0 ) 
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 10. WayPoint2：D -> A，绘制圆弧，无过渡
    // ----------------------------------------------------------
    nMoveType = 2;

    dEndPos_Y = dY - 2 * dArcRadius;

    dAuxPos_X = dX - dArcRadius;
    dAuxPos_Y = dY - dArcRadius;

    ret = HRIF_WayPoint2(BOX_ID, RBT_ID, nMoveType, dX, dEndPos_Y, dZ, dRx, dRy, dRz,
                         dAuxPos_X, dAuxPos_Y, dZ, dRx, dRy, dRz,
                         0, 0, 0, 0, 0, 0, "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPoint2_D_A");

    print_ret("HRIF_WayPoint2(D->A)", ret);
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
