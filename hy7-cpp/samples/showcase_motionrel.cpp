#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <cstdlib>

#include "HR_Pro.h"

// BoxID 和 RbtID 可以在此处进行修改
static const unsigned int BOX_ID = 0;
static const unsigned int RBT_ID = 0;

void print_ret(const std::string& cmd, int ret)
{
    if (ret == 0) {
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

    while ( std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        bool bDone = false;
        int ret = HRIF_IsMotionDone(BOX_ID, RBT_ID, bDone);
        if (ret != 0)
        {
            print_ret("HRIF_IsMotionDone", ret);
            return false;
        }
        if (bDone)
        {
            return true;
        }
    }
    std::cerr << "[ERR] Motion timeout (60s)." << std::endl;
    return false;
}

int main()
{
    std::cout << "======== Relative motion example started. ========" << std::endl;

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
    // 3. 设置全局速度比为0.2
    // ----------------------------------------------------------
    ret = HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
    print_ret("HRIF_SetOverride", ret);
    if (ret != 0) 
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 4. MoveJ运动到安全起点
    // ----------------------------------------------------------
    JointsData safeJoints;
    safeJoints.dJoint[0] = -90;   safeJoints.dJoint[1] = -42.5;
    safeJoints.dJoint[2] = 0;     safeJoints.dJoint[3] = 72.5;
    safeJoints.dJoint[4] = 0;     safeJoints.dJoint[5] = -65;
    safeJoints.dJoint[6] = 0;

    ret = HRIF_MoveJ_nJ(BOX_ID, RBT_ID, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, safeJoints,
                        "TCP", "Base", 30.0, 60.0, 0.0, 1, 0, 0, 0, "Jog_start");
    print_ret("HRIF_MoveJ_nJ", ret);
    if (ret != 0 || !wait_motion_done()) 
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 5. 进行单轴关节相对运动，J2轴正向旋转10°
    // ----------------------------------------------------------
    unsigned short nAxisId = 1;       // 0~6 对应 J1~J7
    unsigned short nDirection = 1;    // 1：正方向，0：负方向
    double dDistance = 10.0;           //相对距离

    ret = HRIF_MoveRelJ(BOX_ID, RBT_ID, nAxisId, nDirection, dDistance);
    print_ret("HRIF_MoveRelJ(J2 +10 deg)", ret);
    if (ret != 0 || !wait_motion_done()) 
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }
    // ----------------------------------------------------------
    // 6. MoveRelL：沿当前 UCS X 反方向移动 30 mm
    // ----------------------------------------------------------
    int nToolMotion = 0;   // 0：相对 UCS  1：相对 TCP
    nAxisId = 0;       // 0~5 对应 X、Y、Z、Rx、Ry、Rz
    nDirection = 0;     
    dDistance = 30.0;

    ret = HRIF_MoveRelL(BOX_ID, RBT_ID, nAxisId, nDirection, dDistance, nToolMotion);
    print_ret("HRIF_MoveRelL(X -30 mm)", ret);
    if (ret != 0 || !wait_motion_done()) 
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 7. WayPointRel：不使用点位列表，叠加值模式下，进行空间相对运动，X正向移动 30 mm，Y反向移动 30 mm
    // ----------------------------------------------------------
    int nType = 1;          // 1：直线运动
    int nPointList = 0;     // 不使用点位列表
    int nrelMoveType = 1;   // 使用叠加相对值
    JointsData pointJoints;
    JointsData axisMask;
    JointsData targets;
    axisMask.dJoint[0] = 1;
    axisMask.dJoint[1] = 1;
    targets.dJoint[0] = 30.0;
    targets.dJoint[1] = -30.0;

    ret = HRIF_WayPointRel_nJ(BOX_ID, RBT_ID, nType, nPointList, 0, 0, 0, 0, 0, 0, pointJoints,
                              nrelMoveType, axisMask, targets, "TCP", "Base",
                              30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPointRel_XY1");

    print_ret("HRIF_WayPointRel_nJ(X +30 mm, Y -30 mm)", ret);
    if (ret != 0 || !wait_motion_done()) 
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 8. WayPointRel：不使用点位列表，叠加值模式下，X反向移动 30 mm，Y 正向移动 30 mm
    // ----------------------------------------------------------
    targets.dJoint[0] = -30.0;
    targets.dJoint[1] = 30.0;
    ret = HRIF_WayPointRel_nJ(BOX_ID, RBT_ID, nType, nPointList, 0, 0, 0, 0, 0, 0, pointJoints,
                              nrelMoveType, axisMask, targets, "TCP", "Base",
                              30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPointRel_XY2");

    print_ret("HRIF_WayPointRel_nJ(X -30 mm, Y -30 mm)", ret);
    if (ret != 0 || !wait_motion_done())
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 9. WayPointRel：不使用点位列表，叠加值模式下，进行关节相对运动，J1 和 J2 同时旋转20°
    // ----------------------------------------------------------
    nType = 0; //关节运动
    targets.dJoint[0] = 10.0;
    targets.dJoint[1] = 10.0;
    ret = HRIF_WayPointRel_nJ(BOX_ID, RBT_ID, nType, nPointList, 0, 0, 0, 0, 0, 0, pointJoints,
                              nrelMoveType, axisMask, targets, "TCP", "Base",
                              30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPointRel_J1_J2");

    print_ret("HRIF_WayPointRel_nJ(J1 +10 deg, J2 +10 deg)", ret);
    if (ret != 0 || !wait_motion_done()) 
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 10. 断开连接
    // ----------------------------------------------------------
    std::cout << "[OK] Relative motion completed."<< std::endl;
    ret = HRIF_DisConnect(BOX_ID);
    print_ret("HRIF_DisConnect", ret);
    return ret == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
