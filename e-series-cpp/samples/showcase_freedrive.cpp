#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "HR_Pro.h"

// BoxID 和 RbtID 可以在此处进行修改
static const unsigned int BOX_ID = 0;
static const unsigned int RBT_ID = 0;

void print_ret(const std::string& cmd, int ret)
{
    if (ret == 0) {
        std::cout << "[OK]  " << cmd << std::endl;
    } else {
        std::string errMsg;
        const int lookupRet = HRIF_GetErrorCodeStr(BOX_ID, ret, errMsg);
        std::cout << "[ERR] " << cmd << " -> " << ret;
        if (lookupRet == 0) std::cout << ": " << errMsg;
        std::cout << std::endl;
    }
}

int main()
{
    std::cout << "======== Force Free Drive Example ========" << std::endl;
    // ----------------------------------------------------------
    // 1. 连接机器人
    // ----------------------------------------------------------
    const std::string ip = "192.168.0.10";
    const unsigned int port = 10003;
    int ret = HRIF_Connect(BOX_ID, ip.c_str(), port);
    print_ret("HRIF_Connect", ret);
    if (ret != 0) {
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 2. 检查机器人是否为使能状态
    // ----------------------------------------------------------
    int nFSM = -1;
    ret = HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, nFSM);
    print_ret("HRIF_ReadCurFSM (FSM=" + std::to_string(nFSM) + ")", ret);
    if (ret != 0) {
        print_ret("HRIF_DisConnect", HRIF_DisConnect(BOX_ID));
        return EXIT_FAILURE;
    }
    if (nFSM != 33) {
        std::cerr << "[ERR] Robot must be in Standby(33), FSM=" << nFSM << std::endl;
        print_ret("HRIF_DisConnect", HRIF_DisConnect(BOX_ID));
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 3. 开启 Z 方向自由度
    // ----------------------------------------------------------
    ret = HRIF_SetFreeDriveMotionFreedom(BOX_ID, RBT_ID, 0, 0, 1, 0, 0, 0);
    print_ret("HRIF_SetFreeDriveMotionFreedom", ret);
    if (ret != 0) {
        print_ret("HRIF_DisConnect", HRIF_DisConnect(BOX_ID));
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 4. 设置启动阈值、速度和柔顺度
    // ----------------------------------------------------------
    const double dForceThreshold = 5.0;   //启动阈值  > 0.1 且 < 100
    const double dTorqueThreshold = 0.4;  // 启动阈值 > 0.1 且 < 10
    ret = HRIF_SetFTWrenchThresholds(BOX_ID, RBT_ID, dForceThreshold, dTorqueThreshold);
    print_ret("HRIF_SetFTWrenchThresholds", ret);
    if (ret != 0) {
        print_ret("HRIF_DisConnect", HRIF_DisConnect(BOX_ID));
        return EXIT_FAILURE;
    }

    const int nSpeedMode = 1; // 0 正常、1 慢速、2 快速、3 焊接
    ret = HRIF_SetFTFreeDriveSpeedMode(BOX_ID, RBT_ID, nSpeedMode);
    print_ret("HRIF_SetFTFreeDriveSpeedMode", ret);
    if (ret != 0) {
        print_ret("HRIF_DisConnect", HRIF_DisConnect(BOX_ID));
        return EXIT_FAILURE;
    }

    const double dMaxLinearVelocity = 10.0; // 最大速度 > 1 且 < 500
    const double dMaxAngularVelocity = 5.0; // 最大角速度 > 1 且 < 180
    ret = HRIF_SetMaxFreeDriveVel(BOX_ID, RBT_ID, dMaxLinearVelocity, dMaxAngularVelocity);
    print_ret("HRIF_SetMaxFreeDriveVel", ret);
    if (ret != 0) {
        print_ret("HRIF_DisConnect", HRIF_DisConnect(BOX_ID));
        return EXIT_FAILURE;
    }

    const double dLinear = 50.0; //柔顺度
    const double dAngular = 50.0; //柔顺度
    ret = HRIF_SetFTFreeFactor(BOX_ID, RBT_ID, dLinear, dAngular);
    print_ret("HRIF_SetFTFreeFactor", ret);
    if (ret != 0) {
        print_ret("HRIF_DisConnect", HRIF_DisConnect(BOX_ID));
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 5. 开启自由驱动
    // ----------------------------------------------------------
    ret = HRIF_SetForceFreeDriveMode(BOX_ID, RBT_ID, 1);
    print_ret("HRIF_SetForceFreeDriveMode(open)", ret);
    if (ret != 0) {
        print_ret("HRIF_DisConnect", HRIF_DisConnect(BOX_ID));
        return EXIT_FAILURE;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    int nForceState = -1;
    ret = HRIF_ReadForceControlState(BOX_ID, RBT_ID, nForceState);
    print_ret("HRIF_ReadForceControlState", ret);
    if (ret != 0 || nForceState != 3)
    {
        std::cerr << "[ERR] Expected free-drive state 3, actual=" << nForceState << std::endl;
        print_ret("HRIF_DisConnect", HRIF_DisConnect(BOX_ID));
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 6. 保持自由驱动 5 秒
    // ----------------------------------------------------------
    const int nDurationSeconds = 5;
    std::cout << "[INFO] Free drive enabled for " << nDurationSeconds
              << " seconds." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(nDurationSeconds));

    // ----------------------------------------------------------
    // 7. 关闭自由驱动，并断开连接
    // ----------------------------------------------------------
    ret = HRIF_SetForceFreeDriveMode(BOX_ID, RBT_ID, 0);
    print_ret("HRIF_SetForceFreeDriveMode(close)", ret);

    ret = HRIF_DisConnect(BOX_ID);
    print_ret("HRIF_DisConnect", ret);
    return ret == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
