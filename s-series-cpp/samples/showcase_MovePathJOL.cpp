#include "HR_Pro.h"

#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdlib>
#include <vector>

// BoxID 和 RbtID 可以在此处进行修改
static const unsigned int BOX_ID = 0;
static const unsigned int RBT_ID = 0;
static const int BOX_DO_BIT = 2;
static const int BOX_DO_MASK = 1 << BOX_DO_BIT;
static constexpr double PI = 3.14159265358979323846;

void print_ret(const std::string& cmd, int ret)
{
    if (ret == 0) {
        std::cout << "[OK]  " << cmd << std::endl;
    } else {
        std::string err_msg;
        const int lookupRet = HRIF_GetErrorCodeStr(BOX_ID, ret, err_msg);
        std::cout << "[ERR] " << cmd << " -> " << ret;
        if (lookupRet == 0) std::cout << ": " << err_msg;
        std::cout << std::endl;
    }
}

int main()
{
    std::cout << "=== MovePathJOL Online Path Example ===" << std::endl;

    // ----------------------------------------------------------
    // 1. 连接机器人
    // ----------------------------------------------------------
    // 需根据机器人的实际 IP 进行修改
    std::string host = "192.168.0.10";
    unsigned short port = 10003;
    int ret = HRIF_Connect(BOX_ID, host.c_str(), port);
    print_ret("HRIF_Connect", ret);
    if (ret != 0) return EXIT_FAILURE;

    // ----------------------------------------------------------
    // 2. 确认机器人处于 Standby(33) 状态
    // ----------------------------------------------------------
    int nFSM = 0;
    ret = HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, nFSM);
    print_ret("HRIF_ReadCurFSM (FSM=" + std::to_string(nFSM) + ")", ret);
    if (ret != 0 || nFSM != 33) {
        std::cerr << "[ERR] Robot must be in Standby(33), FSM=" << nFSM << std::endl;
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 3. 设置速度比，并运动到安全位置
    // ----------------------------------------------------------
    ret = HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
    print_ret("HRIF_SetOverride", ret);
    if (ret != 0) {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    double dJ1 = 0;  double dJ2 = -90;  double dJ3 = 90;
    double dJ4 = -90;  double dJ5 = -90; double dJ6 = 0;

    std::cout << "[INFO] Moving to a safe position with MoveJ..." << std::endl;
    ret = HRIF_MoveJ(BOX_ID, RBT_ID,
                     0, 0, 0, 0, 0, 0,
                     dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
                     "TCP", "Base",
                     30.0, 60.0, 0.0,
                     1, 0, 0, 0, "MovePathJOL_home");
    print_ret("HRIF_MoveJ", ret);
    if (ret != 0) {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    bool bDone = false;
    const auto motionDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ret = HRIF_IsMotionDone(BOX_ID, RBT_ID, bDone);
    } while (ret == 0 && !bDone && std::chrono::steady_clock::now() < motionDeadline);
    if (ret != 0 || !bDone) {
        if (ret != 0) print_ret("HRIF_IsMotionDone", ret);
        else std::cerr << "[ERR] MoveJ timed out" << std::endl;
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }
    std::cout << "[INFO] Safe position reached" << std::endl;


    // ----------------------------------------------------------
    // 4. 准备 10 个点，第 5 个点将 DO2 置高，第 10 个点将 DO2 置低
    // ----------------------------------------------------------
    std::vector<MovePathJOLPointsData> points(10);
    for (int i = 0; i < 10; ++i) {
        const double angle = 2.0 * PI * i / 9.0;
        points[i].dbJ1 = dJ1 + 1.5 * std::sin(angle);
        points[i].dbJ2 = dJ2 + 0.8 * (1.0 - std::cos(angle));
        points[i].dbJ3 = dJ3;
        points[i].dbJ4 = dJ4;
        points[i].dbJ5 = dJ5;
        points[i].dbJ6 = dJ6;
    }
    points[4].nIsSetIO = 1;
    points[4].nBoxDOMask = BOX_DO_MASK;
    points[4].nBoxDOVal = BOX_DO_MASK;

    points[9].nIsSetIO = 1;
    points[9].nBoxDOMask = BOX_DO_MASK;
    points[9].nBoxDOVal = 0;

    ret = HRIF_MovePathJOL(BOX_ID, RBT_ID, 20, 20, 20, points);
    print_ret("HRIF_MovePathJOL (10 points)", ret);
    if (ret != 0) {
        HRIF_GrpStop(BOX_ID, RBT_ID);
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 5. 第 5 个点执行完成后读取 DO2 状态
    // ----------------------------------------------------------
    int nCurIndex = 0;
    int nIndexNum = 0;
    auto pathDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ret = HRIF_GetMovePathJOLIndex(BOX_ID, RBT_ID, nCurIndex, nIndexNum);
    } while (ret == 0 && nCurIndex < 6 && std::chrono::steady_clock::now() < pathDeadline);
    if (ret != 0 || nCurIndex < 6) {
        if (ret != 0) print_ret("HRIF_GetMovePathJOLIndex", ret);
        else std::cerr << "[ERR] Wait for MovePathJOL point 5 completion timed out" << std::endl;
        HRIF_GrpStop(BOX_ID, RBT_ID);
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    int nDOVal = 0;
    ret = HRIF_ReadBoxDO(BOX_ID, BOX_DO_BIT, nDOVal);
    print_ret("HRIF_ReadBoxDO (point 5, DO2=" + std::to_string(nDOVal) + ")", ret);
    if (ret != 0) {
        HRIF_GrpStop(BOX_ID, RBT_ID);
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 6. 等待第 10 个点完成，并再次读取 DO2 状态
    // ----------------------------------------------------------
    pathDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ret = HRIF_GetMovePathJOLIndex(BOX_ID, RBT_ID, nCurIndex, nIndexNum);
    } while (ret == 0 && nCurIndex < 10 && std::chrono::steady_clock::now() < pathDeadline);
    if (ret != 0 || nCurIndex < 10) {
        if (ret != 0) print_ret("HRIF_GetMovePathJOLIndex", ret);
        else std::cerr << "[ERR] Wait for MovePathJOL point 10 timed out" << std::endl;
        HRIF_GrpStop(BOX_ID, RBT_ID);
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    bDone = false;
    const auto pathMotionDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ret = HRIF_IsMotionDone(BOX_ID, RBT_ID, bDone);
    } while (ret == 0 && !bDone && std::chrono::steady_clock::now() < pathMotionDeadline);
    if (ret != 0 || !bDone) {
        if (ret != 0) print_ret("HRIF_IsMotionDone", ret);
        else std::cerr << "[ERR] MovePathJOL completion timed out" << std::endl;
        HRIF_GrpStop(BOX_ID, RBT_ID);
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }
    ret = HRIF_ReadBoxDO(BOX_ID, BOX_DO_BIT, nDOVal);
    print_ret("HRIF_ReadBoxDO (completed, DO2=" + std::to_string(nDOVal) + ")", ret);

    // ----------------------------------------------------------
    // 7. 停止运动并断开连接
    // ----------------------------------------------------------
    std::cout << "[INFO] MovePathJOL completed; disconnecting" << std::endl;
    const int stop_ret = HRIF_GrpStop(BOX_ID, RBT_ID);
    print_ret("HRIF_GrpStop", stop_ret);
    const int disconnect_ret = HRIF_DisConnect(BOX_ID);
    print_ret("HRIF_DisConnect", disconnect_ret);
    return ret == 0 && stop_ret == 0 && disconnect_ret == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
