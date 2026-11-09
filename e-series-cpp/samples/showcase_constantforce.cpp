#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "HR_Pro.h"

// BoxID 和 RbtID 可以在此处修改
static const unsigned int BOX_ID = 0;
static const unsigned int RBT_ID = 0;
enum
{
    FORCE_CONTROL_STATE_CLOSED = 0,
    FORCE_CONTROL_STATE_SEARCH = 1,
    FORCE_CONTROL_STATE_FOLLOW = 2,
    FORCE_CONTROL_STATE_FREEDRIVE = 3
};

bool print_ret(const std::string& cmd, int ret)
{
    if (ret == 0)
    {
        std::cout << "[OK]  " << cmd << std::endl;
        return true;
    }

    std::string errMsg;
    const int lookupRet = HRIF_GetErrorCodeStr(BOX_ID, ret, errMsg);

    std::cerr << "[ERR] " << cmd << " -> " << ret;
    if (lookupRet == 0)
    {
        std::cerr << ": " << errMsg;
    }
    std::cerr << std::endl;

    return false;
}

bool wait_blending_done()
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    while (std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        bool done = false;
        const int ret = HRIF_IsBlendingDone(BOX_ID, RBT_ID, done);
        if (ret != 0)
        {
            return false;
        }
        if (done)
        {
            std::cout << "[ok] Motion Done " << std::endl;
            return true;
        }
    }
    std::cerr << "[ERR] Motion timeout (60s)." << std::endl;
    return false;
}

int main()
{
    std::cout << "======== Constant Force Control Example ========"
              << std::endl;

    // ----------------------------------------------------------
    // 1. 连接机器人
    // ----------------------------------------------------------
    const std::string host = "192.168.0.10";
    const unsigned int port = 10003;

    int ret = HRIF_Connect(BOX_ID, host.c_str(), port);
    if (!print_ret("HRIF_Connect", ret))
    {
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 2. 确认机器人处于 Standby(33)，并确认力控关闭
    // ----------------------------------------------------------
    int nFSM = 0;
    ret = HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, nFSM);
    if (!print_ret("HRIF_ReadCurFSMFromCPS", ret) || nFSM != 33)
    {
        std::cerr << "[ERR] Robot must be in Standby(33), FSM=" << nFSM << std::endl;
        return EXIT_FAILURE;
    }

    int nForceState = -1;
    ret = HRIF_ReadForceControlState(BOX_ID, RBT_ID, nForceState);
    if (!print_ret("HRIF_ReadForceControlState", ret) || nForceState != 0)
    {
        std::cerr << "[ERR] Force control must be closed."
                  << std::endl;
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 3. 设置低速倍率，运动到现场确认的安全探寻起点
    // ----------------------------------------------------------
    ret = HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
    if (!print_ret("HRIF_SetOverride", ret))
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }
    //可以替换为开始探寻的起点
    double dJ1 = 0; double dJ2 = 0; double dJ3 = 90;
    double dJ4 = 0; double dJ5 = 90; double dJ6 = 0;

    ret = HRIF_MoveJ(BOX_ID, RBT_ID, 0, 0, 0, 0, 0, 0, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, 
                    "TCP", "Base", 10.0, 20.0, 0.0, 1, 0, 0, 0, "force_search_start");
    if (!print_ret("HRIF_MoveJ(search start)", ret) || !wait_blending_done())
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 4. 选择恒力模式
    // ----------------------------------------------------------
    const int nStrategy = 0; //恒力控制模式
    ret = HRIF_SetForceControlStrategy(BOX_ID, RBT_ID, nStrategy);
    if (!print_ret("HRIF_SetForceControlStrategy", ret))
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 5. 仅开启 Z 方向自由度，设置目标力和异常受力限制
    // ----------------------------------------------------------
    ret = HRIF_SetControlFreedom(BOX_ID, RBT_ID, 0, 0, 1, 0, 0, 0);
    if (!print_ret("HRIF_SetControlFreedom", ret))
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    const double dTargetFz = -10.0;
    ret = HRIF_SetForceControlGoal(BOX_ID, RBT_ID, 0, 0, dTargetFz, 0, 0, 0);
    if (!print_ret("HRIF_SetForceControlGoal", ret))
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    ret = HRIF_SetForceDataLimit(BOX_ID, RBT_ID, 500, 500, 500, 50, 50, 50, 300, 300, 300, 30, 30, 30);
    if (!print_ret("HRIF_SetForceDataLimit", ret))
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 6. 设置探寻速度、探寻边界和跟随边界
    // ----------------------------------------------------------
    ret = HRIF_SetMaxSearchVelocities(BOX_ID, RBT_ID, 5.0, 1.0);
    if (!print_ret("HRIF_SetMaxSearchVelocities", ret))
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    ret = HRIF_SetMaxSearchDistance(BOX_ID, RBT_ID, 100, 100, 100, 20, 20, 20);
    if (!print_ret("HRIF_SetMaxSearchDistance", ret))
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    ret = HRIF_SetSteadyContactDeviationRange( BOX_ID, RBT_ID, 300, 300, 300, 20, 20, 20, -100, -100, -100, -20, -20, -20);
    if (!print_ret("HRIF_SetSteadyContactDeviationRange", ret))
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 7. 设置动态响应参数
    // ----------------------------------------------------------
    // 设置力和力矩控制的 质量/惯量参数和阻尼参数（此处设置为推荐的默认值）
    ret = HRIF_SetMassParams(BOX_ID, RBT_ID, 40.0, 40.0, 40.0, 10.0, 10.0, 10.0);
    if (!print_ret("HRIF_SetMassParams", ret))
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    ret = HRIF_SetDampParams(BOX_ID, RBT_ID, 800.0, 800.0, 800.0, 40.0, 40.0, 40.0);
    if (!print_ret("HRIF_SetDampParams", ret))
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }
    // ----------------------------------------------------------
    // 8. 开启力控，等待探寻完成
    // ----------------------------------------------------------
    ret = HRIF_SetForceControlState(BOX_ID, RBT_ID, 1);
    if (!print_ret("HRIF_SetForceControlState(open)", ret))
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    int state = -1;
    bool timeout = true;
    while (std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        ret =  HRIF_ReadForceControlState(BOX_ID, RBT_ID, state);
        if (ret != 0)
        {
            break;
        }
        if (state == FORCE_CONTROL_STATE_FOLLOW) // 代表力控探寻完成状态
        {
            std::cout << "[OK] Force state = " << state << std::endl;
            timeout = false;
            break;
        }
        
    }
    
    if( ret !=0 || timeout)
    {
        std::cerr << "[ERR] Expected state !=" << FORCE_CONTROL_STATE_FOLLOW << ", actual=" << state << std::endl;
        ret = HRIF_SetForceControlState(BOX_ID, RBT_ID, 0);
        print_ret("HRIF_SetForceControlState",ret);
        ret = HRIF_DisConnect(BOX_ID);
        print_ret("HRIF_DisConnect",ret);
        return EXIT_FAILURE;
    }
    // ----------------------------------------------------------
    // 9. 力控探寻完成状态后，沿 UCS X 正方向作空间直线运动
    // ----------------------------------------------------------
    double x = 0.0, y = 0.0, z = 0.0;
    double Rx = 0.0, Ry = 0.0, Rz = 0.0;
    double J1 = 0.0, J2 = 0.0, J3 = 0.0;
    double J4 = 0.0, J5 = 0.0, J6 = 0.0;
    ret = HRIF_ReadActCoord(BOX_ID, RBT_ID, "Base", "TCP", J1, J2, J3, J4, J5, J6,
                            x, y, z, Rx, Ry, Rz);
    if (!print_ret("HRIF_ReadActCoord", ret))
    {
        HRIF_SetForceControlState(BOX_ID, RBT_ID, 0);
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    ret = HRIF_MoveL(BOX_ID, RBT_ID, x, y + 50.0 , z, Rx, Ry, Rz, J1, J2, J3, J4, J5, J6, 
                    "TCP", "Base", 10.0, 20.0, 0.0, 0, 0, 0, "force_follow_x");
    if (!print_ret("HRIF_MoveL(Y +50 mm)", ret) || !wait_blending_done())
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 10. 关闭力控，断开连接
    // ----------------------------------------------------------
    ret = HRIF_SetForceControlState(BOX_ID, RBT_ID, 0);
    if (!print_ret("HRIF_SetForceControlState(close)", ret))
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    std::cout << "[OK] Constant force control completed." << std::endl;
    ret = HRIF_DisConnect(BOX_ID);
    print_ret("HRIF_DisConnect", ret);
    return ret == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}