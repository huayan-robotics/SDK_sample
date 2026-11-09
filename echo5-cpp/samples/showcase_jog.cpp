#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <string>

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
    std::cout << "======== Jogging example started. ========" << std::endl;

    // ----------------------------------------------------------
    // 1. 连接机器人，需根据机器人的实际 IP 进行修改
    // ----------------------------------------------------------
    const string host = "192.168.0.10";
    const unsigned short port = 10003;

    int ret = HRIF_Connect(BOX_ID, host.c_str(), port);
    print_ret("HRIF_Connect", ret);
    if (ret != 0)
    {
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 2. 确认机器人处于 Standby(33)就绪状态
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
    safeJoints.dJoint[0] = 90;   safeJoints.dJoint[1] = 45;
    safeJoints.dJoint[2] = 0;     safeJoints.dJoint[3] = -80;
    safeJoints.dJoint[4] = 0;     safeJoints.dJoint[5] = 35;
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
    // 5. 进行关节短点动，J1轴反向运动 10°
    // ----------------------------------------------------------
    unsigned short nAxisID = 2;         // 指定轴转动，0~6 对应 J1~J7
    unsigned short nDirection = 0;      // 指定运动方向 0：反向 1：正向

    for (int i = 0; i < 5; i++)
    {
        ret = HRIF_ShortJogJ(BOX_ID, RBT_ID, nAxisID, nDirection);
        print_ret("HRIF_ShortJogJ", ret);
        if (ret != 0 || !wait_motion_done())
        {
            HRIF_DisConnect(BOX_ID);
            return EXIT_FAILURE;
        }
    }

    // ----------------------------------------------------------
    // 6. 进行空间短点动，X轴反向运动 20 mm
    // ----------------------------------------------------------
    nAxisID = 0;    //指定空间偏移  0~5 对应 X/Y/Z/RX/RY/RZ
    for (int i = 0; i < 10; i++)
    {
        ret = HRIF_ShortJogL(BOX_ID, RBT_ID, nAxisID, nDirection);
        print_ret("HRIF_ShortJogL", ret);
        if (ret != 0 || !wait_motion_done())
        {
            HRIF_DisConnect(BOX_ID);
            return EXIT_FAILURE;
        }
    }

    // ----------------------------------------------------------
    // 7. 进行空间长点动，X轴反向运动；停止保活后等待控制器自动停止
    // ----------------------------------------------------------
    unsigned short nState = 1; // 指定长点动运动状态 0：关闭 1：开启

    ret = HRIF_LongJogL(BOX_ID, RBT_ID, nAxisID, nDirection, nState);
    print_ret("HRIF_LongJogL", ret);
    if (ret != 0)
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }
    // 持续发送 HRIF_LongMoveEvent 维持长点动，采用 100 ms 周期
    for (int i = 0; i < 5; i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ret = HRIF_LongMoveEvent(BOX_ID, RBT_ID);
        i < 4 ? print_ret("HRIF_LongMoveEvent", ret) : print_ret("HRIF_LongMoveEvent(stop)", ret);
        if (ret != 0)
        {
            HRIF_DisConnect(BOX_ID);
            return EXIT_FAILURE;
        }
    }
    // 等待最后一次HRIF_LongMoveEvent完成
    if (!wait_motion_done())
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 8. 进行关节长点动，J4轴，按照启动长点动-持续下发机制-主动发送 nStat=0 的长点动命令停止
    // ----------------------------------------------------------
    nAxisID = 3;    // 指定点动的轴
    nDirection = 1; // 指定运动方向 0：反向 1：正向

    ret = HRIF_LongJogJ(BOX_ID, RBT_ID, nAxisID, nDirection, nState);
    print_ret("HRIF_LongJogJ", ret);
    if (ret != 0)
    {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // 持续发送 HRIF_LongMoveEvent 维持长点动，采用 100 ms 周期
    auto tStart = std::chrono::steady_clock::now();
    while (true)
    {
        ret = HRIF_LongMoveEvent(BOX_ID, RBT_ID);
        print_ret("HRIF_LongMoveEvent", ret);
        if (ret != 0)
        {
            HRIF_DisConnect(BOX_ID);
            return EXIT_FAILURE;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        auto tNow = std::chrono::steady_clock::now();
        double totalTime = std::chrono::duration<double>(tNow - tStart).count();
        if (totalTime > 2.5)
        {
            nState = 0;
            ret = HRIF_LongJogJ(BOX_ID, RBT_ID, nAxisID, nDirection, nState);
            print_ret("HRIF_LongJogJ(stop)", ret);
            if (ret != 0||!wait_motion_done())
            {
                HRIF_DisConnect(BOX_ID);
                return EXIT_FAILURE;
            }
            break;
        }
    }

    // ----------------------------------------------------------
    // 9. 断开连接
    // ----------------------------------------------------------
    std::cout << "[OK] Jogging Motion Completed."<< std::endl;
    ret = HRIF_DisConnect(BOX_ID);
    print_ret("HRIF_DisConnect", ret);
    return ret == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
