#include "HR_Pro.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {
// BoxID 和 RbtID 可以在此处进行修改
const unsigned int BOX_ID = 0;
const unsigned int RBT_ID = 0;
const std::string PATH_NAME = "sdk_relative_path_j";
const double PI = 3.14159265358979323846;

bool printRet(const std::string& command, int ret) {
    if (ret == 0) {
        std::cout << "[OK]  " << command << std::endl;
        return true;
    }

    std::string message;
    const int lookupRet = HRIF_GetErrorCodeStr(BOX_ID, ret, message);
    std::cerr << "[ERR] " << command << " -> " << ret;
    if (lookupRet == 0) {
        std::cerr << ": " << message;
    }
    std::cerr << std::endl;
    return false;
}

bool waitMotion(std::chrono::seconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        bool done = false;
        const int ret = HRIF_IsMotionDone(BOX_ID, RBT_ID, done);
        if (ret != 0) {
            printRet("HRIF_IsMotionDone", ret);
            return false;
        }
        if (done) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cerr << "[ERR] Motion timed out" << std::endl;
    return false;
}

bool waitPathComputed() {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::minutes(10);
    int lastState = -1;
    while (std::chrono::steady_clock::now() < deadline) {
        int stateJ = 0;
        int errorJ = 0;
        int stateL = 0;
        int errorL = 0;
        const int ret = HRIF_ReadPathState(
            BOX_ID, RBT_ID, PATH_NAME, stateJ, errorJ, stateL, errorL
        );
        if (ret != 0) {
            printRet("HRIF_ReadPathState", ret);
            return false;
        }
        if (stateJ != lastState) {
            std::cout << "[INFO] PathJ state=" << stateJ << std::endl;
            lastState = stateJ;
        }
        if (stateJ == 3) {
            return true;
        }
        if (stateJ == 5) {
            std::cerr << "[ERR] PathJ computation failed, error=" << errorJ << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cerr << "[ERR] PathJ computation timed out" << std::endl;
    return false;
}

std::vector<double> buildRelativeCurve(const double base[6]) {
    std::vector<double> points;
    const int pointCount = 13;
    points.reserve(pointCount * 6);
    for (int index = 0; index < pointCount; ++index) {
        const double phase = -0.5 * PI + 2.0 * PI * index / (pointCount - 1);
        const double point[6] = {
            base[0] + 5.0 * std::sin(phase),
            base[1] + 3.0 * (1.0 - std::cos(phase)),
            base[2], base[3], base[4], base[5]
        };
        points.insert(points.end(), point, point + 6);
    }
    return points;
}

std::string serializePoints(const std::vector<double>& points) {
    std::ostringstream stream;
    for (std::size_t index = 0; index < points.size(); ++index) {
        if (index != 0) {
            stream << ',';
        }
        stream << points[index];
    }
    return stream.str();
}
}  // namespace

int main() {
    std::cout << "=== PathJ Relative Curve Example ===" << std::endl;

    // ----------------------------------------------------------
    // 1. 连接机器人并确认处于 Standby(33)
    // ----------------------------------------------------------
    // 需根据机器人的实际 IP 进行修改
    const std::string host = "192.168.0.10";
    int ret = HRIF_Connect(BOX_ID, host.c_str(), 10003);
    if (!printRet("HRIF_Connect", ret)) {
        return EXIT_FAILURE;
    }

    bool ok = true;
    bool motionStarted = false;
    int fsm = 0;
    ret = HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, fsm);
    if (!printRet("HRIF_ReadCurFSMFromCPS", ret)) {
        ok = false;
    } else if (fsm != 33) {
        std::cerr << "[ERR] Robot must be in Standby(33), FSM=" << fsm << std::endl;
        ok = false;
    }

    // ----------------------------------------------------------
    // 2. 设置速度比为0.2，运动到安全基准位姿，并以实际到达位置作为相对点位基准
    // ----------------------------------------------------------
    ret = HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
    printRet("HRIF_SetOverride", ret);
    if (ret != 0) {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    double dJ1 = 0;  double dJ2 = -90;  double dJ3 = 90;
    double dJ4 = -90;  double dJ5 = -90; double dJ6 = 0;
    if (ok) {
        ret = HRIF_MoveJ(
            BOX_ID, RBT_ID,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
            "TCP", "Base", 20.0, 40.0, 0.0, 1, 0, 0, 0, "path_j_home"
        );
        ok = printRet("HRIF_MoveJ (safe pose)", ret);
        motionStarted = ok;
    }
    if (ok) {
        ok = waitMotion(std::chrono::seconds(120));
    }

    double base[6] = {};
    if (ok) {
        ret = HRIF_ReadActJointPos(
            BOX_ID, RBT_ID, base[0], base[1], base[2], base[3], base[4], base[5]
        );
        ok = printRet("HRIF_ReadActJointPos", ret);
    }

    // ----------------------------------------------------------
    // 3. 创建 PathJ，推送 ACS 原始点位并等待轨迹计算完成
    // ----------------------------------------------------------
    std::vector<double> points;
    if (ok) {
        points = buildRelativeCurve(base);
        ret = HRIF_InitPath(
            BOX_ID, RBT_ID, 0, PATH_NAME, 0.15, 5.0,
            30.0, 60.0, 500.0, "Base", "TCP"
        );
        ok = printRet("HRIF_InitPath", ret);
    }
    if (ok) {
        ret = HRIF_PushPathPoints(BOX_ID, RBT_ID, PATH_NAME, serializePoints(points));
        ok = printRet("HRIF_PushPathPoints", ret);
    }
    if (ok) {
        ret = HRIF_EndPushPathPoints(BOX_ID, RBT_ID, PATH_NAME);
        ok = printRet("HRIF_EndPushPathPoints", ret);
    }
    if (ok) {
        ok = waitPathComputed();
    }

    // ----------------------------------------------------------
    // 4. 先运动到轨迹初始点 P0，再运行 PathJ
    // ----------------------------------------------------------
    if (ok) {
        ret = HRIF_MoveJ(
            BOX_ID, RBT_ID,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            points[0], points[1], points[2], points[3], points[4], points[5],
            "TCP", "Base", 20.0, 40.0, 0.0, 1, 0, 0, 0, "path_j_start"
        );
        ok = printRet("HRIF_MoveJ (P0)", ret);
    }
    if (ok) {
        ok = waitMotion(std::chrono::seconds(120));
    }
    if (ok) {
        ret = HRIF_MovePathJ(BOX_ID, RBT_ID, PATH_NAME);
        ok = printRet("HRIF_MovePathJ", ret);
    }
    if (ok) {
        ok = waitMotion(std::chrono::seconds(120));
    }

    if (motionStarted) {
        const int stopRet = HRIF_GrpStop(BOX_ID, RBT_ID);
        if (!printRet("HRIF_GrpStop", stopRet)) {
            ok = false;
        }
    }
    const int disconnectRet = HRIF_DisConnect(BOX_ID);
    if (!printRet("HRIF_DisConnect", disconnectRet)) {
        ok = false;
    }
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
