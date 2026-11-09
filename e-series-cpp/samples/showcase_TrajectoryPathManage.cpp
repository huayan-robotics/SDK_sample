#include "HR_Pro.h"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {
// BoxID 和 RbtID 可以在此处进行修改
const unsigned int BOX_ID = 0;
const unsigned int RBT_ID = 0;
const std::string ORIGINAL_NAME = "sdk_manage_path_temp";
const std::string RENAMED_NAME = "sdk_manage_path_renamed";

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
}  // namespace

int main() {
    std::cout << "=== Trajectory Path Management Example ===" << std::endl;

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
    std::string activeName;
    int fsm = 0;
    ret = HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, fsm);
    if (!printRet("HRIF_ReadCurFSMFromCPS", ret)) {
        ok = false;
    } else if (fsm != 33) {
        std::cerr << "[ERR] Robot must be in Standby(33), FSM=" << fsm << std::endl;
        ok = false;
    }

    // ----------------------------------------------------------
    // 2. 确认临时名称未被占用，再创建专用于本案例的轨迹对象
    // ----------------------------------------------------------
    if (ok) {
        std::string initialPaths;
        ret = HRIF_ReadPathList(BOX_ID, RBT_ID, initialPaths);
        ok = printRet("HRIF_ReadPathList (before create)", ret);
        if (ok && (initialPaths.find(ORIGINAL_NAME) != std::string::npos ||
                   initialPaths.find(RENAMED_NAME) != std::string::npos)) {
            std::cerr << "[ERR] Temporary trajectory name is already in use" << std::endl;
            ok = false;
        }
    }
    if (ok) {
        ret = HRIF_InitPath(
            BOX_ID, RBT_ID, 0, ORIGINAL_NAME, 0.1, 5.0,
            30.0, 60.0, 500.0, "Base", "TCP"
        );
        ok = printRet("HRIF_InitPath", ret);
        if (ok) {
            activeName = ORIGINAL_NAME;
        }
    }

    // ----------------------------------------------------------
    // 3. 读取创建后的轨迹列表和临时轨迹详细信息
    // ----------------------------------------------------------
    if (ok) {
        std::string pathList;
        ret = HRIF_ReadPathList(BOX_ID, RBT_ID, pathList);
        ok = printRet("HRIF_ReadPathList", ret);
        if (ok) {
            std::cout << "[INFO] Path list: " << pathList << std::endl;
        }
    }
    if (ok) {
        std::string pathInfo;
        ret = HRIF_ReadPathInfo(BOX_ID, RBT_ID, ORIGINAL_NAME, pathInfo);
        ok = printRet("HRIF_ReadPathInfo", ret);
        if (ok) {
            std::cout << "[INFO] " << ORIGINAL_NAME << " info: " << pathInfo << std::endl;
        }
    }

    // ----------------------------------------------------------
    // 4. 重命名轨迹并读取新名称下的信息
    // ----------------------------------------------------------
    if (ok) {
        ret = HRIF_UpdatePathName(BOX_ID, RBT_ID, ORIGINAL_NAME, RENAMED_NAME);
        ok = printRet("HRIF_UpdatePathName", ret);
        if (ok) {
            activeName = RENAMED_NAME;
        }
    }
    if (ok) {
        std::string pathInfo;
        ret = HRIF_ReadPathInfo(BOX_ID, RBT_ID, RENAMED_NAME, pathInfo);
        ok = printRet("HRIF_ReadPathInfo (renamed)", ret);
        if (ok) {
            std::cout << "[INFO] " << RENAMED_NAME << " info: " << pathInfo << std::endl;
        }
    }

    // ----------------------------------------------------------
    // 5. 删除临时轨迹并断开连接
    // ----------------------------------------------------------
    if (ok) {
        ret = HRIF_DelPath(BOX_ID, RBT_ID, RENAMED_NAME);
        ok = printRet("HRIF_DelPath", ret);
        if (ok) {
            activeName.clear();
        }
    }
    if (!activeName.empty()) {
        const int deleteRet = HRIF_DelPath(BOX_ID, RBT_ID, activeName);
        if (!printRet("HRIF_DelPath (cleanup)", deleteRet)) {
            ok = false;
        }
    }
    const int disconnectRet = HRIF_DisConnect(BOX_ID);
    if (!printRet("HRIF_DisConnect", disconnectRet)) {
        ok = false;
    }
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
