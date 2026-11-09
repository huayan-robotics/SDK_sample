#include "HR_Pro.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

// BoxID 和 RbtID 可以在此处进行修改
static const unsigned int BOX_ID = 0;
static const unsigned int RBT_ID = 0;

// Print an error code returned by an SDK call or by the controller.
// HRIF_GetErrorCodeStr may also fail, so append its description only on success.
// Always preserve the original code for lookup in the error-code reference.
static void printErrorCode(const std::string& source, int errorCode)
{
    if (errorCode == 0) return;

    std::string message;
    const int ret = HRIF_GetErrorCodeStr(BOX_ID, errorCode, message);
    std::cerr << "[ERROR] " << source << " -> " << errorCode;
    if (ret == 0) std::cerr << ": " << message;
    std::cerr << std::endl;
}

static const char* errorCategory(int errorCode)
{
    // Use the leading digit only for quick log grouping, never as a full diagnosis.
    switch (errorCode / 10000) {
    case 1: return "robot body";
    case 2: return "controller internal";
    case 3: return "external device communication";
    case 4: return "controller internal";
    case 6: return "specific function or device";
    default: return "other";
    }
}

static bool isErrorState(int state)
{
    // Handle only FSM states that explicitly represent errors or safety events.
    // Refer to the state-machine documentation for the active controller version.
    switch (state) {
    case 5:   // Emergency stop
    case 10:  // Safety I/O error
    case 12:  // Safeguard
    case 17:  // EtherCAT error
    case 21:  // Safety collision stop
    case 22:  // Robot error
    case 40:  // Plugin error
    case 52:  // Safe-motion error
        return true;
    default:
        return false;
    }
}

static bool readDiagnostics()
{
    // ReadRobotState provides an error summary and a motion, safety, and power snapshot.
    // Use these output parameters only when the call returns zero.
    int moving = 0, enabled = 0, errorState = 0, errorCode = 0, errorAxis = 0;
    int braking = 0, paused = 0, emergencyStop = 0, safeGuard = 0;
    int electrified = 0, connectedToBox = 0, blendingDone = 0, inPosition = 0;
    int ret = HRIF_ReadRobotState(
        BOX_ID, RBT_ID, moving, enabled, errorState, errorCode, errorAxis,
        braking, paused, emergencyStop, safeGuard, electrified,
        connectedToBox, blendingDone, inPosition);
    if (ret != 0) {
        printErrorCode("HRIF_ReadRobotState", ret);
        return false;
    }

    std::cout << "[STATE] error=" << errorState
              << ", code=" << errorCode
              << ", category=" << errorCategory(errorCode)
              << ", axis=" << errorAxis
              << ", emergencyStop=" << emergencyStop
              << ", safeGuard=" << safeGuard << std::endl;
    printErrorCode("controller", errorCode);

    // errorAxis identifies the related axis; per-axis codes provide fuller diagnostics.
    int controllerCode = 0;
    int axisCode[6] = {0, 0, 0, 0, 0, 0};
    ret = HRIF_ReadAxisErrorCode(
        BOX_ID, RBT_ID, controllerCode,
        axisCode[0], axisCode[1], axisCode[2],
        axisCode[3], axisCode[4], axisCode[5]);
    if (ret != 0) {
        printErrorCode("HRIF_ReadAxisErrorCode", ret);
        return false;
    }

    printErrorCode("axis group", controllerCode);
    for (int i = 0; i < 6; ++i) {
        if (axisCode[i] != 0) printErrorCode("J" + std::to_string(i + 1), axisCode[i]);
    }

    // A safety event may come from a physical signal or a dual-channel mismatch.
    // A reset request cannot clear an active emergency-stop or safeguard input.
    int emergencyCircuitError = 0, emergencySignal = 0;
    int safeguardCircuitError = 0, safeguardSignal = 0;
    ret = HRIF_ReadEmergencyInfo(
        BOX_ID, RBT_ID, emergencyCircuitError, emergencySignal,
        safeguardCircuitError, safeguardSignal);
    if (ret != 0) {
        printErrorCode("HRIF_ReadEmergencyInfo", ret);
        return false;
    }

    std::cout << "[SAFETY] emergencyCircuitError=" << emergencyCircuitError
              << ", emergencySignal=" << emergencySignal
              << ", safeguardCircuitError=" << safeguardCircuitError
              << ", safeguardSignal=" << safeguardSignal << std::endl;
    return true;
}

int main()
{
    // This example does not create faults. Run it only for an existing, safely recoverable alarm.

    // ----------------------------------------------------------
    // 1. 连接机器人
    // ----------------------------------------------------------
    // 需根据机器人的实际 IP 进行修改
    const std::string host = "192.168.0.10";
    const unsigned short port = 10003;

    int ret = HRIF_Connect(BOX_ID, host.c_str(), port);
    if (ret != 0) {
        std::cerr << "[ERROR] HRIF_Connect -> " << ret << std::endl;
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 2. 读取状态机并判断是否需要复位
    // ----------------------------------------------------------
    int state = -1;
    ret = HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, state);
    if (ret != 0) {
        printErrorCode("HRIF_ReadCurFSM", ret);
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    std::cout << "[FSM] " << state << std::endl;
    if (!isErrorState(state)) {
        std::cout << "[INFO] The state machine is not in an error state handled by this example; "
                  << "no reset is required." << std::endl;
        HRIF_DisConnect(BOX_ID);
        return EXIT_SUCCESS;
    }

    // ----------------------------------------------------------
    // 3. 读取错误及安全诊断信息
    // ----------------------------------------------------------
    if (!readDiagnostics()) {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 4. 确认故障原因已排除
    // ----------------------------------------------------------
    std::cout << "Clear the emergency stop, safeguard, collision, communication, or "
              << "equipment fault indicated by the diagnostics." << std::endl;
    std::cout << "After confirming site safety and clearing the cause, "
              << "press Enter to request one reset..." << std::endl;
    std::cin.get();

    // ----------------------------------------------------------
    // 5. 请求复位机器人
    // ----------------------------------------------------------
    ret = HRIF_GrpReset(BOX_ID, RBT_ID);
    if (ret != 0) {
        printErrorCode("HRIF_GrpReset", ret);
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 6. 轮询状态机并确认复位结果
    // ----------------------------------------------------------
    const std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
        ret = HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, state);
        if (ret != 0) break;
        std::cout << "[FSM] " << state << std::endl;
        if (!isErrorState(state)) {
            std::cout << "[OK] The state machine has left the error state. "
                      << "Resume operation through the state-machine flow." << std::endl;
            HRIF_DisConnect(BOX_ID);
            return EXIT_SUCCESS;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    if (ret != 0) printErrorCode("HRIF_ReadCurFSM", ret);
    else {
        std::cerr << "[ERROR] The state machine remains in an error state after reset; "
                  << "preserve the logs and escalate for manual handling." << std::endl;
    }
    HRIF_DisConnect(BOX_ID);
    return EXIT_FAILURE;
}
