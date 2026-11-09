#include "HR_Pro.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

// BoxID 和 RbtID 可以在此处进行修改
static const unsigned int BOX_ID = 0;
static const unsigned int RBT_ID = 0;
static const int TRANSITION_TIMEOUT_SECONDS = 60;

// This is intentionally not a complete controller FSM definition.
enum class FsmState {
    BoxDisconnected = 2,
    EmergencyStopped = 5,
    Blackout = 7,
    SafetyIoError = 10,
    Safeguarding = 12,
    ControllerDisconnected = 14,
    ControllerVersionError = 16,
    EtherCatError = 17,
    ManualGuiding = 20,
    CollisionStop = 21,
    RobotError = 22,
    Disabled = 24,
    Moving = 25,
    LongJogMoving = 26,
    FreeDrive = 31,
    Paused = 32,
    Standby = 33,
    ProgramRunning = 34,
    ProgramHeld = 36,
    PluginDisconnected = 39,
    PluginError = 40,
    LowTemperature = 43,
    SafeMoveError = 52
};

static bool printRet(const std::string& command, int ret)
{
    if (ret == 0) {
        std::cout << "[OK]   " << command << std::endl;
        return true;
    }
    if (ret == 39500) {
        std::cerr << "[ERR]  " << command << " -> 39500: Robot not connected" << std::endl;
        return false;
    }

    std::string message;
    const int lookupRet = HRIF_GetErrorCodeStr(BOX_ID, ret, message);
    std::cerr << "[ERR]  " << command << " -> " << ret;
    if (lookupRet == 0) std::cerr << ": " << message;
    std::cerr << std::endl;
    return false;
}

static bool readState(int& state)
{
    const int ret = HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, state);
    if (ret == 0) return true;

    printRet("HRIF_ReadCurFSMFromCPS", ret);
    std::cerr << "[DIAG] State read failed; no transition decision was made" << std::endl;
    return false;
}

static bool reportAbnormalState(int state)
{
    const char* guidance = NULL;
    switch (static_cast<FsmState>(state)) {
    case FsmState::EtherCatError:
    case FsmState::RobotError:
    case FsmState::PluginError:
    case FsmState::SafeMoveError:
        guidance = "controller error; diagnose the cause before requesting Reset";
        break;
    case FsmState::CollisionStop:
        guidance = "collision stop; inspect the robot and work area before requesting Reset";
        break;
    case FsmState::EmergencyStopped:
    case FsmState::SafetyIoError:
    case FsmState::Safeguarding:
        guidance = "safety stop; restore and verify the external safety circuit first";
        break;
    case FsmState::ControllerVersionError:
    case FsmState::PluginDisconnected:
    case FsmState::LowTemperature:
        guidance = "maintenance condition; operator or service handling is required";
        break;
    default:
        return false;
    }

    std::cerr << "[ERR]  FSM=" << state << ": " << guidance << std::endl;
    return true;
}

static bool isBusinessActive(int state)
{
    switch (static_cast<FsmState>(state)) {
    case FsmState::Moving:
    case FsmState::LongJogMoving:
    case FsmState::Paused:
    case FsmState::ProgramRunning:
    case FsmState::ProgramHeld:
        return true;
    default:
        return false;
    }
}

static bool startRobot()
{
    const std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(TRANSITION_TIMEOUT_SECONDS);
    int previousState = -1;

    while (std::chrono::steady_clock::now() < deadline) {
        int state = -1;
        if (!readState(state)) return false;
        if (state == static_cast<int>(FsmState::Standby)) {
            std::cout << "[READY] Robot reached Standby(33)" << std::endl;
            return true;
        }
        if (reportAbnormalState(state)) return false;

        if (state != previousState) {
            bool actionRequired = true;
            bool actionSucceeded = false;
            switch (static_cast<FsmState>(state)) {
            case FsmState::BoxDisconnected:
                actionSucceeded = printRet("HRIF_Connect2Box", HRIF_Connect2Box(BOX_ID));
                break;
            case FsmState::Blackout:
                actionSucceeded = printRet("HRIF_Electrify", HRIF_Electrify(BOX_ID));
                break;
            case FsmState::ControllerDisconnected:
                actionSucceeded = printRet(
                    "HRIF_Connect2Controller", HRIF_Connect2Controller(BOX_ID));
                break;
            case FsmState::Disabled:
                actionSucceeded = printRet(
                    "HRIF_GrpEnable", HRIF_GrpEnable(BOX_ID, RBT_ID));
                break;
            case FsmState::ManualGuiding:
                actionSucceeded = printRet("HRIF_MoveToSS", HRIF_MoveToSS(BOX_ID, RBT_ID));
                break;
            case FsmState::FreeDrive:
                actionSucceeded = printRet(
                    "HRIF_GrpCloseFreeDriver", HRIF_GrpCloseFreeDriver(BOX_ID, RBT_ID));
                break;
            default:
                // Wait for the controller to complete the current transition.
                actionRequired = false;
                break;
            }
            if (actionRequired && !actionSucceeded) {
                std::cerr << "[DIAG] Startup transition failed at FSM=" << state << std::endl;
                return false;
            }
        }

        previousState = state;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cerr << "[ERR]  Robot did not reach Standby(33) within "
              << TRANSITION_TIMEOUT_SECONDS << " seconds" << std::endl;
    return false;
}

static bool shutdownRobot()
{
    const std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(TRANSITION_TIMEOUT_SECONDS);
    int previousState = -1;

    while (std::chrono::steady_clock::now() < deadline) {
        int state = -1;
        if (!readState(state)) return false;
        if (state == static_cast<int>(FsmState::Blackout)) {
            std::cout << "[SAFE]  Robot reached Blackout(7)" << std::endl;
            return true;
        }
        if (reportAbnormalState(state)) return false;

        if (state != previousState) {
            bool actionRequired = true;
            bool actionSucceeded = false;
            if (isBusinessActive(state)) {
                actionSucceeded = printRet("HRIF_GrpStop", HRIF_GrpStop(BOX_ID, RBT_ID));
            } else {
                switch (static_cast<FsmState>(state)) {
                case FsmState::Standby:
                    actionSucceeded = printRet(
                        "HRIF_GrpDisable", HRIF_GrpDisable(BOX_ID, RBT_ID));
                    break;
                case FsmState::Disabled:
                case FsmState::ControllerDisconnected:
                    actionSucceeded = printRet("HRIF_Blackout", HRIF_Blackout(BOX_ID));
                    break;
                case FsmState::ManualGuiding:
                    actionSucceeded = printRet("HRIF_MoveToSS", HRIF_MoveToSS(BOX_ID, RBT_ID));
                    break;
                case FsmState::FreeDrive:
                    actionSucceeded = printRet(
                        "HRIF_GrpCloseFreeDriver", HRIF_GrpCloseFreeDriver(BOX_ID, RBT_ID));
                    break;
                default:
                    // Wait for the controller to complete the current transition.
                    actionRequired = false;
                    break;
                }
            }
            if (actionRequired && !actionSucceeded) {
                std::cerr << "[DIAG] Shutdown transition failed at FSM=" << state << std::endl;
                return false;
            }
        }

        previousState = state;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cerr << "[ERR]  Robot did not reach Blackout(7) within "
              << TRANSITION_TIMEOUT_SECONDS << " seconds" << std::endl;
    return false;
}

static bool runBusinessTask()
{
    std::cout << "[BUSINESS] Execute the application task here" << std::endl;
    return true;
}

int main()
{
    // ----------------------------------------------------------
    // 1. Connect：连接机器人
    // ----------------------------------------------------------
    // 需根据机器人的实际 IP 进行修改
    const std::string host = "192.168.0.10";
    const unsigned short port = 10003;

    std::cout << "=== Controller State Machine Workflow Example ===" << std::endl;
    int ret = HRIF_Connect(BOX_ID, host.c_str(), port);
    printRet("HRIF_Connect", ret);
    if (ret != 0) return EXIT_FAILURE;

    int result = EXIT_SUCCESS;

    // ----------------------------------------------------------
    // 2. Standby：上电、初始化并使能机器人
    // ----------------------------------------------------------
    if (!startRobot()) result = EXIT_FAILURE;

    // ----------------------------------------------------------
    // 3. Business：执行业务任务
    // ----------------------------------------------------------
    if (result == EXIT_SUCCESS && !runBusinessTask()) {
        std::cerr << "[DIAG] Business task failed; state transition did not fail" << std::endl;
        result = EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 4. Shutdown：停止、去使能并下电
    // ----------------------------------------------------------
    if (result == EXIT_SUCCESS && !shutdownRobot()) result = EXIT_FAILURE;

    // ----------------------------------------------------------
    // 5. Disconnect：断开机器人连接
    // ----------------------------------------------------------
    ret = HRIF_DisConnect(BOX_ID);
    if (!printRet("HRIF_DisConnect", ret)) result = EXIT_FAILURE;
    return result;
}
