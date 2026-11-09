using RobotLibrarys;
using System;
using System.Diagnostics;
using System.Threading;

namespace StateMachineManageExample
{
    internal static class Program
    {
        // BoxID 和 RbtID 可以在此处进行修改
        private const int BOX_ID = 0;
        private const int RBT_ID = 0;
        private const int TRANSITION_TIMEOUT_SECONDS = 60;

        // This is intentionally not a complete controller FSM definition.
        private enum FsmState
        {
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
        }

        private static bool PrintRet(RobotAPI robot, string command, int ret)
        {
            if (ret == 0)
            {
                Console.WriteLine("[OK]   " + command);
                return true;
            }
            if (ret == 39500)
            {
                Console.Error.WriteLine("[ERR]  " + command + " -> 39500: Robot not connected");
                return false;
            }

            string message = "";
            int lookupRet = robot.HRIF_GetErrorCodeStr(BOX_ID, ret, ref message);
            Console.Error.WriteLine(
                "[ERR]  " + command + " -> " + ret + (lookupRet == 0 ? ": " + message : ""));
            return false;
        }

        private static bool ReadState(RobotAPI robot, ref FsmState state)
        {
            int value = -1;
            int ret = robot.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, ref value);
            if (ret == 0)
            {
                state = (FsmState)value;
                return true;
            }

            PrintRet(robot, "HRIF_ReadCurFSMFromCPS", ret);
            Console.Error.WriteLine("[DIAG] State read failed; no transition decision was made");
            return false;
        }

        private static bool ReportAbnormalState(FsmState state)
        {
            string guidance;
            switch (state)
            {
                case FsmState.EtherCatError:
                case FsmState.RobotError:
                case FsmState.PluginError:
                case FsmState.SafeMoveError:
                    guidance = "controller error; diagnose the cause before requesting Reset";
                    break;
                case FsmState.CollisionStop:
                    guidance = "collision stop; inspect the robot and work area before requesting Reset";
                    break;
                case FsmState.EmergencyStopped:
                case FsmState.SafetyIoError:
                case FsmState.Safeguarding:
                    guidance = "safety stop; restore and verify the external safety circuit first";
                    break;
                case FsmState.ControllerVersionError:
                case FsmState.PluginDisconnected:
                case FsmState.LowTemperature:
                    guidance = "maintenance condition; operator or service handling is required";
                    break;
                default:
                    return false;
            }

            Console.Error.WriteLine("[ERR]  FSM=" + (int)state + ": " + guidance);
            return true;
        }

        private static bool IsBusinessActive(FsmState state)
        {
            switch (state)
            {
                case FsmState.Moving:
                case FsmState.LongJogMoving:
                case FsmState.Paused:
                case FsmState.ProgramRunning:
                case FsmState.ProgramHeld:
                    return true;
                default:
                    return false;
            }
        }

        private static bool StartRobot(RobotAPI robot)
        {
            Stopwatch timer = Stopwatch.StartNew();
            FsmState previousState = (FsmState)(-1);

            while (timer.Elapsed < TimeSpan.FromSeconds(TRANSITION_TIMEOUT_SECONDS))
            {
                FsmState state = (FsmState)(-1);
                if (!ReadState(robot, ref state)) return false;
                if (state == FsmState.Standby)
                {
                    Console.WriteLine("[READY] Robot reached Standby(33)");
                    return true;
                }
                if (ReportAbnormalState(state)) return false;

                if (state != previousState)
                {
                    bool actionRequired = true;
                    bool actionSucceeded = false;
                    switch (state)
                    {
                        case FsmState.BoxDisconnected:
                            actionSucceeded = PrintRet(
                                robot, "HRIF_Connect2Box", robot.HRIF_Connect2Box(BOX_ID));
                            break;
                        case FsmState.Blackout:
                            actionSucceeded = PrintRet(
                                robot, "HRIF_Electrify", robot.HRIF_Electrify(BOX_ID));
                            break;
                        case FsmState.ControllerDisconnected:
                            actionSucceeded = PrintRet(
                                robot, "HRIF_Connect2Controller",
                                robot.HRIF_Connect2Controller(BOX_ID));
                            break;
                        case FsmState.Disabled:
                            actionSucceeded = PrintRet(
                                robot, "HRIF_GrpEnable",
                                robot.HRIF_GrpEnable(BOX_ID, RBT_ID));
                            break;
                        case FsmState.ManualGuiding:
                            actionSucceeded = PrintRet(
                                robot, "HRIF_MoveToSS", robot.HRIF_MoveToSS(BOX_ID, RBT_ID));
                            break;
                        case FsmState.FreeDrive:
                            actionSucceeded = PrintRet(
                                robot, "HRIF_GrpCloseFreeDriver",
                                robot.HRIF_GrpCloseFreeDriver(BOX_ID, RBT_ID));
                            break;
                        default:
                            // Wait for the controller to complete the current transition.
                            actionRequired = false;
                            break;
                    }
                    if (actionRequired && !actionSucceeded)
                    {
                        Console.Error.WriteLine(
                            "[DIAG] Startup transition failed at FSM=" + (int)state);
                        return false;
                    }
                }

                previousState = state;
                Thread.Sleep(100);
            }

            Console.Error.WriteLine(
                "[ERR]  Robot did not reach Standby(33) within " +
                TRANSITION_TIMEOUT_SECONDS + " seconds");
            return false;
        }

        private static bool ShutdownRobot(RobotAPI robot)
        {
            Stopwatch timer = Stopwatch.StartNew();
            FsmState previousState = (FsmState)(-1);

            while (timer.Elapsed < TimeSpan.FromSeconds(TRANSITION_TIMEOUT_SECONDS))
            {
                FsmState state = (FsmState)(-1);
                if (!ReadState(robot, ref state)) return false;
                if (state == FsmState.Blackout)
                {
                    Console.WriteLine("[SAFE]  Robot reached Blackout(7)");
                    return true;
                }
                if (ReportAbnormalState(state)) return false;

                if (state != previousState)
                {
                    bool actionRequired = true;
                    bool actionSucceeded = false;
                    if (IsBusinessActive(state))
                    {
                        actionSucceeded = PrintRet(
                            robot, "HRIF_GrpStop", robot.HRIF_GrpStop(BOX_ID, RBT_ID));
                    }
                    else
                    {
                        switch (state)
                        {
                            case FsmState.Standby:
                                actionSucceeded = PrintRet(
                                    robot, "HRIF_GrpDisable",
                                    robot.HRIF_GrpDisable(BOX_ID, RBT_ID));
                                break;
                            case FsmState.Disabled:
                            case FsmState.ControllerDisconnected:
                                actionSucceeded = PrintRet(
                                    robot, "HRIF_BlackOut", robot.HRIF_BlackOut(BOX_ID));
                                break;
                            case FsmState.ManualGuiding:
                                actionSucceeded = PrintRet(
                                    robot, "HRIF_MoveToSS", robot.HRIF_MoveToSS(BOX_ID, RBT_ID));
                                break;
                            case FsmState.FreeDrive:
                                actionSucceeded = PrintRet(
                                    robot, "HRIF_GrpCloseFreeDriver",
                                    robot.HRIF_GrpCloseFreeDriver(BOX_ID, RBT_ID));
                                break;
                            default:
                                // Wait for the controller to complete the current transition.
                                actionRequired = false;
                                break;
                        }
                    }
                    if (actionRequired && !actionSucceeded)
                    {
                        Console.Error.WriteLine(
                            "[DIAG] Shutdown transition failed at FSM=" + (int)state);
                        return false;
                    }
                }

                previousState = state;
                Thread.Sleep(100);
            }

            Console.Error.WriteLine(
                "[ERR]  Robot did not reach Blackout(7) within " +
                TRANSITION_TIMEOUT_SECONDS + " seconds");
            return false;
        }

        private static bool RunBusinessTask()
        {
            Console.WriteLine("[BUSINESS] Execute the application task here");
            return true;
        }

        public static int Main()
        {
            // ----------------------------------------------------------
            // 1. Connect：连接机器人
            // ----------------------------------------------------------
            // 需根据机器人的实际 IP 进行修改
            string host = "192.168.0.10";
            int port = 10003;

            Console.WriteLine("=== Controller State Machine Workflow Example ===");
            RobotAPI robot = new RobotAPI();
            int ret = robot.HRIF_Connect(BOX_ID, host, port);
            PrintRet(robot, "HRIF_Connect", ret);
            if (ret != 0) return 1;

            int result = 0;

            // ----------------------------------------------------------
            // 2. Standby：上电、初始化并使能机器人
            // ----------------------------------------------------------
            if (!StartRobot(robot)) result = 1;

            // ----------------------------------------------------------
            // 3. Business：执行业务任务
            // ----------------------------------------------------------
            if (result == 0 && !RunBusinessTask())
            {
                Console.Error.WriteLine(
                    "[DIAG] Business task failed; state transition did not fail");
                result = 1;
            }

            // ----------------------------------------------------------
            // 4. Shutdown：停止、去使能并下电
            // ----------------------------------------------------------
            if (result == 0 && !ShutdownRobot(robot)) result = 1;

            // ----------------------------------------------------------
            // 5. Disconnect：断开机器人连接
            // ----------------------------------------------------------
            ret = robot.HRIF_DisConnect(BOX_ID);
            if (!PrintRet(robot, "HRIF_DisConnect", ret)) result = 1;
            return result;
        }
    }
}
