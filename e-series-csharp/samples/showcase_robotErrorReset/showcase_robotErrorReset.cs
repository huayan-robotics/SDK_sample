using RobotLibrarys;
using System;
using System.Diagnostics;
using System.Threading;

namespace RobotErrorResetExample
{
    internal static class Program
    {
        // BoxID 和 RbtID 可以在此处进行修改
        private const int BOX_ID = 0;
        private const int RBT_ID = 0;

        // Preserve the original code and append a readable description when available.
        private static void PrintErrorCode(RobotAPI robot, string source, int errorCode)
        {
            if (errorCode == 0) return;
            string message = "";
            int ret = robot.HRIF_GetErrorCodeStr(BOX_ID, errorCode, ref message);
            Console.Error.Write($"[ERROR] {source} -> {errorCode}");
            if (ret == 0) Console.Error.Write($": {message}");
            Console.Error.WriteLine();
        }

        // Use the leading digit only for quick grouping, not as a full diagnosis.
        private static string ErrorCategory(int errorCode)
        {
            switch (errorCode / 10000)
            {
                case 1: return "robot body";
                case 2: return "controller internal";
                case 3: return "external device communication";
                case 4: return "controller internal";
                case 6: return "specific function or device";
                default: return "other";
            }
        }

        // Handle only FSM states that explicitly represent errors or safety events.
        private static bool IsErrorState(int state)
        {
            switch (state)
            {
                case 5: case 10: case 12: case 17:
                case 21: case 22: case 40: case 52:
                    return true;
                default:
                    return false;
            }
        }

        private static bool ReadDiagnostics(RobotAPI robot)
        {
            // ReadRobotState returns an error summary plus motion, safety, and power states.
            int moving = 0, enabled = 0, errorState = 0, errorCode = 0, errorAxis = 0;
            int braking = 0, paused = 0, emergencyStop = 0, safeGuard = 0;
            int electrified = 0, connectedToBox = 0, blendingDone = 0, inPosition = 0;
            int ret = robot.HRIF_ReadRobotState(
                BOX_ID, RBT_ID, ref moving, ref enabled, ref errorState,
                ref errorCode, ref errorAxis, ref braking, ref paused,
                ref emergencyStop, ref safeGuard, ref electrified,
                ref connectedToBox, ref blendingDone, ref inPosition);
            if (ret != 0) {
                PrintErrorCode(robot, "HRIF_ReadRobotState", ret);
                return false;
            }

            Console.WriteLine(
                $"[STATE] error={errorState}, code={errorCode}, " +
                $"category={ErrorCategory(errorCode)}, axis={errorAxis}, " +
                $"emergencyStop={emergencyStop}, safeGuard={safeGuard}");
            PrintErrorCode(robot, "controller", errorCode);

            // Per-axis codes supplement errorAxis and identify the affected joint.
            int controllerCode = 0;
            int j1 = 0, j2 = 0, j3 = 0, j4 = 0, j5 = 0, j6 = 0;
            ret = robot.HRIF_ReadAxisErrorCode(
                BOX_ID, RBT_ID, ref controllerCode,
                ref j1, ref j2, ref j3, ref j4, ref j5, ref j6);
            if (ret != 0) {
                PrintErrorCode(robot, "HRIF_ReadAxisErrorCode", ret);
                return false;
            }
            PrintErrorCode(robot, "axis group", controllerCode);
            int[] axisCodes = { j1, j2, j3, j4, j5, j6 };
            for (int index = 0; index < axisCodes.Length; ++index)
                if (axisCodes[index] != 0) PrintErrorCode(robot, $"J{index + 1}", axisCodes[index]);

            // Observe channel mismatches and physical safety signals separately.
            int emergencyCircuitError = 0, emergencySignal = 0;
            int safeguardCircuitError = 0, safeguardSignal = 0;
            ret = robot.HRIF_ReadEmergencyInfo(
                BOX_ID, RBT_ID, ref emergencyCircuitError, ref emergencySignal,
                ref safeguardCircuitError, ref safeguardSignal);
            if (ret != 0) {
                PrintErrorCode(robot, "HRIF_ReadEmergencyInfo", ret);
                return false;
            }
            Console.WriteLine(
                $"[SAFETY] emergencyCircuitError={emergencyCircuitError}, " +
                $"emergencySignal={emergencySignal}, " +
                $"safeguardCircuitError={safeguardCircuitError}, " +
                $"safeguardSignal={safeguardSignal}");
            return true;
        }

        public static int Main()
        {
            // This example does not create faults; use it only for a safely recoverable alarm.

            // ----------------------------------------------------------
            // 1. 连接机器人
            // ----------------------------------------------------------
            // 需根据机器人的实际 IP 进行修改
            string host = "192.168.0.10";
            int port = 10003;

            RobotAPI robot = new RobotAPI();
            int ret = robot.HRIF_Connect(BOX_ID, host, port);
            if (ret != 0)
            {
                Console.Error.WriteLine($"[ERROR] HRIF_Connect -> {ret}");
                return 1;
            }

            // ----------------------------------------------------------
            // 2. 读取状态机并判断是否需要复位
            // ----------------------------------------------------------
            int state = -1;
            ret = robot.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, ref state);
            if (ret != 0) {
                PrintErrorCode(robot, "HRIF_ReadCurFSM", ret);
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }
            Console.WriteLine($"[FSM] {state}");
            if (!IsErrorState(state))
            {
                Console.WriteLine(
                    "[INFO] The state machine is not in an error state handled by this example; " +
                    "no reset is required.");
                robot.HRIF_DisConnect(BOX_ID);
                return 0;
            }

            // ----------------------------------------------------------
            // 3. 读取错误及安全诊断信息
            // ----------------------------------------------------------
            if (!ReadDiagnostics(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 4. 确认故障原因已排除
            // ----------------------------------------------------------
            Console.WriteLine(
                "Clear the emergency stop, safeguard, collision, communication, or " +
                "equipment fault indicated by the diagnostics.");
            Console.WriteLine(
                "After confirming site safety and clearing the cause, " +
                "press Enter to request one reset...");
            Console.ReadLine();

            // ----------------------------------------------------------
            // 5. 请求复位机器人
            // ----------------------------------------------------------
            ret = robot.HRIF_GrpReset(BOX_ID, RBT_ID);
            if (ret != 0) {
                PrintErrorCode(robot, "HRIF_GrpReset", ret);
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 6. 轮询状态机并确认复位结果
            // ----------------------------------------------------------
            Stopwatch timer = Stopwatch.StartNew();
            while (timer.Elapsed < TimeSpan.FromSeconds(10))
            {
                ret = robot.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, ref state);
                if (ret != 0) {
                    PrintErrorCode(robot, "HRIF_ReadCurFSM", ret);
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }
                Console.WriteLine($"[FSM] {state}");
                if (!IsErrorState(state))
                {
                    Console.WriteLine(
                        "[OK] The state machine has left the error state. " +
                        "Resume operation through the state-machine flow.");
                    robot.HRIF_DisConnect(BOX_ID);
                    return 0;
                }
                Thread.Sleep(200);
            }

            Console.Error.WriteLine(

                "[ERROR] The state machine remains in an error state after reset; " +

                "inspect the diagnostics or contact technical support.");
            robot.HRIF_DisConnect(BOX_ID);
            return 1;
        }
    }
}
