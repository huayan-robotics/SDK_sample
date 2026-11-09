using RobotLibrarys;
using System;
using System.Diagnostics;
using System.Threading;

namespace HuayanRobotExamples
{
    internal static class MotionStatusManagementExample
    {
        // BoxID 和 RbtID 可以在此处进行修改
        private const int BOX_ID = 0;
        private const int RBT_ID = 0;

        private static bool print_ret(RobotAPI robot, string command, int ret)
        {
            if (ret == 0)
            {
                Console.WriteLine("[OK]  " + command);
                return true;
            }

            string message = "";
            int lookupRet = robot.HRIF_GetErrorCodeStr(BOX_ID, ret, ref message);
            Console.Error.Write("[ERR] " + command + " -> " + ret);
            if (lookupRet == 0) Console.Error.Write(": " + message);
            Console.Error.WriteLine();
            return false;
        }

        private static bool wait_motion_done(RobotAPI robot)
        {
            Stopwatch deadline = Stopwatch.StartNew();
            while (deadline.Elapsed < TimeSpan.FromSeconds(60))
            {
                bool done = false;
                int ret = robot.HRIF_IsMotionDone(BOX_ID, RBT_ID, ref done);
                if (ret != 0)
                {
                    print_ret(robot, "HRIF_IsMotionDone", ret);
                    return false;
                }
                if (done) return true;
                Thread.Sleep(500);
            }

            Console.Error.WriteLine("[ERR] Motion timeout (60s).");
            return false;
        }

        public static int Main(string[] args)
        {
            Console.WriteLine("======== Motion Status Management Example ========");
            RobotAPI robot = new RobotAPI();

            // ----------------------------------------------------------
            // 1. 连接机器人，确认处于使能状态 Standby(33) 并设置速度比为 0.2
            // ----------------------------------------------------------
            // 需根据机器人的实际 IP 进行修改
            const string host = "192.168.0.10";
            const int port = 10003;

            int ret = robot.HRIF_Connect(BOX_ID, host, port);
            print_ret(robot, "HRIF_Connect", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            int fsm = 0;
            ret = robot.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, ref fsm);
            print_ret(robot, $"HRIF_ReadCurFSM (FSM={fsm})", ret);
            if (ret != 0 || fsm != 33)
            {
                Console.Error.WriteLine($"[ERR] Robot must be in Standby(33), FSM={fsm}");
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            ret = robot.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
            print_ret(robot, "HRIF_SetOverride", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 2. MoveJ 运动到安全原点
            // ----------------------------------------------------------
            double dJ1 = 0; double dJ2 = 0; double dJ3 = 90;
            double dJ4 = 0; double dJ5 = 90; double dJ6 = 0;

            // 先完整运动到安全原点。
            ret = robot.HRIF_MoveJ(BOX_ID, RBT_ID, 0, 0, 0, 0, 0, 0, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
                                   "TCP", "Base", 30.0, 60.0, 0.0, 1, 0, 0, 0, "moveJ_to_safepoints");
            print_ret(robot, "HRIF_MoveJ (safepoint)", ret);
            if (ret != 0 || !wait_motion_done(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 3. 连续下发用于演示暂停继续和停止的 MoveJ 运动
            // ----------------------------------------------------------
            ret = robot.HRIF_SetBoxDO(BOX_ID, 0, 0);
            print_ret(robot, "HRIF_SetBoxDO (DO0=0)", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            ret = robot.HRIF_MoveJ(BOX_ID, RBT_ID, 0, 0, 0, 0, 0, 0, dJ1 + 50.0, dJ2, dJ3, dJ4, dJ5, dJ6,
                                   "TCP", "Base", 30.0, 60.0, 0.0, 1, 0, 0, 0, "motion_status_interrupt");
            print_ret(robot, "HRIF_MoveJ (interrupt demo)", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            ret = robot.HRIF_cdsSetIO(BOX_ID, RBT_ID, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0);
            print_ret(robot, "HRIF_cdsSetIO", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            ret = robot.HRIF_MoveJ(BOX_ID, RBT_ID, 0, 0, 0, 0, 0, 0, dJ1 - 50.0, dJ2, dJ3, dJ4, dJ5, dJ6,
                                   "TCP", "Base", 30.0, 60.0, 0.0, 1, 0, 0, 0, "motion_status_stop");
            print_ret(robot, "HRIF_MoveJ (stop demo)", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 4. 根据运动 ID 暂停、继续和停止，并通过输出 IO 判断运动是否完成
            // ----------------------------------------------------------
            string commandId = "";
            while (commandId != "motion_status_interrupt")
            {
                ret = robot.HRIF_ReadCurWaypointID(BOX_ID, RBT_ID, ref commandId);
                if (ret != 0)
                {
                    print_ret(robot, "HRIF_ReadCurWaypointID", ret);
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }
                Thread.Sleep(100);
            }
            Console.WriteLine("[INFO] Current motion stage ID is: " + commandId);

            ret = robot.HRIF_GrpInterrupt(BOX_ID, RBT_ID);
            print_ret(robot, "HRIF_GrpInterrupt", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            Thread.Sleep(1000);
            ret = robot.HRIF_GrpContinue(BOX_ID, RBT_ID);
            print_ret(robot, "HRIF_GrpContinue", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            commandId = "";
            while (commandId != "motion_status_stop")
            {
                ret = robot.HRIF_ReadCurWaypointID(BOX_ID, RBT_ID, ref commandId);
                if (ret != 0)
                {
                    print_ret(robot, "HRIF_ReadCurWaypointID", ret);
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }
                Thread.Sleep(100);
            }
            Console.WriteLine("[INFO] Current motion stage ID is: " + commandId);

            ret = robot.HRIF_GrpStop(BOX_ID, RBT_ID);
            print_ret(robot, "HRIF_GrpStop", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            int DO0 = 0;
            while (DO0 != 1)
            {
                ret = robot.HRIF_ReadBoxDO(BOX_ID, 0, ref DO0);
                if (ret != 0)
                {
                    print_ret(robot, "HRIF_ReadBoxDO", ret);
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }
                Thread.Sleep(100);
            }

            Console.WriteLine("[Info] Motion Done! Disconnecting");
            ret = robot.HRIF_DisConnect(BOX_ID);
            print_ret(robot, "HRIF_DisConnect", ret);
            return ret == 0 ? 0 : 1;
        }
    }
}
