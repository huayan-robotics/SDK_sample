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

        private static bool PrintRet(RobotAPI robot, string command, int ret)
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

        private static bool WaitMotionDone(RobotAPI robot)
        {
            Stopwatch deadline = Stopwatch.StartNew();
            while (deadline.Elapsed < TimeSpan.FromSeconds(60))
            {
                bool done = false;
                int ret = robot.HRIF_IsMotionDone(BOX_ID, RBT_ID, ref done);
                if (ret != 0)
                {
                    PrintRet(robot, "HRIF_IsMotionDone", ret);
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
            // 1. 连接机器人，确认处于 Standby(33) 并设置速度比为 0.2
            // ----------------------------------------------------------
            // 需根据机器人的实际 IP 进行修改
            const string host = "192.168.0.10";
            const int port = 10003;

            int ret = robot.HRIF_Connect(BOX_ID, host, port);
            PrintRet(robot, "HRIF_Connect", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            int fsm = 0;
            ret = robot.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, ref fsm);
            PrintRet(robot, "HRIF_ReadCurFSM (FSM=" + fsm + ")", ret);
            if (ret != 0 || fsm != 33)
            {
                Console.Error.WriteLine("[ERR] Robot must be in Standby(33), FSM=" + fsm);
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            ret = robot.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
            PrintRet(robot, "HRIF_SetOverride", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 2. MoveJ_nJ 运动到安全原点
            // ----------------------------------------------------------
            Pose zeroPose = new Pose();
            JointsData safeJoints = new JointsData();
            safeJoints.joint[0] = 90; safeJoints.joint[1] = 45;
            safeJoints.joint[2] = 0; safeJoints.joint[3] = -80;
            safeJoints.joint[4] = 0; safeJoints.joint[5] = 35;
            safeJoints.joint[6] = 0;

            ret = robot.HRIF_MoveJ_nJ(BOX_ID, RBT_ID, zeroPose, safeJoints, "TCP", "Base", 30.0, 60.0,
                                      0.0, 1, 0, 0, 0, "moveJ_to_safepoints");
            PrintRet(robot, "HRIF_MoveJ_nJ (safepoint)", ret);
            if (ret != 0 || !WaitMotionDone(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }
            // ----------------------------------------------------------
            // 3. 连续下发用于演示暂停、继续和停止的 MoveJ_nJ 运动
            // ----------------------------------------------------------
            ret = robot.HRIF_SetBoxDO(BOX_ID, 0, 0);
            PrintRet(robot, "HRIF_SetBoxDO (DO0=0)", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            safeJoints.joint[0] += 10.0;
            ret = robot.HRIF_MoveJ_nJ(BOX_ID, RBT_ID, zeroPose, safeJoints, "TCP", "Base", 30.0, 60.0,
                                      0.0, 1, 0, 0, 0, "motion_status_interrupt");
            PrintRet(robot, "HRIF_MoveJ_nJ (interrupt demo)", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            ret = robot.HRIF_cdsSetIO(BOX_ID, RBT_ID, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0);
            PrintRet(robot, "HRIF_cdsSetIO", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            safeJoints.joint[0] -= 10.0;
            ret = robot.HRIF_MoveJ_nJ(BOX_ID, RBT_ID, zeroPose, safeJoints, "TCP", "Base", 30.0, 60.0,
                                      0.0, 1, 0, 0, 0, "motion_status_stop");
            PrintRet(robot, "HRIF_MoveJ_nJ (stop demo)", ret);
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
                    PrintRet(robot, "HRIF_ReadCurWaypointID", ret);
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }
                Thread.Sleep(100);
            }
            Console.WriteLine("[INFO] Current motion stage ID is: " + commandId);

            ret = robot.HRIF_GrpInterrupt(BOX_ID, RBT_ID);
            PrintRet(robot, "HRIF_GrpInterrupt", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            Thread.Sleep(1000);
            ret = robot.HRIF_GrpContinue(BOX_ID, RBT_ID);
            PrintRet(robot, "HRIF_GrpContinue", ret);
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
                    PrintRet(robot, "HRIF_ReadCurWaypointID", ret);
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }
                Thread.Sleep(100);
            }
            Console.WriteLine("[INFO] Current motion stage ID is: " + commandId);

            ret = robot.HRIF_GrpStop(BOX_ID, RBT_ID);
            PrintRet(robot, "HRIF_GrpStop", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            int do0 = 0;
            while (do0 != 1)
            {
                ret = robot.HRIF_ReadBoxDO(BOX_ID, 0, ref do0);
                if (ret != 0)
                {
                    PrintRet(robot, "HRIF_ReadBoxDO", ret);
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }
                Thread.Sleep(100);
            }

            Console.WriteLine("[Info] Motion Done! Disconnecting");
            ret = robot.HRIF_DisConnect(BOX_ID);
            PrintRet(robot, "HRIF_DisConnect", ret);
            return ret == 0 ? 0 : 1;
        }
    }
}
