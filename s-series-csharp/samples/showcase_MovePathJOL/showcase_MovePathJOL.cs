using RobotLibrarys;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;

namespace MovePathJOLExample
{
    internal static class Program
    {
        // BoxID 和 RbtID 可以在此处进行修改
        private const int BOX_ID = 0;
        private const int RBT_ID = 0;
        private const int BOX_DO_BIT = 2;
        private const int BOX_DO_MASK = 1 << BOX_DO_BIT;

        private static void PrintRet(RobotAPI robot, string command, int ret)
        {
            if (ret == 0)
            {
                Console.WriteLine("[OK]  " + command);
            }
            else
            {
                string errorMessage = "";
                int lookupRet = robot.HRIF_GetErrorCodeStr(BOX_ID, ret, ref errorMessage);
                Console.WriteLine("[ERR] " + command + " -> " + ret +
                                  (lookupRet == 0 ? ": " + errorMessage : ""));
            }
        }

        public static int Main()
        {
            Console.WriteLine("=== MovePathJOL Online Path Example ===");

            // ----------------------------------------------------------
            // 1. 连接机器人
            // ----------------------------------------------------------
            RobotAPI robot = new RobotAPI();

            // 需根据机器人的实际 IP 进行修改
            string host = "192.168.0.10";
            int port = 10003;

            int ret = robot.HRIF_Connect(BOX_ID, host, port);
            PrintRet(robot, "HRIF_Connect", ret);
            if (ret != 0) return 1;

            // ----------------------------------------------------------
            // 2. 确认机器人处于 Standby(33) 状态
            // ----------------------------------------------------------
            int nFSM = 0;
            ret = robot.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, ref nFSM);
            PrintRet(robot, $"HRIF_ReadCurFSM (FSM={nFSM})", ret);
            if (ret != 0 || nFSM != 33)
            {
                Console.WriteLine($"[ERR] Robot must be in Standby(33), FSM={nFSM}");
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 3. 设置速度比，并运动到安全位置
            // ----------------------------------------------------------
            ret = robot.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
            PrintRet(robot, "HRIF_SetOverride", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            double dJ1 = 0; double dJ2 = -90; double dJ3 = 90;
            double dJ4 = -90; double dJ5 = -90; double dJ6 = 0;

            Console.WriteLine("[INFO] Moving to a safe position with MoveJ...");
            ret = robot.HRIF_MoveJ(
                BOX_ID, RBT_ID,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
                "TCP", "Base", 30.0, 60.0, 0.0,
                1, 0, 0, 0, "MovePathJOL_home");
            PrintRet(robot, "HRIF_MoveJ", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            bool motionDone = false;
            Stopwatch motionWait = Stopwatch.StartNew();
            while (!motionDone && motionWait.Elapsed < TimeSpan.FromSeconds(60))
            {
                Thread.Sleep(100);
                ret = robot.HRIF_IsMotionDone(BOX_ID, RBT_ID, ref motionDone);
                if (ret != 0) break;
            }
            if (ret != 0 || !motionDone)
            {
                if (ret != 0) PrintRet(robot, "HRIF_IsMotionDone", ret);
                else Console.WriteLine("[ERR] MoveJ timed out");
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }
            Console.WriteLine("[INFO] Safe position reached");

            // ----------------------------------------------------------
            // 4. 准备并一次性下发 10 个点
            // ----------------------------------------------------------
            const int POINT_COUNT = 10;
            var points = new List<MovePathJOLPointsData>(POINT_COUNT);
            for (int i = 0; i < POINT_COUNT; ++i)
            {
                double angle = 2.0 * Math.PI * i / (POINT_COUNT - 1);
                points.Add(new MovePathJOLPointsData
                {
                    dbJ1 = dJ1 + 1.5 * Math.Sin(angle),
                    dbJ2 = dJ2 + 0.8 * (1.0 - Math.Cos(angle)),
                    dbJ3 = dJ3,
                    dbJ4 = dJ4,
                    dbJ5 = dJ5,
                    dbJ6 = dJ6,
                    nIsSetIO = i == 4 || i == 9 ? 1 : 0,
                    nBoxDOMask = i == 4 || i == 9 ? BOX_DO_MASK : 0,
                    nBoxDOVal = i == 4 ? BOX_DO_MASK : 0
                });
            }

            ret = robot.HRIF_MovePathJOL(BOX_ID, RBT_ID, 20.0, 20.0, 20.0, points);
            PrintRet(robot, "HRIF_MovePathJOL (10 points)", ret);
            if (ret != 0)
            {
                robot.HRIF_GrpStop(BOX_ID, RBT_ID);
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 5. 第 5 个点执行完成后读取 DO2 状态
            // ----------------------------------------------------------
            int currentIndex = 0;
            int pointCount = 0;
            Stopwatch pathWait = Stopwatch.StartNew();
            while (currentIndex < 6 && pathWait.Elapsed < TimeSpan.FromSeconds(60))
            {
                Thread.Sleep(100);
                ret = robot.HRIF_GetMovePathJOLIndex(
                    BOX_ID, RBT_ID, ref currentIndex, ref pointCount);
                if (ret != 0) break;
            }
            if (ret != 0 || currentIndex < 6)
            {
                if (ret != 0) PrintRet(robot, "HRIF_GetMovePathJOLIndex", ret);
                else Console.WriteLine("[ERR] Wait for MovePathJOL point 5 completion timed out");
                robot.HRIF_GrpStop(BOX_ID, RBT_ID);
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            int doValue = 0;
            ret = robot.HRIF_ReadBoxDO(BOX_ID, BOX_DO_BIT, ref doValue);
            PrintRet(robot, $"HRIF_ReadBoxDO (point 5, DO2={doValue})", ret);
            if (ret != 0)
            {
                robot.HRIF_GrpStop(BOX_ID, RBT_ID);
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 6. 等待第 10 个点完成，并再次读取 DO2 状态
            // ----------------------------------------------------------
            pathWait.Restart();
            while (currentIndex < 10 && pathWait.Elapsed < TimeSpan.FromSeconds(60))
            {
                Thread.Sleep(100);
                ret = robot.HRIF_GetMovePathJOLIndex(
                    BOX_ID, RBT_ID, ref currentIndex, ref pointCount);
                if (ret != 0) break;
            }
            if (ret != 0 || currentIndex < 10)
            {
                if (ret != 0) PrintRet(robot, "HRIF_GetMovePathJOLIndex", ret);
                else Console.WriteLine("[ERR] Wait for MovePathJOL point 10 timed out");
                robot.HRIF_GrpStop(BOX_ID, RBT_ID);
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            motionDone = false;
            motionWait.Restart();
            while (!motionDone && motionWait.Elapsed < TimeSpan.FromSeconds(60))
            {
                Thread.Sleep(100);
                ret = robot.HRIF_IsMotionDone(BOX_ID, RBT_ID, ref motionDone);
                if (ret != 0) break;
            }
            if (ret != 0 || !motionDone)
            {
                if (ret != 0) PrintRet(robot, "HRIF_IsMotionDone", ret);
                else Console.WriteLine("[ERR] MovePathJOL completion timed out");
                robot.HRIF_GrpStop(BOX_ID, RBT_ID);
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            ret = robot.HRIF_ReadBoxDO(BOX_ID, BOX_DO_BIT, ref doValue);
            PrintRet(robot, $"HRIF_ReadBoxDO (completed, DO2={doValue})", ret);

            // ----------------------------------------------------------
            // 7. 停止运动并断开连接
            // ----------------------------------------------------------
            Console.WriteLine("[INFO] MovePathJOL completed; disconnecting");
            int stopRet = robot.HRIF_GrpStop(BOX_ID, RBT_ID);
            PrintRet(robot, "HRIF_GrpStop", stopRet);
            int disconnectRet = robot.HRIF_DisConnect(BOX_ID);
            PrintRet(robot, "HRIF_DisConnect", disconnectRet);
            return ret == 0 && stopRet == 0 && disconnectRet == 0 ? 0 : 1;
        }
    }
}
