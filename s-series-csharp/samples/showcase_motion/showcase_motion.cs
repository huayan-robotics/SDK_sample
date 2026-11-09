using RobotLibrarys;
using System;
using System.Diagnostics;
using System.Threading;

namespace HuayanRobotExamples
{
    internal static class MotionExample
    {
        // BoxID 和 RbtID 可以在此处进行修改
        private const int BOX_ID = 0;
        private const int RBT_ID = 0;

        private static void print_ret(RobotAPI robot, string cmd, int ret)
        {
            if (ret == 0)
            {
                Console.WriteLine("[OK]  " + cmd);
                return;
            }

            string errMsg = "";
            int lookupRet = robot.HRIF_GetErrorCodeStr(BOX_ID, ret, ref errMsg);
            Console.Error.WriteLine("[ERR] " + cmd + " -> " + ret + (lookupRet == 0 ? ": " + errMsg : ""));
        }

        private static bool wait_motion_done(RobotAPI robot)
        {
            Stopwatch deadline = Stopwatch.StartNew();
            while (deadline.Elapsed < TimeSpan.FromSeconds(60))
            {
                Thread.Sleep(500);
                bool bDone = false;
                int ret = robot.HRIF_IsMotionDone(BOX_ID, RBT_ID, ref bDone);
                if (ret != 0)
                {
                    print_ret(robot, "HRIF_IsMotionDone", ret);
                    return false;
                }
                if (bDone) return true;
            }

            Console.Error.WriteLine("[ERR] Motion timeout (60s).");
            return false;
        }

        public static int Main(string[] args)
        {
            Console.WriteLine("======== Waypoint Motion Example Start ========");
            RobotAPI robot = new RobotAPI();

            // ----------------------------------------------------------
            // 1. 连接机器人
            // ----------------------------------------------------------
            // 需根据机器人的实际 IP 进行修改
            const string ip = "192.168.0.10";
            const int port = 10003;

            int ret = robot.HRIF_Connect(BOX_ID, ip, port);
            print_ret(robot, "HRIF_Connect", ret);
            if (ret != 0) return 1;

            // ----------------------------------------------------------
            // 2. 确认机器人处于 Standby(33)
            // ----------------------------------------------------------
            int nFSM = 0;
            ret = robot.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, ref nFSM);
            print_ret(robot, $"HRIF_ReadCurFSM (FSM={nFSM})", ret);
            if (ret != 0 || nFSM != 33)
            {
                Console.Error.WriteLine($"[ERR] Robot must be in Standby(33), FSM={nFSM}");
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 3. 设置全局速度比为 0.2
            // ----------------------------------------------------------
            ret = robot.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
            print_ret(robot, "HRIF_SetOverride", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 4. MoveJ 运动到安全关节位置，无过渡
            // ----------------------------------------------------------
            double dJ1 = 0; double dJ2 = -90; double dJ3 = 90;
            double dJ4 = -90; double dJ5 = -90; double dJ6 = 0;

            ret = robot.HRIF_MoveJ(BOX_ID, RBT_ID, 0, 0, 0, 0, 0, 0,
                                   dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, "TCP", "Base",
                                   30.0, 60.0, 0.0, 1, 0, 0, 0, "absolute_MoveJ_home");
            print_ret(robot, "HRIF_MoveJ(Point A)", ret);
            if (ret != 0 || !wait_motion_done(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 5. 读取 TCP 当前位置，用于后续路点运动
            // 从安全点 A 开始绘制云朵形状：
            //                         M
            //             Waypoint ╱     ╲ WaypointEx
            //                    C         D
            //                  ╭             ╮
            //               MoveC          WayPoint2
            //                  ╰             ╯
            //                    B ◄─MoveL── A
            // ----------------------------------------------------------
            const double dStraightLength = 80.0; //直线长度
            const double dArcRadius = 20.0; //圆弧半径
            double dX = 0.0; double dY = 0.0; double dZ = 0.0;
            double dRx = 0.0; double dRy = 0.0; double dRz = 0.0;

            ret = robot.HRIF_ReadActTcpPos(BOX_ID, RBT_ID, ref dX, ref dY, ref dZ,
                                           ref dRx, ref dRy, ref dRz);
            print_ret(robot, "HRIF_ReadActPos", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 6. 从 A 点 MoveL 直线运动到点 B，无过渡
            // ----------------------------------------------------------
            dX += dStraightLength;
            ret = robot.HRIF_MoveL(BOX_ID, RBT_ID, dX, dY, dZ, dRx, dRy, dRz,
                                   0, 0, 0, 0, 0, 0, "TCP", "Base",
                                   30.0, 60.0, 0.0, 0, 0, 0, "absolute_MoveL_approach");
            print_ret(robot, "HRIF_MoveL(Point B)", ret);
            if (ret != 0 || !wait_motion_done(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 7. MoveC：经过辅助点执行 B -> C 圆弧，无过渡
            // ----------------------------------------------------------
            double dAuxPos_X = dX + dArcRadius;
            double dAuxPos_Y = dY + dArcRadius;
            double dEndPos_Y = dY + 2 * dArcRadius;

            ret = robot.HRIF_MoveC(BOX_ID, RBT_ID, dX, dY, dZ, dRx, dRy, dRz,
                                   dAuxPos_X, dAuxPos_Y, dZ, dRx, dRy, dRz, dX, dEndPos_Y, dZ, dRx, dRy, dRz,
                                   0, 1, 0.0, 30.0, 60.0, 0.0, "TCP", "Base", "absolute_MoveC_BC");
            print_ret(robot, "HRIF_MoveC(B->C)", ret);
            if (ret != 0 || !wait_motion_done(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 8. WayPoint：C -> M，有过渡，过渡半径为 20.0 mm
            // ----------------------------------------------------------
            int nMoveType = 1;
            dX -= dStraightLength / 2.0;
            dY += 2.0 * dArcRadius + 50;

            ret = robot.HRIF_WayPoint(BOX_ID, RBT_ID, dX, dY, dZ, dRx, dRy, dRz, 0, 0, 0, 0, 0, 0,
                                      "TCP", "Base", 30.0, 60.0, 20.0, nMoveType,
                                      0, 0, 0, 0, "WayPoint_C_M");
            print_ret(robot, "HRIF_WayPoint(C->M)", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 9. WayPointEx：M -> D，有过渡，过渡半径为 20.0 mm
            // ----------------------------------------------------------
            dX -= dStraightLength / 2.0;
            dY -= 50;

            ret = robot.HRIF_WayPointEx(BOX_ID, RBT_ID, dX, dY, dZ, dRx, dRy, dRz, 0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30.0, 60.0, 20.0,
                                        nMoveType, 0, 0, 0, 0, "WayPointEx_M_D");
            print_ret(robot, "HRIF_WayPointEx(M->D)", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 10. WayPoint2：D -> A，绘制圆弧，无过渡
            // ----------------------------------------------------------
            nMoveType = 2;
            dEndPos_Y = dY - 2 * dArcRadius;
            dAuxPos_X = dX - dArcRadius;
            dAuxPos_Y = dY - dArcRadius;

            ret = robot.HRIF_WayPoint2(BOX_ID, RBT_ID, dX, dEndPos_Y, dZ, dRx, dRy, dRz,
                                       dAuxPos_X, dAuxPos_Y, dZ, dRx, dRy, dRz, 0, 0, 0, 0, 0, 0,
                                       "TCP", "Base", 30.0, 60.0, 0.0, nMoveType, 0, 0, 0, 0, "WayPoint2_D_A");
            print_ret(robot, "HRIF_WayPoint2(D->A)", ret);
            if (ret != 0 || !wait_motion_done(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 11. 断开连接
            // ----------------------------------------------------------
            Console.WriteLine("[OK] Absolute motion completed.");
            ret = robot.HRIF_DisConnect(BOX_ID);
            print_ret(robot, "HRIF_DisConnect", ret);
            return ret == 0 ? 0 : 1;
        }
    }
}
