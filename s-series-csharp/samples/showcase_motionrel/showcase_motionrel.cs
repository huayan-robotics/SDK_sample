using RobotLibrarys;
using System;
using System.Diagnostics;
using System.Threading;

namespace HuayanRobotExamples
{
    internal static class MotionRelExample
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
            Console.WriteLine("======== Relative motion example started. ========");
            RobotAPI robot = new RobotAPI();

            // ----------------------------------------------------------
            // 1. 连接机器人
            // ----------------------------------------------------------
            // 需根据机器人的实际 IP 进行修改
            const string host = "192.168.0.10";
            const int port = 10003;

            int ret = robot.HRIF_Connect(BOX_ID, host, port);
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
            // 4. MoveJ 运动到安全起点
            // ----------------------------------------------------------
            double dJ1 = 0; double dJ2 = -90; double dJ3 = 90;
            double dJ4 = -90; double dJ5 = -90; double dJ6 = 0;

            ret = robot.HRIF_MoveJ(BOX_ID, RBT_ID, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, "TCP", "Base",
                                   30.0, 60.0, 0.0, 1, 0, 0, 0, "Jog_start");
            print_ret(robot, "HRIF_MoveJ", ret);
            if (ret != 0 || !wait_motion_done(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 5. 单轴关节相对运动：J1 轴正向旋转 10°
            // ----------------------------------------------------------
            int nAxisId = 0;
            int nDirection = 1;
            double dDistance = 10.0;

            ret = robot.HRIF_MoveRelJ(BOX_ID, RBT_ID, nAxisId, nDirection, dDistance);
            print_ret(robot, "HRIF_MoveRelJ(J1 +10 deg)", ret);
            if (ret != 0 || !wait_motion_done(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 6. MoveRelL：沿当前 UCS X 正方向移动 30 mm
            // ----------------------------------------------------------
            int nToolMotion = 0;
            nAxisId = 0;
            nDirection = 1;
            dDistance = 30.0;

            ret = robot.HRIF_MoveRelL(BOX_ID, RBT_ID, nAxisId, nDirection, dDistance, nToolMotion);
            print_ret(robot, "HRIF_MoveRelL(X +30 mm)", ret);
            if (ret != 0 || !wait_motion_done(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 7. WayPointRel：叠加值模式，X、Y 同时正向移动 30 mm
            // ----------------------------------------------------------
            int nType = 1;
            int nPointList = 0;
            int nrelMoveType = 1;

            ret = robot.HRIF_WayPointRel(BOX_ID, RBT_ID, nType, nPointList, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                         nrelMoveType, 1, 1, 0, 0, 0, 0, 30.0, 30.0, 0, 0, 0, 0,
                                         "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPointRel_XY_with_radius");
            print_ret(robot, "HRIF_WayPointRel(X +30 mm, Y +30 mm)", ret);
            if (ret != 0 || !wait_motion_done(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 8. WayPointRel：叠加值模式，X 反向、Y 正向移动 30 mm
            // ----------------------------------------------------------
            ret = robot.HRIF_WayPointRel(BOX_ID, RBT_ID, nType, nPointList, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                         nrelMoveType, 1, 1, 0, 0, 0, 0, -30.0, 30.0, 0, 0, 0, 0,
                                         "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPointRel_XY_without_radius");
            print_ret(robot, "HRIF_WayPointRel(X -30 mm, Y +30 mm)", ret);
            if (ret != 0 || !wait_motion_done(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 9. WayPointRel：叠加值模式，J1、J4 同时旋转 10°
            // ----------------------------------------------------------
            nType = 0;
            ret = robot.HRIF_WayPointRel(BOX_ID, RBT_ID, nType, nPointList, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                         nrelMoveType, 1, 0, 0, 1, 0, 0, 10.0, 0, 0, 10.0, 0, 0,
                                         "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPointRel_J1_J2");
            print_ret(robot, "HRIF_WayPointRel(J1 +10 deg, J4 +10 deg)", ret);
            if (ret != 0 || !wait_motion_done(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 10. 断开连接
            // ----------------------------------------------------------
            Console.WriteLine("[OK] Relative motion completed.");
            ret = robot.HRIF_DisConnect(BOX_ID);
            print_ret(robot, "HRIF_DisConnect", ret);
            return ret == 0 ? 0 : 1;
        }
    }
}
