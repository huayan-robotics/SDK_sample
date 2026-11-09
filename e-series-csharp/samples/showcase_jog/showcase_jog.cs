using RobotLibrarys;
using System;
using System.Diagnostics;
using System.Threading;

namespace HuayanRobotExamples
{
    internal static class JogExample
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
            Console.WriteLine("======== Jogging example started. ========");
            RobotAPI robot = new RobotAPI();

            // ----------------------------------------------------------
            // 1. 连接机器人，需根据机器人的实际 IP 进行修改
            // ----------------------------------------------------------
            const string host = "192.168.0.10";
            const int port = 10003;

            int ret = robot.HRIF_Connect(BOX_ID, host, port);
            print_ret(robot, "HRIF_Connect", ret);
            if (ret != 0) return 1;

            // ----------------------------------------------------------
            // 2. 确认机器人处于 Standby(33) 就绪状态
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
            double dJ1 = 0; double dJ2 = 0; double dJ3 = 90;
            double dJ4 = 0; double dJ5 = 90; double dJ6 = 0;

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
            // 5. 进行关节短点动，J1 轴正向运动 10°
            // ----------------------------------------------------------
            int nAxisID = 0;
            int nDirection = 1;
            for (int i = 0; i < 5; i++)
            {
                ret = robot.HRIF_ShortJogJ(BOX_ID, RBT_ID, nAxisID, nDirection);
                print_ret(robot, "HRIF_ShortJogJ", ret);
                if (ret != 0 || !wait_motion_done(robot))
                {
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }
            }

            // ----------------------------------------------------------
            // 6. 进行空间短点动，Z 轴正向运动 20 mm
            // ----------------------------------------------------------
            nAxisID = 2;
            for (int i = 0; i < 10; i++)
            {
                ret = robot.HRIF_ShortJogL(BOX_ID, RBT_ID, nAxisID, nDirection);
                print_ret(robot, "HRIF_ShortJogL", ret);
                if (ret != 0 || !wait_motion_done(robot))
                {
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }
            }

            // ----------------------------------------------------------
            // 7. 进行空间长点动，Z 轴反向运动；停止发送事件后等待停止
            // ----------------------------------------------------------
            int nState = 1;
            nDirection = 0;
            ret = robot.HRIF_LongJogL(BOX_ID, RBT_ID, nAxisID, nDirection, nState);
            print_ret(robot, "HRIF_LongJogL", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            for (int i = 0; i < 5; i++)
            {
                Thread.Sleep(100);
                ret = robot.HRIF_LongMoveEvent(BOX_ID, RBT_ID);
                print_ret(robot, i < 4 ? "HRIF_LongMoveEvent" : "HRIF_LongMoveEvent(stop)", ret);
                if (ret != 0)
                {
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }
            }
            if (!wait_motion_done(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 8. 进行关节长点动，J5 轴反向运动；主动发送 nState=0 停止
            // ----------------------------------------------------------
            nAxisID = 4;
            nDirection = 0;
            ret = robot.HRIF_LongJogJ(BOX_ID, RBT_ID, nAxisID, nDirection, nState);
            print_ret(robot, "HRIF_LongJogJ", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            Stopwatch tStart = Stopwatch.StartNew();
            while (true)
            {
                ret = robot.HRIF_LongMoveEvent(BOX_ID, RBT_ID);
                print_ret(robot, "HRIF_LongMoveEvent", ret);
                if (ret != 0)
                {
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }
                Thread.Sleep(100);

                TimeSpan tNow = tStart.Elapsed;
                double totalTime = tNow.TotalSeconds;
                if (totalTime > 2)
                {
                    nState = 0;
                    ret = robot.HRIF_LongJogJ(BOX_ID, RBT_ID, nAxisID, nDirection, nState);
                    print_ret(robot, "HRIF_LongJogJ(stop)", ret);
                    if (ret != 0 || !wait_motion_done(robot))
                    {
                        robot.HRIF_DisConnect(BOX_ID);
                        return 1;
                    }
                    break;
                }
            }

            // ----------------------------------------------------------
            // 9. 断开连接
            // ----------------------------------------------------------
            Console.WriteLine("[OK] Jogging Motion Completed.");
            ret = robot.HRIF_DisConnect(BOX_ID);
            print_ret(robot, "HRIF_DisConnect", ret);
            return ret == 0 ? 0 : 1;
        }
    }
}
