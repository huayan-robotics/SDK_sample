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

        private static void PrintRet(RobotAPI robot, string cmd, int ret)
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

        private static bool WaitMotionDone(RobotAPI robot)
        {
            Stopwatch deadline = Stopwatch.StartNew();
            while (deadline.Elapsed < TimeSpan.FromSeconds(60))
            {
                Thread.Sleep(500);
                bool done = false;
                int ret = robot.HRIF_IsMotionDone(BOX_ID, RBT_ID, ref done);
                if (ret != 0)
                {
                    PrintRet(robot, "HRIF_IsMotionDone", ret);
                    return false;
                }
                if (done) return true;
            }
            Console.Error.WriteLine("[ERR] Motion timeout (60s).");
            return false;
        }

        private static JointsData Joints(params double[] values)
        {
            JointsData data = new JointsData();
            for (int i = 0; i < values.Length; ++i) data.joint[i] = values[i];
            return data;
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
            PrintRet(robot, "HRIF_Connect", ret);
            if (ret != 0) return 1;

            // ----------------------------------------------------------
            // 2. 确认机器人处于 Standby(33)就绪状态
            // ----------------------------------------------------------
            int fsm = 0;
            ret = robot.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, ref fsm);
            PrintRet(robot, $"HRIF_ReadCurFSM (FSM={fsm})", ret);
            if (ret != 0 || fsm != 33)
            {
                Console.Error.WriteLine($"[ERR] Robot must be in Standby(33), FSM={fsm}");
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 3. 设置全局速度比为0.2
            // ----------------------------------------------------------       
            ret = robot.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
            PrintRet(robot, "HRIF_SetOverride", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 4. MoveJ运动到安全起点
            // ----------------------------------------------------------
            Pose zeroPose = new Pose();
            JointsData safeJoints = new JointsData();
            safeJoints.joint[0] = 90; safeJoints.joint[1] = 45;
            safeJoints.joint[2] = 0; safeJoints.joint[3] = -80;
            safeJoints.joint[4] = 0; safeJoints.joint[5] = 35;
            safeJoints.joint[6] = 0;
            ret = robot.HRIF_MoveJ_nJ(BOX_ID, RBT_ID, zeroPose, safeJoints, "TCP", "Base", 30.0, 60.0,
                                      0.0, 1, 0, 0, 0, "Jog_start");
            PrintRet(robot, "HRIF_MoveJ_nJ", ret);
            if (ret != 0 || !WaitMotionDone(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 5. 进行关节短点动，J3 轴反向运动 10°
            // ----------------------------------------------------------
            int axisId = 2;
            int direction = 0;
            for (int i = 0; i < 5; ++i)
            {
                ret = robot.HRIF_ShortJogJ(BOX_ID, RBT_ID, axisId, direction);
                PrintRet(robot, "HRIF_ShortJogJ", ret);
                if (ret != 0 || !WaitMotionDone(robot))
                {
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }
            }

            // ----------------------------------------------------------
            // 6. 进行空间短点动，X轴反向运动 20 mm
            // ----------------------------------------------------------
            axisId = 0;
            for (int i = 0; i < 10; ++i)
            {
                ret = robot.HRIF_ShortJogL(BOX_ID, RBT_ID, axisId, direction);
                PrintRet(robot, "HRIF_ShortJogL", ret);
                if (ret != 0 || !WaitMotionDone(robot))
                {
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }
            }

            // ----------------------------------------------------------
            // 7. 进行空间长点动，X轴反向运动；停止保活后等待控制器自动停止
            // ----------------------------------------------------------
            int state = 1;
            ret = robot.HRIF_LongJogL(BOX_ID, RBT_ID, axisId, direction, state);
            PrintRet(robot, "HRIF_LongJogL", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }
            for (int i = 0; i < 5; ++i)
            {
                Thread.Sleep(100);
                ret = robot.HRIF_LongMoveEvent(BOX_ID, RBT_ID);
                PrintRet(robot, i < 4 ? "HRIF_LongMoveEvent" : "HRIF_LongMoveEvent(stop)", ret);
                if (ret != 0)
                {
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }
            }
            if (!WaitMotionDone(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 8. 进行关节长点动，J4轴，按照启动长点动-持续下发机制-主动发送 nStat=0 的长点动命令停止
            // ----------------------------------------------------------
            axisId = 3;
            direction = 1;
            ret = robot.HRIF_LongJogJ(BOX_ID, RBT_ID, axisId, direction, state);
            PrintRet(robot, "HRIF_LongJogJ", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            Stopwatch start = Stopwatch.StartNew();
            while (true)
            {
                ret = robot.HRIF_LongMoveEvent(BOX_ID, RBT_ID);
                PrintRet(robot, "HRIF_LongMoveEvent", ret);
                if (ret != 0)
                {
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }
                Thread.Sleep(100);
                if (start.Elapsed.TotalSeconds > 2.5)
                {
                    state = 0;
                    ret = robot.HRIF_LongJogJ(BOX_ID, RBT_ID, axisId, direction, state);
                    PrintRet(robot, "HRIF_LongJogJ(stop)", ret);
                    if (ret != 0 || !WaitMotionDone(robot))
                    {
                        robot.HRIF_DisConnect(BOX_ID);
                        return 1;
                    }
                    break;
                }
            }

            Console.WriteLine("[OK] Jogging Motion Completed.");
            ret = robot.HRIF_DisConnect(BOX_ID);
            PrintRet(robot, "HRIF_DisConnect", ret);
            return ret == 0 ? 0 : 1;
        }
    }
}
