using RobotLibrarys;
using System;
using System.Diagnostics;
using System.Threading;

namespace HuayanRobotExamples
{
    internal static class SpeedJExample
    {
        // BoxID 和 RbtID 可以在此处进行修改
        private const int BOX_ID = 0;
        private const int RBT_ID = 0;

        private static void PrintRet(RobotAPI robot, string command, int ret)
        {
            if (ret == 0)
            {
                Console.WriteLine($"[OK]  {command}");
            }
            else
            {
                string errorMessage = "";
                int lookupRet = robot.HRIF_GetErrorCodeStr(BOX_ID, ret, ref errorMessage);
                Console.WriteLine("[ERR] " + command + " -> " + ret + (lookupRet == 0 ? ": " + errorMessage : ""));
            }
        }

        private static void WaitUntil(Stopwatch timer, double targetSeconds)
        {
            while (true)
            {
                double remainingSeconds = targetSeconds - timer.Elapsed.TotalSeconds;
                if (remainingSeconds <= 0.0)
                {
                    return;
                }

                if (remainingSeconds > 0.002)
                {
                    Thread.Sleep(TimeSpan.FromSeconds(remainingSeconds - 0.001));
                }
                else
                {
                    Thread.SpinWait(100);
                }
            }
        }

        public static int Main(string[] args)
        {
            Console.WriteLine("======== SpeedJ Online Joint Velocity Control Example ========");

            RobotAPI robot = new RobotAPI();

            // ----------------------------------------------------------
            // 1. 连接机器人
            // ----------------------------------------------------------
            // 需根据机器人的实际 IP 进行修改
            const string ip = "192.168.0.10";
            const int port = 10003;

            int ret = robot.HRIF_Connect(BOX_ID, ip, port);
            PrintRet(robot, "HRIF_Connect", ret);
            if (ret != 0)
            {
                return 1;
            }

            // ----------------------------------------------------------
            // 2. 确认机器人处于 Standby(33)。
            // ----------------------------------------------------------
            int currentFsm = 0;
            ret = robot.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, ref currentFsm);
            PrintRet(robot, $"HRIF_ReadCurFSM (FSM={currentFsm})", ret);
            if (ret != 0 || currentFsm != 33)
            {
                Console.WriteLine($"[ERR] Robot must be in Standby(33), FSM={currentFsm}");
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 3. 设置速度比并 MoveJ 运动到安全起始位姿
            // ----------------------------------------------------------
            ret = robot.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
            PrintRet(robot, "HRIF_SetOverride", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            Pose pose = new Pose ();
            JointsData safeJoints = new JointsData();
            safeJoints.joint[0] = 90; safeJoints.joint[1] = 45;
            safeJoints.joint[2] = 0; safeJoints.joint[3] = -80;
            safeJoints.joint[4] = 0; safeJoints.joint[5] = 35;
            safeJoints.joint[6] = 0;
            Console.WriteLine("[INFO] Moving to a safe position with MoveJ...");
            ret = robot.HRIF_MoveJ_nJ(BOX_ID, RBT_ID, pose, safeJoints,
                "TCP", "Base", 30.0, 60.0, 0.0, 1, 0, 0, 0, "servo_home");
            PrintRet(robot, "HRIF_MoveJ_nJ", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            bool motionDone = false;
            Stopwatch motionTimer = Stopwatch.StartNew();
            while (!motionDone && motionTimer.Elapsed < TimeSpan.FromSeconds(60))
            {
                ret = robot.HRIF_IsMotionDone(BOX_ID, RBT_ID, ref motionDone);
                if (ret != 0)
                {
                    PrintRet(robot, "HRIF_IsMotionDone", ret);
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }

                Thread.Sleep(20);
            }
            if (!motionDone)
            {
                Console.WriteLine("[ERR] MoveJ timed out");
                robot.HRIF_GrpStop(BOX_ID, RBT_ID);
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 4. SpeedJ J1、J2 关节画圆的相关参数
            // ----------------------------------------------------------
            const double period = 8.0;              // 完成一周所需时间，s
            const double tAmp = 10.0;               // 目标关节空间圆半径，deg
            const double interval = 0.05;           // SpeedJ 指令更新周期，s
            double omega = 2.0 * Math.PI / period;
            const double jointAcc = 50.0;            // 关节加速度，deg/s^2
            const double runtime = interval * 2.0;   // 指令运行时间，s
            int sampleCount = (int)(period / interval);

            Console.WriteLine(
                "[INFO] Starting continuous SpeedJ command streaming: " +
                $"period={period} s, target amplitude={tAmp} deg, ");

            // ----------------------------------------------------------
            // 5. 连续下发关节速度，绘制一个圆形轨迹
            // ----------------------------------------------------------
            Stopwatch sendTimer = Stopwatch.StartNew();
            JointsData jointsVel = new JointsData();
            for (int sample = 0; sample < sampleCount; ++sample)
            {
                double time = sample * interval;
                jointsVel.joint[0] = tAmp * omega * Math.Cos(omega * time);
                jointsVel.joint[1] = -tAmp * omega * Math.Sin(omega * time);

                ret = robot.HRIF_SpeedJ_nJ(BOX_ID, RBT_ID, jointsVel, jointAcc, runtime);

                if (ret != 0)
                {
                    PrintRet(robot, "HRIF_SpeedJ", ret);
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }

                WaitUntil(sendTimer, (sample + 1) * interval);
            }

            // ----------------------------------------------------------
            // 6. 发送零关节速度，使机器人平稳停止。
            // ----------------------------------------------------------
            jointsVel.joint[0] = 0;
            jointsVel.joint[1] = 0;
            ret = robot.HRIF_SpeedJ_nJ(BOX_ID, RBT_ID, jointsVel, jointAcc, 0.5);
            PrintRet(robot, "HRIF_SpeedJ(stop)", ret);

            Thread.Sleep(500);
            Console.WriteLine("[OK] SpeedJ motion completed.");

            ret = robot.HRIF_GrpStop(BOX_ID, RBT_ID);
            PrintRet(robot, "HRIF_GrpStop", ret);
            robot.HRIF_DisConnect(BOX_ID);
            return ret == 0 ? 0 : 1;
        }
    }
}
