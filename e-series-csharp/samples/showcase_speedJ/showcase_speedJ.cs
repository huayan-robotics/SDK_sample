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
            const double speedOverride = 0.2;
            ret = robot.HRIF_SetOverride(BOX_ID, RBT_ID, speedOverride);
            PrintRet(robot, "HRIF_SetOverride", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            double dJ1 = 0; double dJ2 = 0; double dJ3 = 90;
            double dJ4 = 0; double dJ5 = 90; double dJ6 = 0;

            ret = robot.HRIF_MoveJ(BOX_ID, RBT_ID, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,"TCP", "Base", 
                                    30.0, 60.0, 0.0, 1, 0, 0, 0, "SpeedJ_start");
            PrintRet(robot, "HRIF_MoveJ", ret);
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
            // 4. SpeedJ J1、J5 联动，近似在基坐标系 XY 水平面画小圆
            // ----------------------------------------------------------
            const double j1Amplitude = 12.0;         // J1 角度波形幅值，deg
            const double j5Amplitude = 12.0;        // J5 角度波形幅值，deg
            const double period = 7.5;              // 一圈的运行时间，s
            const double interval = 0.05;           // SpeedJ 指令更新周期，s
            int sampleCount = (int)Math.Round(period / interval);
            double dt = period / sampleCount;
            double omega = 2.0 * Math.PI / period;
            const double jointAcc = 20.0;           // 关节加速度，deg/s^2
            double runtime = dt * 2.0;              // 指令有效时间，需大于 dt
            // SpeedJ 不受 SetOverride 影响，速度由幅值和周期决定；速度比仅用于 MoveJ。

            Console.WriteLine(
                "[INFO] Starting SpeedJ approximate XY circle: " +
                $"period={period} s, J1 amplitude={j1Amplitude} deg, J5 amplitude={j5Amplitude} deg, " +
                $"peak J1/J5 speed={j1Amplitude * omega}/{j5Amplitude * omega} deg/s");

            // ----------------------------------------------------------
            // 5. 连续下发 J1、J5 正余弦速度
            // ----------------------------------------------------------
            Stopwatch sendTimer = Stopwatch.StartNew();

            for (int sample = 0; sample < sampleCount; ++sample)
            {
                double time = sample * dt;
                double j1Velocity = j1Amplitude * omega * Math.Cos(omega * time);
                double j5Velocity = j5Amplitude * omega * Math.Sin(omega * time);

                ret = robot.HRIF_SpeedJ(BOX_ID, RBT_ID, j1Velocity, 0.0, 0.0, 0.0, j5Velocity, 0.0,
                                        jointAcc, runtime);

                if (ret != 0)
                {
                    PrintRet(robot, "HRIF_SpeedJ", ret);
                    robot.HRIF_GrpStop(BOX_ID, RBT_ID);
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }

                WaitUntil(sendTimer, (sample + 1) * dt);
            }

            // ----------------------------------------------------------
            // 6. 发送零关节速度，使机器人平稳停止。
            // ----------------------------------------------------------
            ret = robot.HRIF_SpeedJ(BOX_ID, RBT_ID, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    jointAcc, 0.5);
            PrintRet(robot, "HRIF_SpeedJ(stop)", ret);
            int commandRet = ret;

            Thread.Sleep(500);
            Console.WriteLine("[OK] SpeedJ motion completed.");

            int stopRet = robot.HRIF_GrpStop(BOX_ID, RBT_ID);
            PrintRet(robot, "HRIF_GrpStop", stopRet);
            int disconnectRet = robot.HRIF_DisConnect(BOX_ID);
            PrintRet(robot, "HRIF_DisConnect", disconnectRet);
            return commandRet == 0 && stopRet == 0 && disconnectRet == 0 ? 0 : 1;
        }
    }
}
