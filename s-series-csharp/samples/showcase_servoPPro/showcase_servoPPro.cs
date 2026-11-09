using RobotLibrarys;
using System;
using System.Diagnostics;
using System.Threading;

namespace servo_program
{
    class csharp_servo_program
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

        private static void PrintPose(
            string label,
            double x, double y, double z,
            double rx, double ry, double rz)
        {
            Console.WriteLine(
                $"{label}: X={x}, Y={y}, Z={z}, " +
                $"Rx={rx}, Ry={ry}, Rz={rz}");
        }

        private static void SleepUntil(Stopwatch timer, double targetSeconds)
        {
            while (true)
            {
                double remainingSeconds = targetSeconds - timer.Elapsed.TotalSeconds;
                if (remainingSeconds <= 0.0)
                {
                    return;
                }

                // 最后1ms采用短暂自旋等待，避免重复调用Thread.Sleep
                // 产生较大的累计时间误差。
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

        public static int Main()
        {
            Console.WriteLine("=== ServoP Cartesian Online Control Example ===");

            // ----------------------------------------------------------
            // 1. 连接机器人。
            // ----------------------------------------------------------
            RobotAPI robot = new RobotAPI();

            // 需根据机器人的实际 IP 进行修改
            string host = "192.168.0.10";
            int port = 10003;

            int ret = robot.HRIF_Connect(BOX_ID, host, port);
            PrintRet(robot, "HRIF_Connect", ret);
            if (ret != 0)
            {
                return 1;
            }

            // ----------------------------------------------------------
            // 2. 确认机器人处于 Standby(33)。
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
            // 3. 设置速度比并 MoveJ 运动到安全起始位姿
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
                "TCP", "Base",
                30.0, 60.0, 0.0,
                1, 0, 0, 0,
                "servoP_home");
            PrintRet(robot, "HRIF_MoveJ", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            bool motionDone = false;
            Stopwatch motionTimer = Stopwatch.StartNew();
            do
            {
                Thread.Sleep(50);
                ret = robot.HRIF_IsMotionDone(BOX_ID, RBT_ID, ref motionDone);
            }
            while (ret == 0 && !motionDone && motionTimer.Elapsed < TimeSpan.FromSeconds(60));
            if (ret != 0 || !motionDone)
            {
                if (ret != 0) PrintRet(robot, "HRIF_IsMotionDone", ret);
                else Console.WriteLine("[ERR] MoveJ timed out");
                robot.HRIF_GrpStop(BOX_ID, RBT_ID);
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }
            Console.WriteLine("[INFO] Safe position reached");

            // ----------------------------------------------------------
            // 4. 读取当前TCP位姿，作为ServoP轨迹基准。
            // ----------------------------------------------------------
            double dX = 0.0; double dY = 0.0; double dZ = 0.0;
            double dRx = 0.0; double dRy = 0.0; double dRz = 0.0;
            ret = robot.HRIF_ReadActTcpPos(
                BOX_ID, RBT_ID,
                ref dX, ref dY, ref dZ,
                ref dRx, ref dRy, ref dRz);
            PrintRet(robot, "HRIF_ReadActTcpPos", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }
            PrintPose("[INFO] Current TCP", dX, dY, dZ, dRx, dRy, dRz);

            // ----------------------------------------------------------
            // 5. 启动ServoP 刚度系数模式。
            // ----------------------------------------------------------
            const double STIFFNESS_FACTOR = 8000;  // 刚度系数 8000
            const double LOOKAHEAD_TIME = 0.05;  // 前瞻时间：50ms
            Console.WriteLine($"[INFO] Servo stiffness={STIFFNESS_FACTOR}s, lookahead={LOOKAHEAD_TIME}s");
            ret = robot.HRIF_StartServo(
                BOX_ID, RBT_ID, STIFFNESS_FACTOR, LOOKAHEAD_TIME);
            PrintRet(robot, "HRIF_StartServo", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // 最多等待3s，确认StartServo后进入Moving(25)。
            Stopwatch movingTimer = Stopwatch.StartNew();
            do
            {
                Thread.Sleep(10);
                ret = robot.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, ref nFSM);
            }
            while (nFSM != 25 && movingTimer.Elapsed.TotalSeconds < 3.0);
            Console.WriteLine($"[INFO] FSM after StartServo={nFSM}");
            if (ret != 0 || nFSM != 25)
            {
                if (ret != 0) PrintRet(robot, "HRIF_ReadCurFSMFromCPS", ret);
                else Console.WriteLine("[ERR] Failed to enter Moving(25)");
                robot.HRIF_GrpStop(BOX_ID, RBT_ID);
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 6. 首先推送当前TCP位姿，避免首帧发生跳变。
            // ----------------------------------------------------------
            ret = robot.HRIF_PushServoP(
                BOX_ID, RBT_ID,
                dX, dY, dZ, dRx, dRy, dRz,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // 默认 TCP（全 0）
            PrintRet(robot, "HRIF_PushServoP (sync)", ret);

            // ----------------------------------------------------------
            // 7. 在Z方向生成并按固定周期推送正弦轨迹。
            // ----------------------------------------------------------
            const double AMP = 20.0;             // 振幅：20mm
            const double FREQ = 0.5;             // 频率：0.5Hz
            const double DURATION = 2.0;         // 持续时间：2s
            const double PUSH_PERIOD = 0.004;     // 推送周期：4ms（250 Hz）
            double[] direction = { 1.0, 1.0, 1.0, 1.0, 1.0, 0.0 };
            Stopwatch trajectoryTimer = Stopwatch.StartNew();
            int sample = 1;
            double maximumPushMs = 0.0;
            double totalPushMs = 0.0;
            int successCount = 0;
            int failureCount = 0;

            Console.WriteLine(
                $"[INFO] Streaming a sine trajectory (stiffness={STIFFNESS_FACTOR}, " +
                $"amplitude={AMP} deg, duration={DURATION}s)...");

            while (trajectoryTimer.Elapsed.TotalSeconds < DURATION)
            {
                SleepUntil(trajectoryTimer, sample * PUSH_PERIOD);
                double t = sample++ * PUSH_PERIOD;

                double targetX = dX;
                double targetY = dY;
                double targetZ = dZ + AMP * Math.Sin(2.0 * Math.PI * FREQ * t);
                Stopwatch pushTimer = Stopwatch.StartNew();
                ret = robot.HRIF_PushServoP(
                    BOX_ID, RBT_ID,
                    targetX, targetY, targetZ, dRx, dRy, dRz,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // 默认 TCP（全 0）
                pushTimer.Stop();

                double pushMs = pushTimer.Elapsed.TotalMilliseconds;
                if (pushMs > maximumPushMs)
                {
                    maximumPushMs = pushMs;
                }

                totalPushMs += pushMs;
                if (ret == 0)
                {
                    ++successCount;
                }
                else
                {
                    ++failureCount;
                }
            }

            int totalCount = successCount + failureCount;
            double averagePushMs = totalCount > 0 ? totalPushMs / totalCount : 0.0;
            Console.WriteLine(
                $"[INFO] Completed: succeeded={successCount}, failed={failureCount}, " +
                $"average push time={averagePushMs:F3}ms, " +
                $"maximum push time={maximumPushMs:F3}ms");

            // ----------------------------------------------------------
            // 8. 停止并断开连接。
            // ----------------------------------------------------------
            Console.WriteLine("[INFO] ServoP completed; disconnecting");
            ret = robot.HRIF_GrpStop(BOX_ID, RBT_ID);
            PrintRet(robot, "HRIF_GrpStop", ret);
            ret = robot.HRIF_DisConnect(BOX_ID);
            PrintRet(robot, "HRIF_DisConnect", ret);
            return ret == 0 ? 0 : 1;
        }
    }
}
