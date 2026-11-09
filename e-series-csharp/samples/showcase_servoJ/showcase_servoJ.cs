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
            Console.WriteLine("=== ServoJ 6-Axis Joint-Space Online Control Example ===");

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

            double dX = 0.0;  double dY = 0.0;  double dZ = 0.0;
            double dRx = 0.0; double dRy = 0.0; double dRz = 0.0;

            double dJ1 = 0; double dJ2 = 0; double dJ3 = 90;
            double dJ4 = 0; double dJ5 = 90; double dJ6 = 0;

            Console.WriteLine("[INFO] Moving to a safe position with MoveJ...");
            ret = robot.HRIF_MoveJ(
                BOX_ID, RBT_ID,
                dX, dY, dZ, dRx, dRy, dRz,
                dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
                "TCP", "Base",
                30.0, 60.0, 0.0,
                1, 0, 0, 0,
                "servo_home");
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
                Thread.Sleep(20);
                ret = robot.HRIF_IsMotionDone(BOX_ID, RBT_ID, ref motionDone);
                if (ret != 0) break;
            }
            if (ret != 0 || !motionDone)
            {
                if (ret != 0) PrintRet(robot, "HRIF_IsMotionDone", ret);
                else Console.WriteLine("[ERR] Motion completion timed out");
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }
            Console.WriteLine("[INFO] Safe position reached");

            // ----------------------------------------------------------
            // 4. MoveJ完成后读取实际关节角，作为轨迹基准。
            // ----------------------------------------------------------
            const int JOINT_COUNT = 6;
            double[] currentJoints = new double[JOINT_COUNT];
            ret = robot.HRIF_ReadActJointPos(
                BOX_ID, RBT_ID,
                ref currentJoints[0], ref currentJoints[1],
                ref currentJoints[2], ref currentJoints[3],
                ref currentJoints[4], ref currentJoints[5]);
            PrintRet(robot, "HRIF_ReadActJointPos", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 5. 启动ServoJ 伺服周期模式。
            // ----------------------------------------------------------
            const double SERVO_TIME = 0.01;      // 伺服周期：10ms
            const double LOOKAHEAD_TIME = 0.05;  // 前瞻时间：50ms
            Console.WriteLine($"[INFO] Servo cycle={SERVO_TIME}s, lookahead={LOOKAHEAD_TIME}s");
            ret = robot.HRIF_StartServo(
                BOX_ID, RBT_ID, SERVO_TIME, LOOKAHEAD_TIME);
            PrintRet(robot, "HRIF_StartServo", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            Thread.Sleep(1);

            // ----------------------------------------------------------
            // 6. 首先推送首帧，避免发生跳变。
            // ----------------------------------------------------------
            ret = robot.HRIF_PushServoJ(
                BOX_ID, RBT_ID,
                currentJoints[0], currentJoints[1], currentJoints[2],
                currentJoints[3], currentJoints[4], currentJoints[5]);
            PrintRet(robot, "HRIF_PushServoJ (sync)", ret);

            // ----------------------------------------------------------
            // 7. 生成并按固定周期推送正弦轨迹。
            // ----------------------------------------------------------
            const double AMP = 5.0;              // 振幅：5°
            const double FREQ = 0.5;             // 频率：0.5Hz
            const double DURATION = 2.0;         // 持续时间：2s
            double[] direction = { 1.0, 1.0, 1.0, 1.0, 1.0, 0.0 };
            int total = (int)(DURATION / SERVO_TIME);

            Console.WriteLine(
                $"[INFO] Streaming {total} points " +
                $"(multi-joint sine, +/-{AMP} deg)...");

            Stopwatch trajectoryTimer = Stopwatch.StartNew();
            double maximumPushMs = 0.0;
            double totalPushMs = 0.0;
            int successCount = 0;
            int failureCount = 0;

            for (int i = 1; i <= total; ++i)
            {
                SleepUntil(trajectoryTimer, i * SERVO_TIME);
                double t = i * SERVO_TIME;
                double sineOffset = AMP * Math.Sin(2.0 * Math.PI * FREQ * t);

                double[] target = new double[JOINT_COUNT];
                for (int joint = 0; joint < JOINT_COUNT; ++joint)
                {
                    target[joint] =
                        currentJoints[joint] + direction[joint] * sineOffset;
                }

                Stopwatch pushTimer = Stopwatch.StartNew();
                ret = robot.HRIF_PushServoJ(
                    BOX_ID, RBT_ID,
                    target[0], target[1], target[2],
                    target[3], target[4], target[5]);
                pushTimer.Stop();
                double pushMs = pushTimer.Elapsed.TotalMilliseconds;

                maximumPushMs = Math.Max(maximumPushMs, pushMs);
                totalPushMs += pushMs;

                if (ret != 0)
                {
                    ++failureCount;
                    if (failureCount <= 3)
                    {
                        Console.Error.WriteLine(
                            $"[ERR] Point {i} failed, code={ret}, " +
                            $"push time={pushMs:F3}ms");
                    }
                }
                else
                {
                    ++successCount;
                }
            }

            int pushCount = successCount + failureCount;
            double averagePushMs = pushCount > 0 ? totalPushMs / pushCount : 0.0;
            Console.WriteLine(
                $"[INFO] Completed: succeeded={successCount}, failed={failureCount}, " +
                $"average push time={averagePushMs:F3}ms, " +
                $"maximum push time={maximumPushMs:F3}ms");

            // ----------------------------------------------------------
            // 8. 停止并断开连接。
            // ----------------------------------------------------------
            Console.WriteLine("[INFO] ServoJ completed; disconnecting");
            ret = robot.HRIF_GrpStop(BOX_ID, RBT_ID);
            PrintRet(robot, "HRIF_GrpStop", ret);
            ret = robot.HRIF_DisConnect(BOX_ID);
            PrintRet(robot, "HRIF_DisConnect", ret);
            return ret == 0 ? 0 : 1;
        }
    }
}
