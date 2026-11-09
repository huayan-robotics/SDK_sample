using RobotLibrarys;
using System;
using System.Diagnostics;
using System.Threading;

namespace showcase_servoJPro
{
    internal class Program
    {
        // BoxID 和 RbtID 可以在此处进行修改
        private const int BOX_ID = 0;
        private const int RBT_ID = 0;

        private static bool PrintRet(RobotAPI robot, string operation, int ret)
        {
            if (ret == 0)
            {
                Console.WriteLine($"[OK] {operation}");
                return true;
            }

            string errorMessage = "";
            int lookupRet = robot.HRIF_GetErrorCodeStr(BOX_ID, ret, ref errorMessage);
            Console.WriteLine("[ERROR] " + operation + " failed (ret=" + ret + ")" +
                (lookupRet == 0 ? ": " + errorMessage : ""));

            return false;
        }

        private static void SleepUntil(Stopwatch timer, double targetSeconds)
        {
            while (true)
            {
                double remainingMs = (targetSeconds - timer.Elapsed.TotalSeconds) * 1000.0;
                if (remainingMs <= 0.0)
                {
                    return;
                }

                if (remainingMs > 2.0)
                {
                    Thread.Sleep((int)remainingMs - 1);
                }
                else
                {
                    Thread.Yield();
                }
            }
        }

        private static int Main(string[] args)
        {
            Console.WriteLine("=== ServoJ 6-Axis Joint-Space Online Control Example ===");

            // ----------------------------------------------------------
            // 1. 连接机器人
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
            int state = 0;
            ret = robot.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, ref state);
            PrintRet(robot, $"HRIF_ReadCurFSM (FSM={state})", ret);
            if (ret != 0 || state != 33)
            {
                Console.WriteLine($"[ERR] Robot must be in Standby(33), FSM={state}");
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 3. 设置速度比并 MoveJ 运动到安全起始位姿
            // ----------------------------------------------------------
            ret = robot.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
            if (!PrintRet(robot, "HRIF_SetOverride", ret))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return ret;
            }

            Pose pose = new Pose();
            JointsData safeJoints = new JointsData();
            safeJoints.joint[0] = -90; safeJoints.joint[1] = -42.5;
            safeJoints.joint[2] = 0; safeJoints.joint[3] = 72.5;
            safeJoints.joint[4] = 0; safeJoints.joint[5] = -65;
            safeJoints.joint[6] = 0;
            Console.WriteLine("[INFO] Moving to a safe position with MoveJ...");
            ret = robot.HRIF_MoveJ_nJ(
                BOX_ID, RBT_ID, pose, safeJoints,
                "TCP", "Base", 30.0, 60.0, 0.0, 1, 0, 0, 0, "servo_home");
            PrintRet(robot, "HRIF_MoveJ_nJ", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            bool motionDone = false;
            Stopwatch motionTimer = Stopwatch.StartNew();
            do
            {
                Thread.Sleep(0);
                ret = robot.HRIF_IsMotionDone(BOX_ID, RBT_ID, ref motionDone);
                if (ret != 0)
                {
                    PrintRet(robot, "HRIF_IsMotionDone", ret);
                    robot.HRIF_DisConnect(BOX_ID);
                    return ret;
                }
            }
            while (!motionDone && motionTimer.Elapsed < TimeSpan.FromSeconds(60));
            if (!motionDone)
            {
                Console.WriteLine("[ERR] MoveJ timed out");
                robot.HRIF_GrpStop(BOX_ID, RBT_ID);
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }
            Console.WriteLine("[INFO] Safe position reached");

            // ----------------------------------------------------------
            // 4. 读取当前关节位置（MoveJ 后的基准）
            // ----------------------------------------------------------
            JointsData currentJoints = new JointsData();
            ret = robot.HRIF_ReadActJointPos_nJ(BOX_ID, RBT_ID, ref currentJoints);
            PrintRet(robot, "HRIF_ReadActJointPos", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 5. 启动 Servo 模式
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

            ret = robot.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, ref state);
            if (!PrintRet(robot, "HRIF_ReadCurFSM", ret))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return ret;
            }

            Console.WriteLine($"[INFO] Robot state after starting servo: {state}");
            Thread.Sleep(1);

            // ----------------------------------------------------------
            // 6. 先推送当前关节位置（同步，避免首帧突变）
            // ----------------------------------------------------------
            ret = robot.HRIF_PushServoJ_nJ(BOX_ID, RBT_ID, currentJoints);
            PrintRet(robot, "HRIF_PushServoJ (sync)", ret);

            // ----------------------------------------------------------
            // 7. 基于当前关节位置生成正弦轨迹并推送
            // ----------------------------------------------------------
            const double AMP = 5.0;              // 振幅：5°
            const double FREQ = 0.5;             // 频率：0.5Hz
            const double DURATION = 2.0;         // 持续时间：2s
            const int JOINT_COUNT = 7;
            double[] direction = { 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
            Stopwatch trajectoryTimer = Stopwatch.StartNew();
            double maximumPushMs = 0.0;
            double totalPushMs = 0.0;
            int successCount = 0;
            int failureCount = 0;

            Console.WriteLine(
                $"[INFO] Streaming a sine trajectory (stiffness={STIFFNESS_FACTOR}, " +
                $"amplitude={AMP} deg, duration={DURATION}s)...");

            const double PUSH_PERIOD = 0.004; // 4 ms（250 Hz）
            int sample = 1;
            while (trajectoryTimer.Elapsed.TotalSeconds < DURATION)
            {
                SleepUntil(trajectoryTimer, sample * PUSH_PERIOD);
                double t = sample++ * PUSH_PERIOD;

                double offset = AMP * Math.Sin(2.0 * Math.PI * FREQ * t);
                JointsData target = new JointsData();
                for (int j = 0; j < JOINT_COUNT; ++j)
                {
                    target.joint[j] = currentJoints.joint[j] + direction[j] * offset;
                }

                Stopwatch pushTimer = Stopwatch.StartNew();
                ret = robot.HRIF_PushServoJ_nJ(BOX_ID, RBT_ID, target);
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
            // 8. 停止并断开连接
            // ----------------------------------------------------------
            ret = robot.HRIF_GrpStop(BOX_ID, RBT_ID);
            PrintRet(robot, "HRIF_GrpStop", ret);
            ret = robot.HRIF_DisConnect(BOX_ID);
            PrintRet(robot, "HRIF_DisConnect", ret);
            return ret;
        }
    }
}
