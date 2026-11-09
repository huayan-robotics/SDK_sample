using RobotLibrarys;
using System;
using System.Diagnostics;
using System.Threading;

namespace HuayanRobotExamples
{
    internal static class SpeedLExample
    {
        // BoxID 和 RbtID 可以在此处进行修改
        private const int BOX_ID = 0;
        private const int RBT_ID = 0;

        private static void print_ret(RobotAPI robot, string cmd, int ret)
        {
            if (ret == 0)
            {
                Console.WriteLine($"[OK]  {cmd}");
            }
            else
            {
                string errMsg = "";
                int lookupRet = robot.HRIF_GetErrorCodeStr(BOX_ID, ret, ref errMsg);
                Console.WriteLine("[ERR] " + cmd + " -> " + ret + (lookupRet == 0 ? ": " + errMsg : ""));
            }
        }

        private static void sleep_until(
            Stopwatch nextDirectionTime,
            double targetSeconds)
        {
            while (true)
            {
                double remainingSeconds =
                    targetSeconds - nextDirectionTime.Elapsed.TotalSeconds;
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
            Console.WriteLine("======== SpeedL Online Cartesian Velocity Control Example ========");

            RobotAPI robot = new RobotAPI();

            // ----------------------------------------------------------
            // 1. 连接机器人
            // ----------------------------------------------------------
            // 需根据机器人的实际 IP 进行修改
            const string ip = "192.168.0.10";
            const int port = 10003;

            int ret = robot.HRIF_Connect(BOX_ID, ip, port);
            print_ret(robot, "HRIF_Connect", ret);
            if (ret != 0)
            {
                return 1;
            }

            // ----------------------------------------------------------
            // 2. 确认机器人处于 Standby(33)。
            // ----------------------------------------------------------
            int nFSM = 0;
            ret = robot.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, ref nFSM);
            print_ret(robot, $"HRIF_ReadCurFSM (FSM={nFSM})", ret);
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
            print_ret(robot, "HRIF_SetOverride", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            double dJ1 = 0; double dJ2 = -90; double dJ3 = 90;
            double dJ4 = -90; double dJ5 = -90; double dJ6 = 0;

            ret = robot.HRIF_MoveJ(BOX_ID, RBT_ID, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, "TCP", "Base", 
                                   30.0, 60.0, 0.0, 1, 0, 0, 0, "SpeedL_start");
            print_ret(robot, "HRIF_MoveJ", ret);
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
                    print_ret(robot, "HRIF_IsMotionDone", ret);
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
            // 4. SpeedL 画正六边形的相关参数
            // ----------------------------------------------------------
            const double linearSpeed = 25.0;      // SpeedL 指令线速度，mm/s
            const double linearAcc = 50.0;        // TCP 线加速度，mm/s^2
            const double angularAcc = 100.0;       // TCP 角加速度，deg/s^2
            const double holdTime = 4.0;  // 每个方向保持时间，s
            const double runtime = 5.0;     // 必须大于方向保持时间

            const double SQRT3_OVER_2 = 0.8660254037844386;
            double[,] directions =
            {
                { 1.0,  0.0 },
                { 0.5,  SQRT3_OVER_2 },
                {-0.5,  SQRT3_OVER_2 },
                {-1.0,  0.0 },
                {-0.5, -SQRT3_OVER_2 },
                { 0.5, -SQRT3_OVER_2 }
            };

            Console.WriteLine(
                "[INFO] Starting continuous SpeedL command streaming" +
                $", commanded speed={linearSpeed} mm/s" +
                $", maintain direction={holdTime} s");

            // ----------------------------------------------------------
            // 5. 连续下发空间速度，绘制一个正六边形
            // ----------------------------------------------------------
            Stopwatch nextDirectionTime = Stopwatch.StartNew();

            for (int i = 0; i < directions.GetLength(0); ++i)
            {
                double vx = linearSpeed * directions[i, 0];
                double vy = linearSpeed * directions[i, 1];

                Console.WriteLine(
                    $"[INFO] Commanded direction {i + 1}: " +
                    $"Vx={vx} mm/s, Vy={vy} mm/s");

                ret = robot.HRIF_SpeedL(BOX_ID,RBT_ID, vx, vy,0.0,0.0,0.0,0.0,
                                        linearAcc, angularAcc, runtime);
                print_ret(robot, $"HRIF_SpeedL(direction {i + 1})",ret);

                if (ret != 0)
                {
                    robot.HRIF_DisConnect(BOX_ID);
                    return 1;
                }

                sleep_until(nextDirectionTime, (i + 1) * holdTime);
            }

            // ----------------------------------------------------------
            // 6. 发送零关节速度，使机器人平稳停止。
            // ----------------------------------------------------------
            const double stopRuntime = 1.0;
            ret = robot.HRIF_SpeedL(BOX_ID,RBT_ID, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    linearAcc, angularAcc, stopRuntime);
            print_ret(robot, "HRIF_SpeedL(stop)", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            Thread.Sleep(TimeSpan.FromSeconds(stopRuntime));
            Console.WriteLine("[OK] SpeedL motion completed.");

            ret = robot.HRIF_GrpStop(BOX_ID, RBT_ID);
            print_ret(robot, "HRIF_GrpStop", ret);
            ret = robot.HRIF_DisConnect(BOX_ID);
            print_ret(robot, "HRIF_DisConnect", ret);
            return ret == 0 ? 0 : 1;
        }
    }
}
