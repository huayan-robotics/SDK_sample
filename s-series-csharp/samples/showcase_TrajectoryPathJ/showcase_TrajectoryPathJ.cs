using RobotLibrarys;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Text;
using System.Threading;

namespace TrajectoryPathJExample
{
    internal static class Program
    {
        // BoxID 和 RbtID 可以在此处进行修改
        private const int BOX_ID = 0;
        private const int RBT_ID = 0;
        private const string PATH_NAME = "sdk_relative_path_j";

        private static bool PrintRet(RobotAPI robot, string command, int ret)
        {
            if (ret == 0)
            {
                Console.WriteLine("[OK]  " + command);
                return true;
            }

            string message = "";
            int lookupRet = robot.HRIF_GetErrorCodeStr(BOX_ID, ret, ref message);
            string suffix = lookupRet == 0 ? ": " + message : "";
            Console.Error.WriteLine("[ERR] {0} -> {1}{2}", command, ret, suffix);
            return false;
        }

        private static bool WaitMotion(RobotAPI robot, TimeSpan timeout)
        {
            Stopwatch timer = Stopwatch.StartNew();
            while (timer.Elapsed < timeout)
            {
                bool done = false;
                int ret = robot.HRIF_IsMotionDone(BOX_ID, RBT_ID, ref done);
                if (ret != 0)
                {
                    PrintRet(robot, "HRIF_IsMotionDone", ret);
                    return false;
                }
                if (done)
                {
                    return true;
                }
                Thread.Sleep(50);
            }

            Console.Error.WriteLine("[ERR] Motion timed out");
            return false;
        }

        private static bool WaitPathComputed(RobotAPI robot)
        {
            Stopwatch timer = Stopwatch.StartNew();
            int lastState = -1;
            while (timer.Elapsed < TimeSpan.FromMinutes(10))
            {
                int stateJ = 0;
                int errorJ = 0;
                int stateL = 0;
                int errorL = 0;
                int ret = robot.HRIF_ReadPathState(
                    BOX_ID, RBT_ID, PATH_NAME,
                    ref stateJ, ref errorJ, ref stateL, ref errorL
                );
                if (ret != 0)
                {
                    PrintRet(robot, "HRIF_ReadPathState", ret);
                    return false;
                }
                if (stateJ != lastState)
                {
                    Console.WriteLine("[INFO] PathJ state={0}", stateJ);
                    lastState = stateJ;
                }
                if (stateJ == 3)
                {
                    return true;
                }
                if (stateJ == 5)
                {
                    Console.Error.WriteLine("[ERR] PathJ computation failed, error={0}", errorJ);
                    return false;
                }
                Thread.Sleep(200);
            }

            Console.Error.WriteLine("[ERR] PathJ computation timed out");
            return false;
        }

        private static List<double> BuildRelativeCurve(double[] baseJoints)
        {
            const int pointCount = 13;
            var points = new List<double>(pointCount * 6);
            for (int index = 0; index < pointCount; ++index)
            {
                double phase = -0.5 * Math.PI + 2.0 * Math.PI * index / (pointCount - 1);
                points.Add(baseJoints[0] + 5.0 * Math.Sin(phase));
                points.Add(baseJoints[1] + 3.0 * (1.0 - Math.Cos(phase)));
                points.Add(baseJoints[2]);
                points.Add(baseJoints[3]);
                points.Add(baseJoints[4]);
                points.Add(baseJoints[5]);
            }
            return points;
        }

        private static string SerializePoints(List<double> points)
        {
            var text = new StringBuilder();
            foreach (double value in points)
            {
                if (text.Length != 0)
                {
                    text.Append(',');
                }
                text.Append(value.ToString(CultureInfo.InvariantCulture));
            }
            return text.ToString();
        }

        private static int Main()
        {
            Console.WriteLine("=== PathJ Relative Curve Example ===");

            // ----------------------------------------------------------
            // 1. 连接机器人并确认处于 Standby(33)
            // ----------------------------------------------------------
            // 需根据机器人的实际 IP 进行修改
            string host = "192.168.0.10";
            var robot = new RobotAPI();
            int ret = robot.HRIF_Connect(BOX_ID, host, 10003);
            if (!PrintRet(robot, "HRIF_Connect", ret))
            {
                return 1;
            }

            bool ok = true;
            bool motionStarted = false;
            int fsm = 0;
            ret = robot.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, ref fsm);
            if (!PrintRet(robot, "HRIF_ReadCurFSMFromCPS", ret))
            {
                ok = false;
            }
            else if (fsm != 33)
            {
                Console.Error.WriteLine("[ERR] Robot must be in Standby(33), FSM={0}", fsm);
                ok = false;
            }

            // ----------------------------------------------------------
            // 2. 设置速度比为0.2，运动到安全基准位姿，并以实际到达位置作为相对点位基准
            // ----------------------------------------------------------
            ret = robot.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
            PrintRet(robot, "HRIF_SetOverride", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            double dJ1 = 0;  double dJ2 = -90;  double dJ3 = 90;
            double dJ4 = -90;  double dJ5 = -90; double dJ6 = 0;
            if (ok)
            {
                ret = robot.HRIF_MoveJ(
                    BOX_ID, RBT_ID,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
                    "TCP", "Base", 30.0, 60.0, 0.0, 1, 0, 0, 0, "path_j_home"
                );
                ok = PrintRet(robot, "HRIF_MoveJ (safe pose)", ret);
                motionStarted = ok;
            }
            if (ok)
            {
                ok = WaitMotion(robot, TimeSpan.FromSeconds(120));
            }

            double[] joints = new double[6];
            if (ok)
            {
                ret = robot.HRIF_ReadActJointPos(
                    BOX_ID, RBT_ID,
                    ref joints[0], ref joints[1], ref joints[2],
                    ref joints[3], ref joints[4], ref joints[5]
                );
                ok = PrintRet(robot, "HRIF_ReadActJointPos", ret);
            }

            // ----------------------------------------------------------
            // 3. 创建 PathJ，推送 ACS 原始点位并等待轨迹计算完成
            // ----------------------------------------------------------
            List<double> points = null;
            if (ok)
            {
                points = BuildRelativeCurve(joints);
                ret = robot.HRIF_InitPath(
                    BOX_ID, RBT_ID, 0, PATH_NAME, 0.15, 5.0,
                    30.0, 60.0, 500.0, "Base", "TCP"
                );
                ok = PrintRet(robot, "HRIF_InitPath", ret);
            }
            if (ok)
            {
                ret = robot.HRIF_PushPathPoints(
                    BOX_ID, RBT_ID, PATH_NAME, SerializePoints(points)
                );
                ok = PrintRet(robot, "HRIF_PushPathPoints", ret);
            }
            if (ok)
            {
                ret = robot.HRIF_EndPushPathPoints(BOX_ID, RBT_ID, PATH_NAME);
                ok = PrintRet(robot, "HRIF_EndPushPathPoints", ret);
            }
            if (ok)
            {
                ok = WaitPathComputed(robot);
            }

            // ----------------------------------------------------------
            // 4. 先运动到轨迹初始点 P0，再运行 PathJ
            // ----------------------------------------------------------
            if (ok)
            {
                ret = robot.HRIF_MoveJ(
                    BOX_ID, RBT_ID,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    points[0], points[1], points[2], points[3], points[4], points[5],
                    "TCP", "Base", 30.0, 60.0, 0.0, 1, 0, 0, 0, "path_j_start"
                );
                ok = PrintRet(robot, "HRIF_MoveJ (P0)", ret);
            }
            if (ok)
            {
                ok = WaitMotion(robot, TimeSpan.FromSeconds(120));
            }
            if (ok)
            {
                ret = robot.HRIF_MovePathJ(BOX_ID, RBT_ID, PATH_NAME);
                ok = PrintRet(robot, "HRIF_MovePathJ", ret);
            }
            if (ok)
            {
                ok = WaitMotion(robot, TimeSpan.FromSeconds(120));
            }

            if (motionStarted)
            {
                int stopRet = robot.HRIF_GrpStop(BOX_ID, RBT_ID);
                if (!PrintRet(robot, "HRIF_GrpStop", stopRet))
                {
                    ok = false;
                }
            }
            int disconnectRet = robot.HRIF_DisConnect(BOX_ID);
            if (!PrintRet(robot, "HRIF_DisConnect", disconnectRet))
            {
                ok = false;
            }
            return ok ? 0 : 1;
        }
    }
}
