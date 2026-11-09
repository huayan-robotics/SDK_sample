using RobotLibrarys;
using System;
using System.Diagnostics;
using System.Threading;

namespace HuayanRobotExamples
{
    internal static class MotionExample
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

        private static Pose Point(double x, double y, double z, double rx, double ry, double rz)
        {
            return new Pose { dX = x, dY = y, dZ = z, dRX = rx, dRY = ry, dRZ = rz };
        }

        private static JointsData Joints(params double[] values)
        {
            JointsData data = new JointsData();
            for (int i = 0; i < values.Length; ++i) data.joint[i] = values[i];
            return data;
        }

        public static int Main(string[] args)
        {
            Console.WriteLine("======== Waypoint Motion Example Start ========");
            RobotAPI robot = new RobotAPI();

            // ----------------------------------------------------------
            // 1. 连接机器人
            // ----------------------------------------------------------
            // 需根据机器人的实际 IP 进行修改
            const string host = "192.168.0.10";
            const int port = 10003;
            int ret = robot.HRIF_Connect(BOX_ID, host, port);
            PrintRet(robot, "HRIF_Connect", ret);
            if (ret != 0) return 1;

            // ----------------------------------------------------------
            // 2. 确认机器人处于 Standby(33)
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
            // 3. 设置全局速度比为 0.2
            // ----------------------------------------------------------
            ret = robot.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
            PrintRet(robot, "HRIF_SetOverride", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 4. MoveJ运动到安全关节位置, 无过渡，过渡半径为 0.0 mm；
            // 若设置了过渡半径，并且用HRIF_IsMotionDone命令判断此条路点运动是否完成，则过渡不生效
            // ----------------------------------------------------------
            Pose zeroPose = new Pose();
            JointsData safeJoints = new JointsData();
            safeJoints.joint[0] = 90; safeJoints.joint[1] = 45;
            safeJoints.joint[2] = 0; safeJoints.joint[3] = -80;
            safeJoints.joint[4] = 0; safeJoints.joint[5] = 35;
            safeJoints.joint[6] = 0;
            ret = robot.HRIF_MoveJ_nJ(BOX_ID, RBT_ID, zeroPose, safeJoints, "TCP", "Base", 30.0, 60.0,
                                      0.0, 1, 0, 0, 0, "absolute_MoveJ_home");
            PrintRet(robot, "HRIF_MoveJ_nJ(Point A)", ret);
            if (ret != 0 || !WaitMotionDone(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 5.读取TCP当前位置，用于后续路点运动
            // 从安全点 A 开始在 XZ 平面绘制云朵形状，A 到 B 沿 Z 负方向，云朵向 X 正方向展开，Y 保持不变
            //                         M
            //             Waypoint ╱     ╲ WaypointEx
            //                    C         D
            //                  ╭             ╮
            //            WayPoint2          WayPoint2
            //                  ╰             ╯
            //                    B ◄─MoveL── A
            // ----------------------------------------------------------
            const double straightLength = 30.0;
            const double arcRadius = 10.0;
            double x = 0.0, y = 0.0, z = 0.0, rx = 0.0, ry = 0.0, rz = 0.0;
            ret = robot.HRIF_ReadActTcpPos(BOX_ID, RBT_ID, ref x, ref y, ref z, ref rx, ref ry, ref rz);
            PrintRet(robot, "HRIF_ReadActPos", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 6. 从 A 点沿 Z 负方向 MoveL 直线运动到点 B，无过渡
            // ----------------------------------------------------------
            z -= straightLength;
            Pose pose = Point(x, y, z, rx, ry, rz);
            ret = robot.HRIF_MoveL_nJ(BOX_ID, RBT_ID, pose, safeJoints, "TCP", "Base", 30.0, 60.0,
                                      0.0, 0, 0, 0, "absolute_MoveL_approach");
            PrintRet(robot, "HRIF_MoveL_nJ(Point B)", ret);
            if (ret != 0 || !WaitMotionDone(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 7. WayPoint2：经过辅助点执行 B -> C 圆弧，无过渡
            // ----------------------------------------------------------
            Pose auxPose = Point(x + arcRadius, y, z - arcRadius, rx, ry, rz);
            Pose endPose = Point(x + 2.0 * arcRadius, y, z, rx, ry, rz);
            ret = robot.HRIF_WayPoint2_nJ(BOX_ID, RBT_ID, endPose, auxPose, safeJoints, "TCP", "Base", 50.0,
                                          50.0, 0.0, 2, 0, 0, 0, 0, "WayPoint2_B_C");
            PrintRet(robot, "HRIF_WayPoint2_nJ(B->C)", ret);
            if (ret != 0 || !WaitMotionDone(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 8. WayPoint：C -> M，有过渡，过渡半径为 20.0 mm
            // ----------------------------------------------------------
            int moveType = 1;
            z += straightLength / 4.0;
            x += 2.0 * arcRadius + 30.0;
            pose = Point(x, y, z, rx, ry, rz);
            ret = robot.HRIF_WayPoint_nJ(BOX_ID, RBT_ID, pose, safeJoints, "TCP", "Base", 30.0, 60.0,
                                         20.0, moveType, 0, 0, 0, 0, "WayPoint_C_M");
            PrintRet(robot, "HRIF_WayPoint_nJ(C->M)", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 9. WayPointEx：M -> D，，有过渡，过渡半径为 20.0 mm
            // ----------------------------------------------------------
            z += straightLength / 4.0;
            x -= 30.0;
            pose = Point(x, y, z, rx, ry, rz);
            ret = robot.HRIF_WayPointEx_nJ(BOX_ID, RBT_ID, pose, safeJoints, zeroPose, zeroPose, 30.0, 60.0,
                                           20.0, moveType, 0, 0, 0, 0, "WayPointEx_M_D");
            PrintRet(robot, "HRIF_WayPointEx_nJ(M->D)", ret);
            if (ret != 0)
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 10. WayPoint2：D -> A，绘制圆弧，无过渡
            // ----------------------------------------------------------
            moveType = 2;
            endPose = Point(x - 2.0 * arcRadius, y, z, rx, ry, rz);
            auxPose = Point(x - arcRadius / 2.0, y, z + arcRadius / 2.0, rx, ry, rz);
            ret = robot.HRIF_WayPoint2_nJ(BOX_ID, RBT_ID, endPose, auxPose, safeJoints, "TCP", "Base", 50.0,
                                          50.0, 0.0, moveType, 0, 0, 0, 0, "WayPoint2_D_A");
            PrintRet(robot, "HRIF_WayPoint2_nJ(D->A)", ret);
            if (ret != 0 || !WaitMotionDone(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            Console.WriteLine("[OK] Absolute motion completed.");

            // ----------------------------------------------------------
            // 11. 断开连接
            // ----------------------------------------------------------
            ret = robot.HRIF_DisConnect(BOX_ID);
            PrintRet(robot, "HRIF_DisConnect", ret);
            return ret == 0 ? 0 : 1;
        }
    }
}
