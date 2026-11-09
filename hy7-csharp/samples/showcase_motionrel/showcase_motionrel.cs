using RobotLibrarys;
using System;
using System.Diagnostics;
using System.Threading;

namespace HuayanRobotExamples
{
    internal static class MotionRelExample
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
            Console.WriteLine("======== Relative motion example started. ========");
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
            safeJoints.joint[0] = -90; safeJoints.joint[1] = -42.5;
            safeJoints.joint[2] = 0; safeJoints.joint[3] = 72.5;
            safeJoints.joint[4] = 0; safeJoints.joint[5] = -65;
            safeJoints.joint[6] = 0;
            ret = robot.HRIF_MoveJ_nJ(BOX_ID, RBT_ID, zeroPose, safeJoints, "TCP", "Base", 
                                      30.0, 60.0, 0.0, 1, 0, 0, 0, "Jog_start");
            PrintRet(robot, "HRIF_MoveJ_nJ", ret);
            if (ret != 0 || !WaitMotionDone(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 5. 进行单轴关节相对运动，J2 轴正向旋转 10°
            // ----------------------------------------------------------
            int axisId = 1;
            int direction = 1;
            double distance = 10.0;
            ret = robot.HRIF_MoveRelJ(BOX_ID, RBT_ID, axisId, direction, distance);
            PrintRet(robot, "HRIF_MoveRelJ(J2 +10 deg)", ret);
            if (ret != 0 || !WaitMotionDone(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 6. MoveRelL：沿当前 UCS X 反方向移动 30 mm
            // ----------------------------------------------------------
            int toolMotion = 0;
            axisId = 0;
            direction = 0;
            distance = 30.0;
            ret = robot.HRIF_MoveRelL(BOX_ID, RBT_ID, axisId, direction, distance, toolMotion);
            PrintRet(robot, "HRIF_MoveRelL(X -30 mm)", ret);
            if (ret != 0 || !WaitMotionDone(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 7. WayPointRel：不使用点位列表，叠加值模式下，进行空间相对运动，X正向移动 30 mm，Y反向移动 30 mm
            // ----------------------------------------------------------
            int moveType = 1;
            const int pointList = 0;
            const int relativeType = 1;
            JointsData pointJoints = Joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            JointsData axisMask = Joints(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            JointsData target = Joints(30.0, -30.0, 0.0, 0.0, 0.0, 0.0);
            ret = robot.HRIF_WayPointRel_nJ(BOX_ID, RBT_ID, moveType, pointList, zeroPose, pointJoints, relativeType,
                                            axisMask, target, "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPointRel_XY1");
            PrintRet(robot, "HRIF_WayPointRel_nJ(X +30 mm, Y -30 mm)", ret);
            if (ret != 0 || !WaitMotionDone(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 8. WayPointRel：不使用点位列表，叠加值模式下，X反向移动 30 mm，Y 正向移动 30 mm
            // ----------------------------------------------------------
            target = Joints(-30.0, 30.0, 0.0, 0.0, 0.0, 0.0);
            ret = robot.HRIF_WayPointRel_nJ(BOX_ID, RBT_ID, moveType, pointList, zeroPose, pointJoints, relativeType,
                                            axisMask, target, "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPointRel_XY2");
            PrintRet(robot, "HRIF_WayPointRel_nJ(X -30 mm, Y +30 mm)", ret);
            if (ret != 0 || !WaitMotionDone(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            // ----------------------------------------------------------
            // 9. WayPointRel：不使用点位列表，叠加值模式下，进行关节相对运动，J1 和 J2 同时旋转20°
            // ----------------------------------------------------------
            moveType = 0;
            target = Joints(10.0, 10.0, 0.0, 0.0, 0.0, 0.0);
            ret = robot.HRIF_WayPointRel_nJ(BOX_ID, RBT_ID, moveType, pointList, zeroPose, pointJoints, relativeType,
                                            axisMask, target, "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, 0, "WayPointRel_J1_J2");
            PrintRet(robot, "HRIF_WayPointRel_nJ(J1 +10 deg, J2 +10 deg)", ret);
            if (ret != 0 || !WaitMotionDone(robot))
            {
                robot.HRIF_DisConnect(BOX_ID);
                return 1;
            }

            Console.WriteLine("[OK] Relative motion completed.");
            
            // ----------------------------------------------------------
            // 10. 断开连接
            // ----------------------------------------------------------
            ret = robot.HRIF_DisConnect(BOX_ID);
            PrintRet(robot, "HRIF_DisConnect", ret);
            return ret == 0 ? 0 : 1;
        }
    }
}
