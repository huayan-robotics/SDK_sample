using RobotLibrarys;
using System;
using System.Diagnostics;
using System.Threading;

namespace HuayanRobotExamples
{
    internal static class FreeDriveExample
    {
        // BoxID 和 RbtID 可以在此处修改。
        private const int BOX_ID = 0;
        private const int RBT_ID = 0;

        private static bool print_ret(RobotAPI robot, string cmd, int ret)
        {
            if (ret == 0)
            {
                Console.WriteLine("[OK]  " + cmd);
                return true;
            }
            string errMsg = "";
            int lookupRet = robot.HRIF_GetErrorCodeStr(BOX_ID, ret, ref errMsg);
            Console.Error.WriteLine("[ERR] " + cmd + " -> " + ret + (lookupRet == 0 ? ": " + errMsg : ""));
            return false;
        }

        private static int fail(RobotAPI robot)
        {
            int ret = robot.HRIF_DisConnect(BOX_ID);
            print_ret(robot, "HRIF_DisConnect", ret);
            return 1;
        }

        public static int Main(string[] args)
        {
            Console.WriteLine("======== Force Free Drive Example ========");
            RobotAPI robot = new RobotAPI();
            // ----------------------------------------------------------
            // 1. 连接机器人
            // ----------------------------------------------------------
            const string ip = "192.168.0.10";
            const int port = 10003;
            int ret = robot.HRIF_Connect(BOX_ID, ip, port);
            if (!print_ret(robot, "HRIF_Connect", ret))
            {
                return 1;
            }

            // ----------------------------------------------------------
            // 2. 确认机器人处于 Standby(33)
            // ----------------------------------------------------------
            int nFSM = -1;
            ret = robot.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, ref nFSM);
            if (!print_ret(robot, "HRIF_ReadCurFSMFromCPS", ret) || nFSM != 33)
            {
                Console.Error.WriteLine("[ERR] Robot must be in Standby(33), FSM=" + nFSM);
                return fail(robot);
            }

            // ----------------------------------------------------------
            // 3. 仅开启 Z 平移自由度
            // ----------------------------------------------------------
            ret = robot.HRIF_SetFreeDriveMotionFreedom(BOX_ID, RBT_ID, 0, 0, 1, 0, 0, 0);
            if (!print_ret(robot, "HRIF_SetFreeDriveMotionFreedom", ret))
            {
                return fail(robot);
            }

            // ----------------------------------------------------------
            // 4. 设置启动阈值、速度和柔顺度
            // ----------------------------------------------------------
            const double dForceThreshold = 5.0;
            const double dTorqueThreshold = 0.4;

            ret = robot.HRIF_SetFTWrenchThresholds(BOX_ID, RBT_ID, dForceThreshold, dTorqueThreshold);
            if (!print_ret(robot, "HRIF_SetFTWrenchThresholds", ret))
            {
                return fail(robot);
            }

            const int nSpeedMode = 1; 

            ret = robot.HRIF_SetFTFreeDriveSpeedMode(BOX_ID, RBT_ID, nSpeedMode);
            if (!print_ret(robot, "HRIF_SetFTFreeDriveSpeedMode", ret))
            {
                return fail(robot);
            }

            const double dMaxLinearVelocity = 10.0;
            const double dMaxAngularVelocity = 5.0;

            ret = robot.HRIF_SetMaxFreeDriveVel(BOX_ID, RBT_ID, dMaxLinearVelocity, dMaxAngularVelocity);
            if (!print_ret(robot, "HRIF_SetMaxFreeDriveVel", ret))
            {
                return fail(robot);
            }

            const double dLinear = 50.0;
            const double dAngular = 50.0;

            ret = robot.HRIF_SetFTFreeFactor(BOX_ID, RBT_ID, dLinear, dAngular);
            if (!print_ret(robot, "HRIF_SetFTFreeFactor", ret))
            {
                return fail(robot);
            }

            // ----------------------------------------------------------
            // 5. 开启自由驱动
            // ----------------------------------------------------------
            ret = robot.HRIF_SetForceFreeDriveMode(BOX_ID, RBT_ID, 1);
            if (!print_ret(robot, "HRIF_SetForceFreeDriveMode(open)", ret))
            {
                return fail(robot);
            }

            Thread.Sleep(1000);

            int nForceState = -1;
            ret = robot.HRIF_ReadForceControlState(BOX_ID, RBT_ID, ref nForceState);
            if (!print_ret(robot, "HRIF_ReadForceControlState", ret) || nForceState != 3)
            {
                Console.Error.WriteLine("[ERR] Expected force state 3, actual=" + nForceState);
                return fail(robot);
            }

            // ----------------------------------------------------------
            // 6. 保持自由驱动 5 秒
            // ----------------------------------------------------------
            const int nDurationSeconds = 5;
            Console.WriteLine("[INFO] Free drive enabled for " + nDurationSeconds + " seconds.");
            Thread.Sleep(TimeSpan.FromSeconds(nDurationSeconds));

            // ----------------------------------------------------------
            // 7. 关闭自由驱动并断开连接
            // ----------------------------------------------------------
            ret = robot.HRIF_SetForceFreeDriveMode(BOX_ID, RBT_ID, 0);
            if (!print_ret(robot, "HRIF_SetForceFreeDriveMode(close)", ret))
            {
                return fail(robot);
            }

            ret = robot.HRIF_DisConnect(BOX_ID);
            print_ret(robot, "HRIF_DisConnect", ret);
            return ret == 0 ? 0 : 1;
        }
    }
}
