using RobotLibrarys;
using System;
using System.Diagnostics;
using System.Threading;

namespace HuayanRobotExamples
{
    internal static class ConstantForceExample
    {
        // BoxID 和 RbtID 可以在此处修改。
        private const int BOX_ID = 0;
        private const int RBT_ID = 0;
        private const int FORCE_CONTROL_STATE_CLOSED = 0;
        private const int FORCE_CONTROL_STATE_SEARCH = 1;
        private const int FORCE_CONTROL_STATE_FOLLOW = 2;
        private const int FORCE_CONTROL_STATE_FREEDRIVE = 3;

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

        private static bool wait_blending_done(RobotAPI robot)
        {
            Stopwatch deadline = Stopwatch.StartNew();
            while (deadline.Elapsed < TimeSpan.FromSeconds(60))
            {
                Thread.Sleep(500);
                bool done = false;
                int ret = robot.HRIF_IsBlendingDone(BOX_ID, RBT_ID, ref done);
                if (ret != 0)
                {
                    return false;
                }
                if (done)
                {
                    Console.WriteLine("[ok] Motion Done ");
                    return true;
                }
            }
            Console.Error.WriteLine("[ERR] Motion timeout (60s).");
            return false;
        }

        public static int Main(string[] args)
        {
            Console.WriteLine("======== Constant Force Control Example ========");
            RobotAPI robot = new RobotAPI();
            // ----------------------------------------------------------
            // 1. 连接机器人
            // ----------------------------------------------------------
            const string host = "192.168.0.10";
            const int port = 10003;
            int ret = robot.HRIF_Connect(BOX_ID, host, port);
            if (!print_ret(robot, "HRIF_Connect", ret))
            {
                return 1;
            }

            // ----------------------------------------------------------
            // 2. 确认机器人处于 Standby(33)，并确认力控关闭
            // ----------------------------------------------------------
            int nFSM = -1;
            ret = robot.HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, ref nFSM);
            if (!print_ret(robot, "HRIF_ReadCurFSMFromCPS", ret) || nFSM != 33)
            {
                Console.Error.WriteLine("[ERR] Robot must be in Standby(33), FSM=" + nFSM);
                return fail(robot);
            }

            int nForceState = -1;
            ret = robot.HRIF_ReadForceControlState(BOX_ID, RBT_ID, ref nForceState);
            if (!print_ret(robot, "HRIF_ReadForceControlState", ret) || nForceState != 0)
            {
                Console.Error.WriteLine("[ERR] Expected force state 0, actual=" + nForceState);
                return fail(robot);
            }

            // ----------------------------------------------------------
            // 3. 设置低速倍率，运动到现场确认的安全探寻起点
            // ----------------------------------------------------------
            ret = robot.HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
            if (!print_ret(robot, "HRIF_SetOverride", ret))
            {
                return fail(robot);
            }

            // 请替换为初始探寻关节角度。
            double dJ1 = 0; double dJ2 = -90; double dJ3 = 90;
            double dJ4 = -90; double dJ5 = -90; double dJ6 = 0;

            ret = robot.HRIF_MoveJ(BOX_ID, RBT_ID, 0, 0, 0, 0, 0, 0, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
                                    "TCP", "Base", 10.0, 20.0, 0.0, 1, 0, 0, 0, "force_search_start");
            if (!print_ret(robot, "HRIF_MoveJ(search start)", ret) || !wait_blending_done(robot))
            {
                return fail(robot);
            }

            // ----------------------------------------------------------
            // 4. 选择恒力模式
            // ----------------------------------------------------------
            const int nStrategy = 0;

            ret = robot.HRIF_SetForceControlStrategy(BOX_ID, RBT_ID, nStrategy);
            if (!print_ret(robot, "HRIF_SetForceControlStrategy", ret))
            {
                return fail(robot);
            }

            // ----------------------------------------------------------
            // 5. 仅开启 Z 平移自由度，设置目标力与异常力保护
            // ----------------------------------------------------------
            ret = robot.HRIF_SetControlFreedom(BOX_ID, RBT_ID, 0, 0, 1, 0, 0, 0);
            if (!print_ret(robot, "HRIF_SetControlFreedom", ret))
            {
                return fail(robot);
            }

            const double dTargetFz = -10.0;

            ret = robot.HRIF_SetForceControlGoal(BOX_ID, RBT_ID, 0, 0, dTargetFz, 0, 0, 0);
            if (!print_ret(robot, "HRIF_SetForceControlGoal", ret))
            {
                return fail(robot);
            }

            ret = robot.HRIF_SetForceDataLimit(BOX_ID, RBT_ID, 500, 500, 500, 50, 50, 50, 300, 300, 300, 30, 30, 30);
            if (!print_ret(robot, "HRIF_SetForceDataLimit", ret))
            {
                return fail(robot);
            }

            // ----------------------------------------------------------
            // 6. 设置探寻速度、探寻边界和跟随边界
            // ----------------------------------------------------------
            ret = robot.HRIF_SetMaxSearchVelocities(BOX_ID, RBT_ID, 5.0, 1.0);
            if (!print_ret(robot, "HRIF_SetMaxSearchVelocities", ret))
            {
                return fail(robot);
            }

            ret = robot.HRIF_SetMaxSearchDistance(BOX_ID, RBT_ID, 100, 100, 100, 20, 20, 20);
            if (!print_ret(robot, "HRIF_SetMaxSearchDistance", ret))
            {
                return fail(robot);
            }

            ret = robot.HRIF_SetSteadyContactDeviationRange(BOX_ID, RBT_ID, 300, 300, 300, 20, 20, 20,
                                                            -100, -100, -100, -20, -20, -20);
            if (!print_ret(robot, "HRIF_SetSteadyContactDeviationRange", ret))
            {
                return fail(robot);
            }

            // ----------------------------------------------------------
            // 7. 设置动态响应参数
            // ----------------------------------------------------------
            // 设置力和力矩控制的质量/惯量参数和阻尼参数（此处设置为推荐的默认值）。
            ret = robot.HRIF_SetMassParams(BOX_ID, RBT_ID, 40.0, 40.0, 40.0, 10.0, 10.0, 10.0);
            if (!print_ret(robot, "HRIF_SetMassParams", ret))
            {
                return fail(robot);
            }

            ret = robot.HRIF_SetDampParams(BOX_ID, RBT_ID, 800.0, 800.0, 800.0, 40.0, 40.0, 40.0);
            if (!print_ret(robot, "HRIF_SetDampParams", ret))
            {
                return fail(robot);
            }

            // ----------------------------------------------------------
            // 8. 开启力控，等待探寻完成
            // ----------------------------------------------------------
            ret = robot.HRIF_SetForceControlState(BOX_ID, RBT_ID, 1);
            if (!print_ret(robot, "HRIF_SetForceControlState(open)", ret))
            {
                return fail(robot);
            }

            Stopwatch deadline = Stopwatch.StartNew();
            int state = -1;
            bool timeout = true;
            while (deadline.Elapsed < TimeSpan.FromSeconds(60))
            {
                Thread.Sleep(500);
                ret = robot.HRIF_ReadForceControlState(BOX_ID, RBT_ID, ref state);
                if (ret != 0)
                {
                    break;
                }
                if (state == FORCE_CONTROL_STATE_FOLLOW) // 代表力控探寻完成状态。
                {
                    Console.WriteLine("[OK] Force state = " + state);
                    timeout = false;
                    break;
                }
            }

            if (ret != 0 || timeout)
            {
                Console.Error.WriteLine("[ERR] Expected state !=" + FORCE_CONTROL_STATE_FOLLOW + ", actual=" + state);
                ret = robot.HRIF_SetForceControlState(BOX_ID, RBT_ID, 0);
                print_ret(robot, "HRIF_SetForceControlState", ret);
                return fail(robot);
            }

            // ----------------------------------------------------------
            // 9. 在探寻完成状态下，沿 UCS X 正方向进行空间直线运动
            // ----------------------------------------------------------
            double x = 0.0, y = 0.0, z = 0.0;
            double Rx = 0.0, Ry = 0.0, Rz = 0.0;
            double J1 = 0.0, J2 = 0.0, J3 = 0.0;
            double J4 = 0.0, J5 = 0.0, J6 = 0.0;
            ret = robot.HRIF_ReadActCoord(BOX_ID, RBT_ID, "Base", "TCP", ref J1, ref J2, ref J3, ref J4, ref J5, ref J6,
                                    ref x, ref y, ref z, ref Rx, ref Ry, ref Rz);
            if (!print_ret(robot,"HRIF_ReadActCoord", ret))
            {
                robot.HRIF_SetForceControlState(BOX_ID, RBT_ID, 0);
                robot.HRIF_DisConnect(BOX_ID);
                return fail(robot);
            }

            ret = robot.HRIF_MoveL(BOX_ID, RBT_ID, x, y + 50.0 , z, Rx, Ry, Rz, J1, J2, J3, J4, J5, J6, 
                                    "TCP", "Base", 30.0, 60.0, 0.0, 0, 0, 0, "force_follow_x");
            if (!print_ret(robot,"HRIF_MoveL(Y +50 mm)", ret) || !wait_blending_done(robot))
            {
                robot.HRIF_SetForceControlState(BOX_ID, RBT_ID, 0);
                robot.HRIF_DisConnect(BOX_ID);
                return fail(robot);
            }
            // ----------------------------------------------------------
            // 10. 关闭力控，断开连接
            // ----------------------------------------------------------
            ret = robot.HRIF_SetForceControlState(BOX_ID, RBT_ID, 0);
            if (!print_ret(robot, "HRIF_SetForceControlState(close)", ret))
            {
                return fail(robot);
            }

            Console.WriteLine("[OK] Constant force control completed.");

            ret = robot.HRIF_DisConnect(BOX_ID);
            print_ret(robot, "HRIF_DisConnect", ret);
            return ret == 0 ? 0 : 1;
        }
    }
}
