using RobotLibrarys;
using System;

namespace TrajectoryPathManageExample
{
    internal static class Program
    {
        // BoxID 和 RbtID 可以在此处进行修改
        private const int BOX_ID = 0;
        private const int RBT_ID = 0;
        private const string ORIGINAL_NAME = "sdk_manage_path_temp";
        private const string RENAMED_NAME = "sdk_manage_path_renamed";

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

        private static int Main()
        {
            Console.WriteLine("=== Trajectory Path Management Example ===");

            // ----------------------------------------------------------
            // 1. 连接机器人并确认处于 Standby(33)
            // ----------------------------------------------------------
            // 需根据机器人的实际 IP 进行修改
            const string host = "192.168.0.10";
            var robot = new RobotAPI();
            int ret = robot.HRIF_Connect(BOX_ID, host, 10003);
            if (!PrintRet(robot, "HRIF_Connect", ret))
            {
                return 1;
            }

            bool ok = true;
            string activeName = "";
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
            // 2. 确认临时名称未被占用，再创建专用于本案例的轨迹对象
            // ----------------------------------------------------------
            if (ok)
            {
                string initialPaths = "";
                ret = robot.HRIF_ReadPathList(BOX_ID, RBT_ID, ref initialPaths);
                ok = PrintRet(robot, "HRIF_ReadPathList (before create)", ret);
                if (ok && (initialPaths.Contains(ORIGINAL_NAME) || initialPaths.Contains(RENAMED_NAME)))
                {
                    Console.Error.WriteLine("[ERR] Temporary trajectory name is already in use");
                    ok = false;
                }
            }
            if (ok)
            {
                ret = robot.HRIF_InitPath(
                    BOX_ID, RBT_ID, 0, ORIGINAL_NAME, 0.1, 5.0,
                    30.0, 60.0, 500.0, "Base", "TCP"
                );
                ok = PrintRet(robot, "HRIF_InitPath", ret);
                if (ok)
                {
                    activeName = ORIGINAL_NAME;
                }
            }

            // ----------------------------------------------------------
            // 3. 读取创建后的轨迹列表和临时轨迹详细信息
            // ----------------------------------------------------------
            if (ok)
            {
                string pathList = "";
                ret = robot.HRIF_ReadPathList(BOX_ID, RBT_ID, ref pathList);
                ok = PrintRet(robot, "HRIF_ReadPathList", ret);
                if (ok)
                {
                    Console.WriteLine("[INFO] Path list: " + pathList);
                }
            }
            if (ok)
            {
                string pathInfo = "";
                ret = robot.HRIF_ReadPathInfo(BOX_ID, RBT_ID, ORIGINAL_NAME, ref pathInfo);
                ok = PrintRet(robot, "HRIF_ReadPathInfo", ret);
                if (ok)
                {
                    Console.WriteLine("[INFO] {0} info: {1}", ORIGINAL_NAME, pathInfo);
                }
            }

            // ----------------------------------------------------------
            // 4. 重命名轨迹并读取新名称下的信息
            // ----------------------------------------------------------
            if (ok)
            {
                ret = robot.HRIF_UpdatePathName(BOX_ID, RBT_ID, ORIGINAL_NAME, RENAMED_NAME);
                ok = PrintRet(robot, "HRIF_UpdatePathName", ret);
                if (ok)
                {
                    activeName = RENAMED_NAME;
                }
            }
            if (ok)
            {
                string pathInfo = "";
                ret = robot.HRIF_ReadPathInfo(BOX_ID, RBT_ID, RENAMED_NAME, ref pathInfo);
                ok = PrintRet(robot, "HRIF_ReadPathInfo (renamed)", ret);
                if (ok)
                {
                    Console.WriteLine("[INFO] {0} info: {1}", RENAMED_NAME, pathInfo);
                }
            }

            // ----------------------------------------------------------
            // 5. 删除临时轨迹并断开连接
            // ----------------------------------------------------------
            if (ok)
            {
                ret = robot.HRIF_DelPath(BOX_ID, RBT_ID, RENAMED_NAME);
                ok = PrintRet(robot, "HRIF_DelPath", ret);
                if (ok)
                {
                    activeName = "";
                }
            }
            if (activeName.Length != 0)
            {
                int deleteRet = robot.HRIF_DelPath(BOX_ID, RBT_ID, activeName);
                if (!PrintRet(robot, "HRIF_DelPath (cleanup)", deleteRet))
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
