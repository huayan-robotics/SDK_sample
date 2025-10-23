#include "HR_Pro.h"
#include "funtion.h"
#include "CommonForInterface.h"

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <string.h>
#include <thread>
#include <limits>

// string IP = "192.168.0.10";
string IP = "10.20.60.102";
string IP2 = "192.168.56.13";

void Show();
void InitSetOverride();

int main(int argc, char **argv)
{
    int nRet = HRIF_Connect(0, IP.c_str(), 10003);
    if (nRet)
    {
        cout << "连接机器人失败/Robot Connect failure" << endl;
    }
    else
    {
        InitSetOverride();
        int Input = 0; // 输入对应的示例
        char ch = ' '; // 输入对应的选择
        while (true)
        {
            switch (Input)
            {
            case 0:
                Show();
                break;
            case 1:
                UserCase_XToStandby(); // 启动任务，将机械臂从其它状态返回到Standby状态
                break;
            case 2:
                UserCase_SetSimRobot(); // 设置真实机器人为模拟机器人
                break;
            case 3:
                UserCase_BrakeControl(); // 抱闸和松闸操作
                break;
            case 4:
                UserCase_Grap(); // 轴组控制
                break;
            case 5:
                UserCase_Script(); // 脚本运行示例
                break;
            case 6:
                UserCase_ControlBoxIO(); // 电箱控制及监控示例
                break;
            case 7:
                UserCase_BoxIDSample(IP,IP2); // 不同IP使用不同的BoxID使用场景示例
                break;
            case 8:
                UserCase_SysStateShow(); // 机器人系统状态显示
                break;
            case 9:
                UserCase_PonitInfo(); // 点位信息获取
                break;
            case 10:
                UserCase_RbtStateShow(); // 机器人运行状态显示
                break;
            case 11:
                UserCase_CoordTran(); // 坐标变换工具使用示例
                break;
            case 12:
                UserCase_ConfigTcpUcs(); // 配置TCP和UCS示例
                break;
            case 13:
                UserCase_ForceControl(); // 恒力控+越障+MoveL运动示例
                break;
            case 14:
                UserCase_FreeDrive(); // 自由驱动示例
                break;
            case 15:
                UserCase_MoveCircle(); // 圆弧运动使用示例
                break;
            case 16:
                UserCase_MoveInStandby(); // 处于就绪状态才可下发成功的运动指令示例
                break;
            case 17:
                UserCase_WayPoint(); // 路点运动使用示例
                break;
            case 18:
                UserCase_MoveInStartPoint(); // 需要运动到初始点位才可运行指令示例
                break;
            case 19:
                UserCase_MovePath(); // 运行轨迹示例
                break;
            case 20:
                UserCase_MovepathJ(); // 关节轨迹运动示例
                break;
            case 21:
                UserCase_MovePathL(); // 空间轨迹运动示例
                break;
            case 22:
                UserCase_ServoJ(); // 使用ServoJ进行实时控制示例
                break;
            case 23:
                UserCase_ServoP(); // 使用ServoP进行实时控制示例
                break;
            case 24:
                UserCase_Trace(); // 传送带位置跟随示例
                break;
            case 25:
                UserCase_ConnectToModBus(); // 连接到机器人末端MoudleBus
                break;
            case 26:
                ParsePortJsondataDemo(IP); // 获取端口的JSON数据解析示例
                break;
            case 27:
                ParsePortJson(IP); //10006端口解析示例（面向对象封装）
                break;
            case 28:
                UserCase_CalibrationForce(); //10006端口解析示例（面向对象封装）
                break;
            default:
                break;
            }
            std::cout << "请输入你想要执行的操作:";
            if (!(std::cin >> Input)) // 如果输入失败，可能是非整数输入
            {
                std::cin.clear();                                                   // 清除错误标志
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // 忽略直到下一个换行符
                std::cout << "错误：这不是一个整数。\n";
                continue;
            }
            if (Input == -1)
            {
                nRet = HRIF_DisConnect(0); // 断开连接
                IsSuccess(nRet, "HRIF_DisConnect");
                break;
            }
            else if (Input == -100)
            {
                std::cout << "机器人即将进行断电,系统将会关闭,如再次确认后输入y执行" << endl;
                cout << "请输入y/n:";
                std::cin >> ch;
                if (ch == 'y')
                {
                    nRet = HRIF_ShutdownRobot(0); // 断开连接
                    break;
                }
            }
            InitSetOverride();
            std::cin.clear();
        }
    }
    return 0;
}

/**
 *	@brief: 将所有接口的功能简要打印出来，每个示例独立运行，互不耦合
 *	@param : 无
 *	@return : 无
 */
void Show()
{
    cout << "-100.关闭机器人" << endl
         << "-1.断开连接" << endl
         << "0.重新显示" << endl
         << "1.启动任务,将机械臂从其它状态返回到Standby状态" << endl
         << "2.设置真实机器人为模拟机器人" << endl
         << "3.抱闸和松闸操作" << endl
         << "4.轴组控制" << endl
         << "5.脚本运行示例" << endl
         << "6.电箱控制及监控示例" << endl
         << "7.不同IP使用不同的BoxID使用场景示例" << endl
         << "8.机器人系统状态显示" << endl
         << "9.点位信息显示" << endl
         << "10.机器人运动状态显示" << endl
         << "11.坐标变换工具使用示例" << endl
         << "12.配置TCP和UCS示例" << endl
         << "13.恒力控+越障+MoveL运动示例" << endl
         << "14.自由驱动示例" << endl
         << "15.圆弧运动使用示例" << endl
         << "16.处于就绪状态才可下发成功的运动指令示例" << endl
         << "17.路点运动使用示例" << endl
         << "18.需要运动到初始点位才可运行指令示例" << endl
         << "19.运行轨迹示例" << endl
         << "20.关节轨迹运动示例" << endl
         << "21.空间轨迹运动示例" << endl
         << "22.使用ServoJ进行实时控制示例" << endl
         << "23.使用ServoP进行实时控制示例" << endl
         << "24.传送带位置跟随示例" << endl
         << "25.连接机器人末端MoudleBus" << endl
         << "26.获取端口的JSON数据解析示例" << endl
         << "27.获取端口的JSON数据解析示例(面向对象封装)" << endl
         <<"28.标定力示例"<<endl;
}

/**
 *	@brief: 初始化设置速度比，真实机器人将速度设置为20%，模拟机器人将速度设置为100%
 *	@param : 无
 *	@return : 无
 */
void InitSetOverride()
{
    int nSimulateRobot = -1;

    HRIF_IsSimulateRobot(0, nSimulateRobot); // 判断是否为模拟机器人
    switch (nSimulateRobot)                  // 判断机器人的状态并
    {
    case 0:
        HRIF_SetOverride(0, 0, 0.15);
        break;
    case 1:
        HRIF_SetOverride(0, 0, 1);
        break;
    default:
        break;
    }
}