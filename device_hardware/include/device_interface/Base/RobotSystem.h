/**
 * @file RobotSystem.h
 *
 * @brief  机器人控制系统初始化必要接口
 * @author hanbing
 * @version 1.0
 * @date 2020-04-08
 *
 */

#ifndef ROBOTSYSTEM_H_
#define ROBOTSYSTEM_H_
/*---------------------------- Includes ------------------------------------*/
#include "device_interface/Base/robotStruct.h"


#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif


/**
 * @brief 程序优先级设置
 * @param priority  优先级
 *
 * @return int 0:正确，错误返回其他
 */
extern int initPriority(int priority);

/**
 * @brief 线程优先级设置
 * @param priority  优先级
 *
 * @return int 0:正确，错误返回其他
 */
int initCurrentThreadPriority(int priority);

/**
 * @brief 线程名称设置
 * @param name  线程名
 *
 * @return int 0:正确，错误返回其他
 */
int setCurrentThreadName(char* name);

/**
 * @brief 解析程序命令行
 *
 * @param argc  用于存放命令行参数的个数
 * @param argv  是个字符指针的数组，每个元素都是一个字符指针，指向一个字符串，即命令行中的每一个参数。
 * @param arg  返回解析结果
 *
 * @return int 0:正确，错误返回其他
 */
extern int commandLineParser(int argc, char *argv[],command_arg* arg);

/**
 * @brief 命令行参数输入
 *
 * @param carg  命令行参数，如：”--path /hanbing --port 6665 --iscopy true“。（NULL为默认）
 * @param arg  返回解析结果
 *
 * @return int 0:正确，错误返回其他
 */
extern int commandLineParser1(const char* carg, command_arg* arg);

/**
 * @brief 运行信息写入日志文件
 *
 * @param __format  格式化输入
 * @return int 0:正确，错误返回其他
 */
extern int Rdebug(const char *__restrict __format, ...);

/**
 * @brief 系统初始化
 *
 * @param arg 系统启动参数(NULL为默认)
 * @return int 0:成功; 1:失败
 */
extern int system_initialize(command_arg* arg);

/**
 * @brief 系统状态是否正常
 *
 * @return int 1:状态正常; 0:状态错误或强制退出
 */
extern int robot_ok();

/**
 * @brief 运动状态是否正常
 *
 * @return int 1:可以运动; 0:状态错误或有机器人正在运动
 */
extern int robot_move_ok();

/**
 * @brief 机器人是否正在运动
 *
 * @return int 1:正在运动; 0:未运动或出现错误
 */
int robot_runing();

#ifdef __cplusplus
}
}
#endif

#endif /* ROBOTSYSTEM_H_ */
