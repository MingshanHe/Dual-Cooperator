/**
 * @file MovePlan.h
 * 
 * @brief  机器人运动控制接口函数
 * @author hanbing
 * @version 1.0
 * @date 2020-03-31
 * 
 */

#ifndef MOVEPLAN_H_
#define MOVEPLAN_H_

/*---------------------------- Includes ------------------------------------*/
#include "device_interface/Base/robotStruct.h"



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/**
 * @brief 设置指令线程运行方式，设置后对所有指令起作用
 *
 * @param isthread 1:线程运行，0:阻塞运行
 */
void setMoveThread(int isthread);

/**
 * @brief 等待运动结束（开线程运行时有效）
 *
 * @param robot_index 机器人索引号
 * @return int 运动完成：_move_finish(0),_move_stop(7),  机器人正在运行：_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动错误状态：_move_error(-1)
 */
int wait_move_finish(int robot_index);

/**
 * @brief 获取机器人运动状态
 *
 * @param robot_index 机器人索引号
 * @return int 运动完成：_move_finish(0),_move_stop(7),  机器人正在运行：_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动错误状态：_move_error(-1)
 */
int get_robot_move_state(int robot_index);

/**
 * @brief 休眠
 *
 * @param millisecond 休眠时间（毫秒）
 */
void Rsleep(int millisecond);

/**
 * @brief 设置机器人运动速度的百分比（加速度和加加速度）
 *
 * @param acc 加速度（0~1）
 * @param jerk 加加速度（0~1）
 * @param robot_index 机器人索引号
 */
extern void AccSet(double acc, double jerk, int robot_index);

/**
 * @brief 上电或开始运动
 *
 */
extern void move_start(void);

/**
 * @brief 停止运动或下电
 *
 */
extern void move_stop(void);

/**
 * @brief 绝对位置运动指令
 * 
 * @param rjoint 期望位置（关节） 单位：rad
 * @param rspeed 运动速度   单位：rad/s
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int moveA(robjoint *rjoint, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj);

/**
 * @brief 双臂绝对位置运动指令
 * 
 * @param rjoint1 机器人1的期望位置（关节）  单位：rad
 * @param rjoint2 机器人2的期望位置（关节）   单位：rad
 * @param rspeed1 机器人1的运动速度   单位：rad/s
 * @param rspeed2 机器人2的运动速度   单位：rad/s
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int dual_moveA(robjoint *rjoint1, robjoint *rjoint2, speed *rspeed1, speed *rspeed2, zone *rzone1, zone *rzone2, tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);

/**
 * @brief 选取多机械臂中的一个做绝对位置运动指令
 * 
 * @param rjoint 期望位置（关节） 单位：rad
 * @param rspeed 运动速度   单位：rad/s
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param _index 机器人索引号
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int multi_moveA(robjoint *rjoint, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj, int _index);

/**
 * @brief 关节运动指令
 * 
 * @param rpose 期望位置（位姿） 
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int moveJ(robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj);

/**
 * @brief 双臂关节运动指令
 * 
 * @param rpose1 机器人1的期望位置（位姿）
 * @param rpose2 机器人2的期望位置（位姿）
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int dual_moveJ(robpose *rpose1, robpose *rpose2, speed *rspeed1, speed *rspeed2, zone *rzone1, zone *rzone2, tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);

/**
 * @brief 选取多机械臂中的一个做绝对位置运动指令
 * 
 * @param rpose 期望位置（位姿） 
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param _index 机器人索引号
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int multi_moveJ(robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj, int _index);

/**
 * @brief 线性运动指令
 * 
 * @param rpose 期望位置（位姿）
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int moveL(robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj);

/**
 * @brief 双臂线性运动指令
 * 
 * @param rpose1 机器人1的期望位置（位姿）
 * @param rpose2 机器人2的期望位置（位姿）
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int dual_moveL(robpose *rpose1, robpose *rpose2, speed *rspeed1, speed *rspeed2, zone *rzone1, zone *rzone2, tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);

/**
 * @brief 选取多机械臂中的一个做线性运动指令
 * 
 * @param rpose 期望位置（位姿） 
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param _index 机器人索引号
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int multi_moveL(robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj, int _index);

/**
 * @brief 圆弧运动指令
 * 
 * @param rpose 期望位置（位姿）
 * @param rpose_mid 途经点期望位置（位姿）
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int moveC(robpose *rpose, robpose *rpose_mid, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj);

/**
 * @brief 双臂圆弧运动指令
 * 
 * @param rpose1 机器人1的期望位置（位姿）
 * @param rpose2 机器人2的期望位置（位姿）
 * @param rpose_mid1 机器人1的途经点期望位置（位姿）
 * @param rpose_mid2 机器人2的途经点期望位置（位姿）
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int dual_moveC(robpose *rpose1, robpose *rpose2, robpose *rpose_mid1, robpose *rpose_mid2, speed *rspeed1, speed *rspeed2, zone *rzone1, zone *rzone2, tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);

/**
 * @brief 选取多机械臂中的一个做圆弧运动指令
 * 
 * @param rpose 期望位置（位姿）
 * @param rpose_mid 途经点期望位置（位姿）
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param _index 机器人索引号
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int multi_moveC(robpose *rpose, robpose *rpose_mid, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj, int _index);

/**
 * @brief 螺旋线运动指令
 * 
 * @param rpose 旋转圆弧位置（位姿）（不影响姿态运动）
 * @param rpose_mid 旋转圆弧中间位置（位姿）（不影响姿态运动）
 * @param pose_line 方向位置（位姿）(决定最终运动姿态)
 * @param screw 螺距(mm)
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系 
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int moveH(robpose* rpose, robpose* rpose_mid, robpose* pose_line, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 平面螺旋线运动指令
 *
 * @param pose_line 方向位置（位姿）
 * @param radius 半径(mm)
 * @param screw 螺距(mm)
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int moveHP(robpose* pose_line, double radius, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 双臂螺旋线运动指令
 * 
 * @param rpose1 机器人1的圆弧位置（位姿）（不影响姿态运动）
 * @param rpose2 机器人2的圆弧位置（位姿）（不影响姿态运动）
 * @param rpose_mid1 机器人1的圆弧中间位置（位姿）（不影响姿态运动）
 * @param rpose_mid2 机器人2的圆弧中间位置（位姿）（不影响姿态运动）
 * @param pose_line1 机器人1的方向位置（位姿）(决定最终运动姿态)
 * @param pose_line2 机器人2的方向位置（位姿）(决定最终运动姿态)
 * @param screw1 机器人1螺距(mm)
 * @param screw2 机器人2螺距(mm)
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int dual_moveH(robpose* rpose1, robpose* rpose2, robpose* rpose_mid1, robpose* rpose_mid2, robpose* pose_line1,robpose* pose_line2, double screw1,double screw2,speed* rspeed1, speed* rspeed2, zone* rzone1, zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);

/**
 * @brief 双臂平面螺旋线运动指令
 *
 * @param pose_line1 机器人1的方向位置（位姿）
 * @param pose_line2 机器人2的方向位置（位姿）
 * @param radius1 机器人1半径（mm）
 * @param radius2 机器人2半径（mm）
 * @param screw1 机器人1螺距(mm)
 * @param screw2 机器人2螺距(mm)
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int dual_moveHP(robpose* pose_line1, robpose* pose_line2, double radius1, double radius2, double screw1, double screw2,speed* rspeed1, speed* rspeed2, zone* rzone1, zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);

/**
 * @brief 选取多机械臂中的一个做螺旋线运动指令
 * 
 * @param rpose 旋转圆弧位置（位姿）（不影响姿态运动）
 * @param rpose_mid 旋转圆弧中间位置（位姿）（不影响姿态运动）
 * @param pose_line 方向位置（位姿）(决定最终运动姿态)
 * @param screw 螺距(mm)
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系  
 * @param _index 机器人索引号
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int multi_moveH(robpose* rpose, robpose* rpose_mid, robpose* pose_line, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 选取多机械臂中的一个做平面螺旋线运动指令
 *
 * @param pose_line 方向位置（位姿）
 * @param radius 半径(mm)
 * @param screw 螺距(mm)
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param _index 机器人索引号
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int multi_moveHP(robpose* pose_line, double radius, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 关节空间的B样条运动
 * 
 * @param filename 轨迹点存放文件名
 * @param rspeed 运动速度
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int moveS(char *filename, speed *rspeed, tool *rtool, wobj *rwobj);

/**
 * @brief 双臂关节空间的B样条运动
 * 
 * @param filename1 机器人1的轨迹点存放文件名
 * @param filename2 机器人2的轨迹点存放文件名
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int dual_moveS(char *filename1, char *filename2, speed *rspeed1, speed *rspeed2, tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);

/**
 * @brief 选取多机械臂中的一个做关节空间的B样条运动
 * 
 * @param filename 轨迹点存放文件名
 * @param rspeed 运动速度
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param _index 机器人索引号
 * @return int 正确返回值:_move_finish(0),_move_stop(7),  异常返回值: 机器人正在运行_move_run_joint(1),_move_run_line(2),_move_run_cricle(3),_move_run_helical(4),_move_run_bspline(5),_move_run_zone(6),_move_run_zone_finish(8)；运动状态错误_move_error(-1), -4~-2：初始化数据错误
 */
extern int multi_moveS(char *filename, speed *rspeed, tool *rtool, wobj *rwobj, int _index);

/**
 * @brief 机器人位姿绝对工件偏移补偿
 *
 * @param rpose 机器人初始位姿
 * @param x x方向补偿值(mm)
 * @param y y方向补偿值(mm)
 * @param z z方向补偿值(mm)
 * @param k 姿态变量1补偿值(rad)
 * @param p 姿态变量2补偿值(rad)
 * @param s 姿态变量3补偿值(rad)
 * @return robpose 机器人补偿后位姿
 */
extern robpose Offs(const robpose* rpose, double x, double y, double z, double k, double p, double s);

/**
 * @brief 机器人位姿相对工具偏移补偿
 *
 * @param rpose 机器人初始位姿
 * @param x x方向补偿值(mm)
 * @param y y方向补偿值(mm)
 * @param z z方向补偿值(mm)
 * @param k 姿态变量1补偿值(rad)
 * @param p 姿态变量2补偿值(rad)
 * @param s 姿态变量3补偿值(rad)
 * @return robpose 机器人补偿后位姿
 */
extern robpose OffsRel(const robpose* rpose, double x, double y, double z, double k, double p, double s);



/**
 * @brief 获取机器人当前关节角
 * 
 * @param joint 当前关节角数据(rad/mm)
 * @param _index 机器人索引号
 */
extern void robot_getJoint(double* joint, int _index);

/**
 * @brief 获取机器人当前位姿
 * 
 * @param tool 工具
 * @param wobj 工件
 * @param pospose 当前位姿数据(mm,rad)
 * @param _index 机器人索引号
 * @return int 0:成功；其他失败
 */
extern int robot_getCartesian(tool* rtool,wobj* rwobj, double* pospose, int _index);

/**
 * @brief 设置数字量输出
 * 
 * @param id 数字量索引号
 * @param flag 0或1
 */
extern void SetDo(int id, int flag);

/**
 * @brief 获取数字量输入
 * 
 * @param id 数字量输出索引号
 * @return int 返回值0或1
 */
extern int GetDi(int id);

/**
 * @brief 等待数字量输入赋值
 * 
 * @param id 数字量输入索引号
 * @param value 触发信号0或1
 */
extern void WaitDi(int id, int value);

/**
 * @brief 设置模拟量输出
 * 
 * @param id 模拟量输出索引号
 * @param flag 模拟量
 */
extern void SetAo(int id, double flag);

/**
 * @brief 获取模拟量输入
 * 
 * @param id 模拟量输入索引号
 * @return double 模拟量
 */
extern double GetAi(int id);



/**
 * @brief 获取机器人自由度
 * 
 * @param _index 机器人索引号
 * @return int 返回机器人自由度
 */
extern int robot_getDOF(int _index);

/**
 * @brief 获取机器人附加轴数量
 * 
 * @param _index 机器人附加轴分组索引
 * @return int 返回附加轴数量 
 */
extern int additionaxis_getDOF(int _index);

/**
 * @brief 获取机器人数量
 * 
 * @return int 返回机器人数量
 */
extern int robot_getNUM();

/**
 * @brief 获取附加轴组数量
 *
 * @return int 返回附加轴组数量
 */
extern int additionaxis_getNUM();

//----------------------------------------------------------------------------------------communication interface--------------------------------------------------------------------------

/**
 * @brief 创建tcp server（阻塞）
 * 
 * @param ip ip地址(NULL为本机ip)
 * @param port 端口号
 * @param sName server名称
 * @return int 成功返回0，错误返回其他
 */
extern int SocketCreate(const char *ip, int port, const char *sName);

/**
 * @brief 创建tcp client（阻塞）
 * 
 * @param ip ip地址
 * @param port 端口号
 * @param sName client名称
 * @return int 成功返回0，错误返回其他
 */
extern int ClientCreate(const char *ip, int port, const char *sName);

/**
 * @brief 创建tcp server（非阻塞）
 *
 * @param ip ip地址(NULL为本机ip)
 * @param port 端口号
 * @param timeout 最大阻塞时间（us），负数为阻塞
 * @param sName server名称
 * @return int 成功返回0，错误返回其他
 */
extern int SocketCreate1(const char* ip, int port, int timeout, const char* sName);

/**
 * @brief 创建tcp client（非阻塞）
 *
 * @param ip ip地址
 * @param port 端口号
 * @param timeout 最大阻塞时间（us），负数为阻塞
 * @param sName client名称
 * @return int 成功返回0，错误返回其他
 */
extern int ClientCreate1(const char* ip, int port, int timeout, const char* sName);

/**
 * @brief 关闭socket（含server和client）
 * 
 * @param sName server名称或client名称
 * @return int 成功返回0 ，错误返回其他
 */
extern int SocketClose(const char *sName);

/**
 * @brief TCP发送Byte型数据
 * 
 * @param data Byte型数据段
 * @param len 数据段长度
 * @param sName socket名称
 * @return int 成功返回接收长度，错误返回值小于零
 */
extern int SocketSendByteI(byte *data, int len, const char *sName);

/**
 * @brief TCP接收Byte型数据
 * 
 * @param data Byte型数据段
 * @param len 数据段长度
 * @param sName socket名称
 * @return int 成功返回接收数据长度，错误返回值小于零
 */
extern int SocketRecvByteI(byte *data, int len, const char *sName);


/**
 * @brief TCP发送数据
 * 
 * @param header 协议头
 * @param header_format 协议头格式
 * @param hf_len 协议头长度
 * @param data_int int型数组
 * @param data_float float型数组
 * @param data_format 数据格式
 * @param df_len 数据段长度
 * @param sName socket名称
 * @return int 成功返回接收数据长度，错误返回值小于零
 */
extern int SocketSendByteII(int *header, int (*header_format)[2], int hf_len,
                            int *data_int, float *data_float, int (*data_format)[2], int df_len, const char *sName);

/**
 * @brief TCP接收数据
 * 
 * @param header 协议头
 * @param header_format 协议头格式
 * @param hf_len 协议头长度
 * @param data_int int型数组
 * @param data_float float型数组
 * @param data_format 数据格式
 * @param df_len 数据段长度
 * @param sName socket名称
 * @return int 成功返回接收数据长度，错误返回值小于零
 */
extern int SocketRecvByteII(int *header, int (*header_format)[2], int hf_len,
                            int *data_int, float *data_float, int (*data_format)[2], int df_len, const char *sName);


/**
 * @brief 通过TCP发送一个byte型数据
 * 
 * @param data byte型数据
 * @param sName socket名称
 * @return int 成功返回1，错误返回其他
 */
extern int SocketSendByte(byte data, const char *sName);

/**
 * @brief 通过TCPt接收一个byte型数据
 * 
 * @param data byte型数据
 * @param sName socket名称
 * @return int 成功返回1，错误返回其他
 */
extern int SocketRecvByte(byte *data, const char *sName);

/**
 * @brief 通过TCP发送一个String型数据
 * 
 * @param data String型数据
 * @param sName socket名称
 * @return int 成功返回1，错误返回其他
 */
extern int SocketSendString(char *data, const char *sName);

/**
 * @brief 通过TCP接收一个String型数据
 * 
 * @param data String型数据
 * @param sName socket名称
 * @return int 成功返回1，错误返回其他
 */
extern int SocketRecvString(char *data, const char *sName);

/**
 * @brief 通过TCP发送一个double型数据
 * 
 * @param data double型数据
 * @param sName socket名称 
 * @return int 成功返回1，错误返回其他
 */
extern int SocketSendDouble(double data, const char *sName);

/**
 * @brief 通过TCP接收一个double型数据
 * 
 * @param data double型数据
 * @param sName socket名称
 * @return int 成功返回1，错误返回其他
 */
extern int SocketRecvDouble(double *data, const char *sName);

/**
 * @brief 通过TCP发送一个int型数据
 * 
 * @param data int型数据
 * @param sName socket名称
 * @return int 成功返回1，错误返回其他
 */
extern int SocketSendInt(int data, const char *sName);

/**
 * @brief 通过TCP接收一个int型数据
 * 
 * @param data int型数据
 * @param sName socket名称
 * @return int 成功返回1，错误返回其他
 */
extern int SocketRecvInt(int *data, const char *sName);

/**
 * @brief 通过TCP发送一个byte型数组
 * 
 * @param data byte型数组
 * @param n 数组长度
 * @param sName socket名称
 * @return int 成功>0，错误返回其他
 */
extern int SocketSendByteArray(byte *data, int n, const char *sName);

/**
 * @brief 通过TCP接收一个byte型数组
 * 
 * @param data byte型数组
 * @param sName socket名称
 * @return int 成功>0，错误返回其他
 */
extern int SocketRecvByteArray(byte *data, const char *sName);

/**
 * @brief 通过TCP发送一个double型数组
 * 
 * @param data double型数组
 * @param n 数组长度
 * @param sName socket名称
 * @return int 成功>0，错误返回其他
 */
extern int SocketSendDoubleArray(double *data, int n, const char *sName);

/**
 * @brief 通过TCP接收一个double型数组
 * 
 * @param data double型数组
 * @param sName  socket名称
 * @return int 成功>0，错误返回其他
 */
extern int SocketRecvDoubleArray(double *data, const char *sName);

/**
 * @brief 通过TCP发送一个int型数组
 * 
 * @param data int型数组
 * @param n 数组长度
 * @param sName socket名称
 * @return int 成功>0，错误返回其他
 */
extern int SocketSendIntArray(int *data, int n, const char *sName);

/**
 * @brief 通过TCP接收一个int型数组
 * 
 * @param data  int型数组
 * @param sName socket名称
 * @return int 成功>0，错误返回其他
 */
extern int SocketRecvIntArray(int *data, const char *sName);

/**
 * @brief 创建udp server（阻塞）
 *
 * @param ip ip地址(NULL为本机ip)
 * @param port 端口号
 * @param sName server名称
 * @return int 成功返回0，错误返回其他
 */
extern int UDPServerCreate(const char* ip, int port, const char* sName);

/**
 * @brief 创建udp client（阻塞）
 *
 * @param ip ip地址
 * @param port 端口号
 * @param sName client名称
 * @return int 成功返回0，错误返回其他
 */
extern int UDPClientCreate(const char* ip, int port, const char* sName);

/**
 * @brief 创建udp server（非阻塞）
 *
 * @param ip ip地址(NULL为本机ip)
 * @param port 端口号
 * @param timeout 最大阻塞时间（us），负数为阻塞
 * @param sName server名称
 * @return int 成功返回0，错误返回其他
 */
extern int UDPServerCreate1(const char* ip, int port, int timeout, const char* sName);

/**
 * @brief 创建udp client（非阻塞）
 *
 * @param ip ip地址(NULL为本机ip)
 * @param port 端口号
 * @param timeout 最大阻塞时间（us），负数为阻塞
 * @param sName server名称
 * @return int 成功返回0，错误返回其他
 */
extern int UDPClientCreate1(const char* ip, int port, int timeout, const char* sName);

/**
 * @brief 通过UDP发送byte型数组
 * 
 * @param data byte型数组
 * @param len 数组长度
 * @param sName socket名称
 * @return int 成功返回接收长度，错误返回值小于零
 */
extern int UDPSendByteI(byte *data, int len, const char *sName);

/**
 * @brief 通过UDP接收byte型数组
 * 
 * @param data byte型数组
 * @param len 数组长度
 * @param sName socket名称
 * @return int 成功返回接收长度，错误返回值小于零
 */
extern int UDPRecvByteI(byte *data, int len, const char *sName);

/**
 * @brief UDP发送数据
 * 
 * @param header 协议头
 * @param header_format 协议头格式
 * @param hf_len 协议头长度
 * @param data_int int型数组
 * @param data_float float型数组
 * @param data_format 数据格式
 * @param df_len 数据段长度
 * @param sName socket名称
 * @return int 成功返回接收数据长度，错误返回值小于零
 */
extern int UDPSendByteII(int *header, int (*header_format)[2], int hf_len,
                         int *data_int, float *data_float, int (*data_format)[2], int df_len, const char *sName);

/**
 * @brief UDP接收数据
 * 
 * @param header 协议头
 * @param header_format 协议头格式
 * @param hf_len 协议头长度
 * @param data_int int型数组
 * @param data_float float型数组
 * @param data_format 数据格式
 * @param df_len 数据段长度
 * @param sName socket名称
 * @return int 成功返回接收数据长度，错误返回值小于零
 */
extern int UDPRecvByteII(int *header, int (*header_format)[2], int hf_len,
                         int *data_int, float *data_float, int (*data_format)[2], int df_len, const char *sName);

/**
 * @brief UDP发送byte型数据
 * 
 * @param data byte型数据
 * @param sName socket名称
 * @return int 成功返回1，错误返回其他
 */
extern int UDPSendByte(byte data, const char *sName);

/**
 * @brief UDP接收byte型数据
 * 
 * @param data byte型数据
 * @param sName socket名称
 * @return int  成功返回1，错误返回其他
 */
extern int UDPRecvByte(byte *data, const char *sName);

/**
 * @brief 通过UDP发送一个string型数据
 * 
 * @param data string型数据
 * @param sName socket名称
 * @return int 成功返回字符串长度，错误返回其他
 */
extern int UDPSendString(char *data, const char *sName);

/**
 * @brief 通过UDP接收一个string型数据
 * 
 * @param data string型数据
 * @param sName  socket名称
 * @return int 成功返回字符串长度，错误返回其他
 */
extern int UDPRecvString(char *data, const char *sName);

/**
 * @brief 通过UDP发送一个double型数据
 * 
 * @param data double型数据
 * @param sName socket名称
 * @return int 成功返回1，错误返回其他
 */
extern int UDPSendDouble(double data, const char *sName);

/**
 * @brief 通过UDP接收一个double型数据
 * 
 * @param data double型数据
 * @param sName socket名称
 * @return int 成功返回1，错误返回其他
 */
extern int UDPRecvDouble(double *data, const char *sName);

/**
 * @brief 通过UDP发送一个int型数据
 * 
 * @param data int型数据
 * @param sName socket名称
 * @return int 成功返回1，错误返回其他
 */
extern int UDPSendInt(int data, const char *sName);

/**
 * @brief 通过UDP接收一个int型数据
 * 
 * @param data int型数据
 * @param sName socket名称
 * @return int 成功返回1，错误返回其他
 */
extern int UDPRecvInt(int *data, const char *sName);

/**
 * @brief 通过UDP发送一个byte型数组
 * 
 * @param data byte型数组
 * @param n 数组长度
 * @param sName socket名称
 * @return int 成功返回数据，错误返回其他
 */
extern int UDPSendByteArray(int *data, int n, const char *sName);

/**
 * @brief 通过UDP接收一个byte型数组
 * 
 * @param data byte型数组
 * @param sName socket名称
 * @return int 成功返回数据长度，错误返回其他
 */
extern int UDPRecvByteArray(int *data, const char *sName);

/**
 * @brief 通过UDP发送一个double型数组
 * 
 * @param data double型数组
 * @param n 数组长度
 * @param sName socket名称
 * @return int 成功返回数据长度，错误返回其他
 */
extern int UDPSendDoubleArray(double *data, int n, const char *sName);

/**
 * @brief 通过UDP接收一个double型数组
 * 
 * @param data double型数组
 * @param sName socket名称
 * @return int 成功返回数据长度，错误返回其他
 */
extern int UDPRecvDoubleArray(double *data, const char *sName);

/**
 * @brief 通过UDP发送一个int型数组
 * 
 * @param data int型数组
 * @param n 数组长度
 * @param sName socket名称
 * @return int 成功返回数据长度，错误返回其他
 */
extern int UDPSendIntArray(int *data, int n, const char *sName);

/**
 * @brief 通过UDP接收一个int型数组
 * 
 * @param data int型数组
 * @param sName socket名称
 * @return int 成功返回数据长度，错误返回其他
 */
extern int UDPRecvIntArray(int *data, const char *sName);

//----------------------------------------------------------------------------------------thread interface--------------------------------------------------------------------------
/**
 * @brief 创建线程
 * 
 * @param fun 线程回调执行函数
 * @param arg 执行参数
 * @param name 线程名
 * @param detached_flag 线程属性标识thread attribute,0:PTHREAD_CREATE_JOINABLE  other:PTHREAD_CREATE_DETACHED
 * @return int 成功返回0，失败返回其他
 */
extern int ThreadCreat(void *(*fun)(void *), void *arg, const char *name, int detached_flag);

/**
 * @brief 线程数据释放
 * 
 * @param name 线程名
 * @return int 成功返回0，失败返回其他
 */
extern int ThreadDataFree(const char *name);


/**
 * @brief thread join (当线程属性为PTHREAD_CREATE_JOINABLE时有效)
 * 
 * @param name 线程名
 * @return int 成功返回0，失败返回其他
 */
extern int ThreadWait(const char *name);


#ifdef __cplusplus
}
}
#endif


#endif /* MOVEPLAN_H_ */
