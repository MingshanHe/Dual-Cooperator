/*
 * metaType.h
 *
 *  Created on: 2020-9-17
 *      Author: hanbing
 */

#ifndef METATYPE_H_
#define METATYPE_H_

#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/*-------------------------------------------------------------------------*/
/**
 * @brief 机器人运动状态
 */
/*-------------------------------------------------------------------------*/

typedef enum MoveState{
	_move_error=-1,///>错误状态
	_move_finish=0,///>允许开始新的运动
	_move_run_joint,///>正在进行关节空间运动
	_move_run_line,///>正在进行直线运动
	_move_run_circle,///>正在进行圆弧运动
	_move_run_helical,///>正在进行螺旋线运动
	_move_run_bspline,///>正在进行样条曲线运动
	_move_run_zone,///>正在进行转弯区运动
	_move_stop,///>驱动被强制停止，需要恢复到_move_finish才可重新运动
	_move_run_zone_finish,///>转弯区完成状态
	_move_egm,///>正在进行外部引导运动
	_move_direct_teach///>正在进行拖动运动
}MoveState;

/*-------------------------------------------------------------------------*/
/**
 * @brief 返回状态
 */
/*-------------------------------------------------------------------------*/
#define SUCCESS 0//返回成功
#define ERR_ROBOTMODE -101 //机器人模式错误
#define ERR_FUNNCTIONBUSY -102 //系统忙，正在运行功能模块
#define ERR_UNINITIALIZEDDATA -103//使用的数据异常
#define ERR_THREADCREATEFAILURE -104//线程创建失败
#define ERR_TARGETDATAHELD -105//目标数据区被占用
#define ERR_FORCE_SENSOR -106 //力矩传感器数据获取或转换失败
#define ERR_INVERSEKINEMATICS -107 //逆运动学求解错误
#define ERR_TARGETJOINTJUMP -108 //目标关节位置跳跃
#define ERR_MOVESTATE -109 //运动状态错误
#define ERR_NUPOWER -110 //未使能
#define ERR_MOVING -111 //机器人正在移动
#define ERR_ROBOTINDEX -112//机器人索引错误
#define ERR_ROBOTJOINTLIMIT -113//关节限位
#define ERR_ROBOTJOINTVELOCITYLIMIT -114//关节速度限制
#define ERR_ROBOTSINGULAR -115//机器人奇异
#define ERR_KINEMATICS -116 //逆运动学求解错误
#define ERR_FRAMETRANSFORM -117 //坐标变换错误
#define ERR_POWERON -118 //使能失败
#define ERR_POWEROFF -119 //下使能失败
#define ERR_ROBOTPROJECT -120 //机器人项目失败
#define ERR_TEACHMOVE -121 //示教运动失败
#define ERR_GRIPCONTROL -122 //示教运动失败
#define ERR_MOVESPEEDLIMIT -123 //笛卡尔移动速度超限
#define ERR_FORCESENSORLIMIT -124//六维力传感器超限
#define ERR_UNDEFINEDFUNCTION -125//未定义功能实现
#define ERR_DEVICEINEXISTENCE -126//操作设备不存在
#define ERR_SAVEDATA -127//保存数据失败
#define ERR_INDEXLIMIT -128 //索引超出限制
#define ERR_TOOLWOBJ -129 //工具或工件使用错误
#define ERR_SPATIALCONSTRAINT  -130 //笛卡尔空间限制


/*-------------------------------------------------------------------------*/
/**
 * @brief 约束属性
 */
/*-------------------------------------------------------------------------*/
typedef enum{
	_constraint_noallow=0,///>完全不允许区域
	_constraint_slowallow,///>不允许区域，但允许缓慢移动
	_constraint_allow,///>完全允许区域，其他区域完全不允许
	_constraint_allowslow///>完全允许区域，其他区域允许缓慢移动
}constraint_property;

/*-------------------------------------------------------------------------*/
/**
 * @brief 约束状态
 */
/*-------------------------------------------------------------------------*/
typedef enum{
	_constraint_allowstate=0,///>处于允许区域
	_constraint_slowstate,///>处于不允许区域，但允许缓慢移动
	_constraint_noallowstate,///>处于完全不允许区域
}constraint_state;

#ifdef __cplusplus
}
}
#endif

#endif /* METATYPE_H_ */
