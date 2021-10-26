/**
 * @file grip_interface.h
 * @brief  夹爪创建、释放、操作
 * 
 *
 * @date Created on: 2020-8-6
 * @author hanbing
 * 
 */
#ifndef GRIP_INTERFACE_H_
#define GRIP_INTERFACE_H_


#ifdef __cplusplus

namespace HYYRobotBase
{
extern "C" {
#endif

/**
 * @brief 创建夹爪（系统内默认创建名字为“gripConfig.ini”的夹爪）
 *
 * @param gripName 指定操作夹爪名字
 * @param fileName 夹爪配置文件
 * @return int 成功返回0，失败返回其他
 */
extern int CreateGrip2(const char* gripName, const char* fileName);

/**
 * @brief 释放夹爪
 *
 * @param gripName 操作夹爪名字
 * @return int 成功返回0，失败返回其他
 */
extern int DestroyGrip(const char* gripName);

/**
 * @brief 获取指定索引的夹爪名字
 *
 * @param index 夹爪索引
 * @param gripName 返回夹爪名字，可以为NULL
 * @return char* 名字空间地址
 */
extern const char* GetGripName(int index,char* gripName);

/**
 * @brief 夹爪控制
 *
 * @param gripName 操作夹爪名字
 * @param close_percent 夹具夹紧程度，从0.0~1.0之间取值，0.0完全伸展开，1.0处于夹紧状态；
 * @return int 成功返回0，失败返回其他
 */
extern int ControlGrip(const char* gripName, double close_percent);


#ifdef __cplusplus
}
}

#endif

#endif /* GRIP_INTERFACE_H_ */
