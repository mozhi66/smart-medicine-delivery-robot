/**
  ******************************************************************************
  * @file    chassis.h
  * @brief   智能药箱 麦轮底盘驱动（4 电机 / 2 片 TB6612FNG / TIM4 四路 PWM）
  *
  * 职责划分（重要）：
  *   本模块"只做执行"，不做任何定位/导航/决策。
  *   PC 通过 $MV 下发 (前后 y, 横移 x, 自转 rx) 三个归一化速度与持续时长，
  *   本模块负责把它翻译成四个轮子的方向脚电平 + PWM 占空比。
  *
  * 运动学（标准麦轮逆解，与 FTC/ROS 惯例一致）：
  *     FL = y + x + rx        FR = y - x - rx
  *     RL = y - x + rx        RR = y + x - rx
  *   其中：y 前后（+ 前进） / x 横移（+ 右移） / rx 自转（+ 顺时针）
  *   每轮结果限幅到 [-1, 1]，再乘速度上限得到比较值。
  *
  * 与机械臂的关系（安全要点）：
  *   底盘运动和机械臂动作必须互斥 —— 底盘移动过程中机械臂必须锁定，
  *   否则会带臂撞货架。本模块提供 CHASSIS_IsMoving() 供上层判断，
  *   同时 $MV 处理函数在接受指令时会拒绝"机械臂正在动作"的请求。
  *
  * 引脚与定时器分配的完整理由见 main.h 第十节。
  *
  * @author  智能药箱毕业设计（Keil SPL 移植版）
  * @version V1.0
  ******************************************************************************
  */

#ifndef __CHASSIS_H
#define __CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

/** 双工作模式（融合自文件夹 1）：MANUAL=手机蓝牙，AUTO=PC 串口自动对列 */
typedef enum
{
    CHASSIS_MODE_MANUAL = 0,
    CHASSIS_MODE_AUTO   = 1
} ChassisMode_t;

/* ==========================================================================
 * 一、初始化与周期调用
 * ========================================================================== */

/**
 * @brief  底盘模块初始化
 * @note   实现要点（上电安全）：
 *           1) 先把 8 个方向脚初始化为推挽输出并全部拉低；
 *           2) 配置 TIM4 时基与四路 PWM，**比较值先写 0**（占空比 0）；
 *           3) 最后使能 PWM 输出 —— 保证上电瞬间四个轮子都是停的，
 *              不会因为 CCR 初值异常而"窜车"。
 * @param  无
 * @retval 无
 */
void CHASSIS_Init(void);

/**
 * @brief  周期任务：运动计时到期后自动停车
 * @note   必须由主循环频繁调用（建议每个主循环周期都调）。
 *         底盘不像舵机有 1ms 中断，这里用主循环轮询的方式计时：
 *         若"持续运动模式"下达的时长已到，自动减速停车。
 *         函数内部不做阻塞延时。
 * @param  无
 * @retval 无
 */
void CHASSIS_Update(void);

/* ==========================================================================
 * 二、运动控制
 * ========================================================================== */

/**
 * @brief  麦轮运动（对应 $MV,x,y,rx,tt）
 * @note   语义：以 (y, x, rx) 三个归一化速度持续运动 tt 毫秒后自动停车。
 *         tt = 0 表示"一直动到收到新指令或 $MS 停车"（连续控制模式，
 *         手机手动遥控时用得上）。
 *         三个速度分量会被限幅到 [-1, 1]，再乘当前速度上限。
 * @param  y   前后速度，+ 前进 / - 后退，范围 [-1, 1]
 * @param  x   横移速度，+ 右移 / - 左移，范围 [-1, 1]
 * @param  rx  自转速度，+ 顺时针 / - 逆时针，范围 [-1, 1]
 * @param  ms  持续时长(ms)，0 = 不限时（直到下一条指令）
 * @retval 无
 */
void CHASSIS_Move(float y, float x, float rx, uint32_t ms);

/**
 * @brief  立即停车（对应 $MS）
 * @note   把四路 PWM 比较值清零、方向脚全部拉低，并清掉运动计时。
 *         这是急停链路的一环，必须无阻塞、立即生效。
 * @param  无
 * @retval 无
 */
void CHASSIS_Stop(void);

/**
 * @brief  设置速度上限百分比（对应 $MC,pct）
 * @param  pct 百分比 0~100。0 表示此后所有运动都静止（相当于锁死）。
 * @retval 无
 */
void CHASSIS_SetSpeedPct(uint8_t pct);

/**
 * @brief  读取当前速度上限百分比
 * @param  无
 * @retval 百分比 0~100
 */
uint8_t CHASSIS_GetSpeedPct(void);

/**
 * @brief  当前是否正在运动（有非零 PWM 输出）
 * @note   供上层做"底盘动时机械臂必须锁定"的互斥判断。
 * @param  无
 * @retval 1 正在运动；0 静止
 */
uint8_t CHASSIS_IsMoving(void);

/**
 * @brief  读取四个轮子当前的有符号速度（调试/上报用）
 * @param  out 长度至少 4 的 float 数组，顺序 FL/FR/RL/RR
 * @retval 无
 */
void CHASSIS_GetWheelSpeeds(float *out);

/**
 * @brief  读取当前归一化的 (y, x, rx) 速度分量（调试/上报用）
 * @param  y  输出：前后分量（可为 NULL）
 * @param  x  输出：横移分量（可为 NULL）
 * @param  rx 输出：自转分量（可为 NULL）
 * @retval 无
 */
void CHASSIS_GetMotion(float *y, float *x, float *rx);

/** 工作模式（切换时自动停车） */
void          CHASSIS_SetMode(ChassisMode_t mode);
ChassisMode_t CHASSIS_GetMode(void);

/* ==========================================================================
 * 三、底层单元（一般由 CHASSIS_Move 内部调用，单独暴露便于调试）
 * ========================================================================== */

/**
 * @brief  设置单个电机的转向
 * @param  w   电机编号 0=FL / 1=FR / 2=RL / 3=RR
 * @param  fwd 1 = 正转（前进方向），0 = 反转
 * @retval 无
 */
void CHASSIS_SetDir(uint8_t w, uint8_t fwd);

/**
 * @brief  设置单个电机的 PWM 比较值
 * @param  w   电机编号 0=FL / 1=FR / 2=RL / 3=RR
 * @param  v   比较值 0 ~ CHASSIS_PWM_MAX
 * @retval 无
 */
void CHASSIS_SetPwm(uint8_t w, uint16_t v);

/**
 * @brief  设置单个电机的带符号速度
 * @param  w   电机编号 0=FL / 1=FR / 2=RL / 3=RR
 * @param  spd 速度，[-1, 1]，负值表示反转
 * @retval 无
 */
void CHASSIS_SetWheel(uint8_t w, float spd);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H */
