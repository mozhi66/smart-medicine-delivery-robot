/**
  ******************************************************************************
  * @file    servo.h
  * @brief   智能药箱 4 路舵机驱动 / 同步插值 / 限位保护 / Flash 持久化
  *
  * 职责划分（重要）：
  *   本模块"只做执行"，不做任何运动学/视觉/决策。
  *   PC 通过 $SA 下发四个目标角与总时长，本模块保证：
  *       - 四个舵机在同一时刻启动；
  *       - 在 1ms 定时中断里按同一个归一化时间参数 t 做线性插值；
  *       - 同一时刻到达（t=1），即"同步"。
  *
  * 插值模型：
  *     每次运动开始时记录 start[i] = 当前角，target[i] = 目标角；
  *     每 1ms：t = elapsed / total（clamp 到 [0,1]）
  *             cur[i] = start[i] + (target[i] - start[i]) * t
  *     由于四个通道共用同一个 elapsed/total，所以天然同步，
  *     与"每个舵机各自按速度跑"的做法有本质区别（后者快的先到）。
  *
  * 限位策略（最后一道防线）：
  *     任何来自 PC 的角度，在执行前都会与 Flash 中的软限位比较，
  *     越限直接拒绝并返回错误，不会执行、也不会"截断执行"。
  *
  * @author  智能药箱毕业设计（Keil SPL 移植版）
  * @version V1.0
  ******************************************************************************
  */

#ifndef __SERVO_H
#define __SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

/* ==========================================================================
 * 一、数据结构
 * ========================================================================== */

/**
 * @brief 单个舵机通道的运行时状态
 */
typedef struct
{
    float    angle_now;      /**< 当前输出角度(度)。1ms 插值会刷新它；
                                  也是 $POSE 上报的取值 */
    float    angle_start;    /**< 本次插值运动的起点角度(度) */
    float    angle_target;   /**< 本次插值运动的终点角度(度) */
    float    limit_min;      /**< 软限位下限(度)，来自 Flash 配置 */
    float    limit_max;      /**< 软限位上限(度)，来自 Flash 配置 */
    int16_t  trim_us;        /**< 中位微调(us)，叠加在换算出的脉宽上 */
    uint8_t  inverted;       /**< 1 = 输出脉宽在 [MIN,MAX] 内镜像（安装方向相反） */
    uint16_t pulse_now;      /**< 当前实际写入 CCR 的脉宽值(us) */
} ServoChannel_t;

/**
 * @brief 动作序列中的一个"点位"
 * @note  该结构会被原样存进 Flash，修改成员顺序/大小必须同步递增
 *        ACT_STORE_VERSION，否则旧数据会被判定为无效（这正是期望行为）。
 *        当前大小 = 4*4 + 2 + 2 = 20 字节，本身即 4 字节对齐。
 */
typedef struct
{
    float    ang[SERVO_COUNT]; /**< 四个舵机的目标角度(度) */
    uint16_t move_ms;          /**< 走到该点位所用的时间(ms) */
    uint16_t hold_ms;          /**< 到达后停留时间(ms)，用于夹爪动作的时间裕度 */
} ActionPoint_t;

/**
 * @brief 一个动作序列槽
 * @note  成员顺序刻意把 count 放在数组之后，避免结构体填充，
 *        大小 = 8*20 + 1 + 3 = 164 字节。
 */
typedef struct
{
    ActionPoint_t pts[RUN_POINTS_PER_SLOT]; /**< 点位数组 */
    uint8_t       count;                    /**< 有效点位数（<= RUN_POINTS_PER_SLOT） */
    uint8_t       reserved[3];              /**< 保留，保证 4 字节对齐且将来可扩展 */
} ActionSlot_t;

/**
 * @brief Flash 配置区镜像（舵机限位 / HOME / 夹爪 / 微调 / 方向）
 * @note  大小 = 4+2+2+16+16+16+4+4+8+4 = 76 字节，4 字节对齐，无填充。
 */
typedef struct
{
    uint32_t magic;                     /**< 魔数 CFG_STORE_MAGIC，用于判断数据是否有效 */
    uint16_t version;                   /**< 结构版本 CFG_STORE_VERSION */
    uint16_t checksum;                  /**< 从 version 之后所有字节的 16 位累加和 */
    float    limit_min[SERVO_COUNT];    /**< 各舵机软限位下限 */
    float    limit_max[SERVO_COUNT];    /**< 各舵机软限位上限 */
    float    home[SERVO_COUNT];         /**< 各舵机 HOME 角度 */
    float    grip_open;                 /**< 夹爪张开角度（$GC,open） */
    float    grip_close;                /**< 夹爪闭合角度（$GC,close） */
    int16_t  trim_us[SERVO_COUNT];      /**< 各舵机中位微调(us) */
    uint8_t  inverted[SERVO_COUNT];     /**< 各舵机方向取反标志 */
} ServoCfgStore_t;

/**
 * @brief Flash 动作表区镜像
 * @note  大小 = 4+2+2+4+4*164 = 668 字节 < 1024 字节（1KB 页）
 */
typedef struct
{
    uint32_t    magic;                  /**< 魔数 ACT_STORE_MAGIC */
    uint16_t    version;                /**< 结构版本 ACT_STORE_VERSION */
    uint16_t    checksum;               /**< 从 version 之后所有字节的 16 位累加和 */
    uint8_t     slot_count;             /**< 槽位总数 */
    uint8_t     reserved[3];            /**< 保留对齐 */
    ActionSlot_t slots[RUN_SLOT_COUNT]; /**< 槽位数组 */
} ActionStore_t;

/* ==========================================================================
 * 二、初始化与周期调用
 * ========================================================================== */

/**
 * @brief  舵机模块初始化
 * @note   实现要点（上电安全）：
 *           1) 先从 Flash 载入配置，无效则用 main.h 中的默认值；
 *           2) 在"启动 PWM 输出之前"先把 CCR 写成 BOOT 角对应脉宽，
 *              这样 PWM 一使能输出的就是合理脉宽，绝不会先输出一个
 *              随机宽脉冲导致舵机乱抖；
 *           3) 再使能 TIM2 计数器与四路 CCx 输出；
 *           4) 最后启动一次"从 BOOT 角到 HOME 角的平滑同步运动"。
 * @param  无
 * @retval 无
 */
void SERVO_Init(void);

/**
 * @brief  1ms 周期任务：推进插值 / 推进动作序列 / 超时检测
 * @note   必须由 TIM3 的 1ms 更新中断回调调用（见 stm32f10x_it.c 中
 *         TIM3_IRQHandler）。函数内部不使用任何阻塞延时，MCU 占用时间 < 20us。
 * @param  无
 * @retval 无
 */
void SERVO_Update1ms(void);

/* ==========================================================================
 * 三、角度 / 脉宽换算与限位
 * ========================================================================== */

/**
 * @brief  角度(度) -> PWM 脉宽(us)
 * @note   公式：pulse = PULSE_MIN + (angle / SPAN) * (PULSE_MAX - PULSE_MIN)
 *        随后：若 inverted 则在 [MIN,MAX] 内镜像，再叠加 trim_us，
 *        最后钳位到 [SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US]。
 * @param  id   舵机编号 0..SERVO_COUNT-1
 * @param  ang  角度(度)
 * @retval 脉宽(us)，可直接写入 CCR（因为定时器已配置为 1 计数 = 1us）
 */
uint16_t SERVO_AngleToPulse(uint8_t id, float ang);

/**
 * @brief  把某舵机的当前角度立刻刷新到 PWM 输出寄存器
 * @param  id 舵机编号 0..SERVO_COUNT-1
 * @retval 无
 */
void SERVO_ApplyOutput(uint8_t id);

/**
 * @brief  角度限位检查（软限位 + 硬件绝对限位）
 * @param  id  舵机编号
 * @param  ang 待检查角度(度)
 * @retval 1 在限位内，可以执行；0 越限，必须拒绝
 */
uint8_t SERVO_CheckLimit(uint8_t id, float ang);

/**
 * @brief  设置某舵机的软限位（并尝试写入 Flash 持久化）
 * @note   会先做合法性检查：min < max，且都落在硬件绝对限位内，
 *         且留最小行程裕度；非法则返回 0 且不写入。
 *         写入成功后，若当前角度落在新的限位外，会被立即钳位到边界。
 * @param  id     舵机编号
 * @param  min_deg 下限
 * @param  max_deg 上限
 * @retval 1 成功（已存入 RAM 与 Flash）；0 参数非法或 Flash 写入失败
 */
uint8_t SERVO_SetLimits(uint8_t id, float min_deg, float max_deg);

/**
 * @brief  读取某舵机的软限位
 * @param  id      舵机编号
 * @param  min_deg 输出：下限（可为 NULL）
 * @param  max_deg 输出：上限（可为 NULL）
 * @retval 无
 */
void SERVO_GetLimits(uint8_t id, float *min_deg, float *max_deg);

/* ==========================================================================
 * 四、运动控制
 * ========================================================================== */

/**
 * @brief  四路舵机同步插值运动（对应 $SA）
 * @note   实现：所有通道共享同一 elapsed/total，因此同时启动、同时到达。
 *         该调用是非阻塞的，只写参数，实际推进在 1ms 中断里完成。
 * @param  ang     长度为 SERVO_COUNT 的目标角度数组(度)
 * @param  ms      运动总时长(ms)，会被钳位到 [MIN_MOVE_MS, MAX_MOVE_MS]
 * @param  src_cmd 触发本次运动的指令名（"SA"/"HOM"/"RUN"等），
 *                 运动结束时用它拼接 $DONE,<src_cmd>
 * @retval 1 已启动；0 参数越限（此时不会改动任何运动状态）
 */
uint8_t SERVO_StartSyncMove(const float *ang, uint32_t ms, const char *src_cmd);

/**
 * @brief  单舵机运动（对应 $SS，调试用）
 * @note   内部仍然走同步插值框架：其它三路的 start==target==当前角，
 *         即保持不动，只有指定通道变化，保证代码路径唯一、行为一致。
 * @param  id      舵机编号 0..SERVO_COUNT-1
 * @param  ang     目标角度(度)
 * @param  ms      运动总时长(ms)
 * @param  src_cmd 指令名，用于 $DONE 上报
 * @retval 1 已启动；0 参数非法
 */
uint8_t SERVO_MoveSingle(uint8_t id, float ang, uint32_t ms, const char *src_cmd);

/**
 * @brief  夹爪开/合（对应 $GC）
 * @param  is_open 1 = 张开到 grip_open；0 = 闭合到 grip_close
 * @param  ms      运动时长(ms)
 * @param  src_cmd 指令名
 * @retval 1 已启动；0 参数非法
 */
uint8_t SERVO_SetGripper(uint8_t is_open, uint32_t ms, const char *src_cmd);

/**
 * @brief  回到 HOME 位（对应 $HOM）
 * @param  src_cmd 指令名
 * @retval 1 已启动；0 失败
 */
uint8_t SERVO_GoHome(const char *src_cmd);

/**
 * @brief  执行 Flash 中预存的动作序列（对应 $RUN）
 * @note   序列由若干"点位"组成，每个点位 = 一次同步插值运动 + 一段停留。
 *         执行是非阻塞的：本函数只装载序列，推进在 1ms 中断里完成。
 *         全部点位走完后才产生一次 DONE 事件（中间点位不发 DONE）。
 *         装载前会对"所有点位的所有角度"做限位检查，任一点越限则整条
 *         序列不执行，避免走到一半被拒。
 * @param  slot_id 槽位编号，1..RUN_SLOT_COUNT
 * @retval 1 已启动；0 槽位非法 / 空序列 / 角度越限
 */
uint8_t SERVO_StartSequence(uint8_t slot_id);

/**
 * @brief  急停（对应 $EST）
 * @note   立刻停止插值：把每个通道的 target 设为当前角，
 *         清零运动与序列状态。当前 PWM 脉宽保持不变，
 *         因此舵机停在原地并保持力矩（除非 ESTOP_RELEASE_PWM=1）。
 * @param  无
 * @retval 无
 */
void SERVO_StopNow(void);

/**
 * @brief  关闭全部四路 PWM 输出（初始化失败/错误处理时使用）
 * @param  无
 * @retval 无
 */
void Servo_PwmStopAll(void);

/**
 * @brief  当前是否正在运动
 * @param  无
 * @retval 1 正在运动（插值中 或 动作序列执行中）；0 空闲
 */
uint8_t SERVO_IsBusy(void);

/**
 * @brief  超时与堵转检测（由主循环周期调用）
 * @note   正常插值一定会按时结束，本函数是"1ms 中断万一停摆"的最后兜底：
 *         运动开始后经过 (总时长 + MOTION_TIMEOUT_MARGIN_MS) 仍处于忙态，
 *         则强制冻结当前位置并上报 $ERR,04；
 *         连续超时达到阈值再上报 $ERR,05 并终止动作序列。
 * @param  无
 * @retval 无
 */
void SERVO_CheckTimeout(void);

/**
 * @brief  取走一次"运动完成"事件
 * @note   事件由 SERVO_Update1ms 在运动真正结束时置位，
 *         主循环调用本函数取走后发送 $DONE,<cmd>。
 *         只保留最近一次事件（正常通信下不会堆积）。
 * @param  out      输出缓冲区，用于接收指令名
 * @param  out_size 缓冲区大小
 * @retval 1 取到事件；0 无事件
 */
uint8_t SERVO_TakeDoneEvent(char *out, uint8_t out_size);

/**
 * @brief  读取当前四个舵机的角度
 * @param  out 长度至少 SERVO_COUNT 的 float 数组
 * @retval 无
 */
void SERVO_GetPose(float *out);

/**
 * @brief  读取某个通道的运行时状态（调试用）
 * @param  id 舵机编号
 * @retval 通道结构体指针；编号非法时返回最后一个通道的指针
 */
const ServoChannel_t *SERVO_GetChannel(uint8_t id);

/* ==========================================================================
 * 五、Flash 持久化
 * ========================================================================== */

/**
 * @brief  把当前配置（限位/HOME/夹爪/微调/方向）写入 Flash
 * @note   F103 只能按"半字(2 字节)"编程，且必须先整页擦除，
 *         所以实现是：关中断 -> 解锁 -> 擦 1KB 页 -> 逐半字写入 -> 上锁 -> 开中断。
 * @param  无
 * @retval 1 成功；0 失败
 */
uint8_t SERVO_SaveCfgToFlash(void);

/**
 * @brief  把当前 RAM 中的动作表写入 Flash
 * @param  无
 * @retval 1 成功；0 失败
 */
uint8_t SERVO_SaveActionsToFlash(void);

/**
 * @brief  从 Flash 载入配置；魔数/版本/校验和任一不对则加载默认值
 * @param  无
 * @retval 1 载入 Flash 中的有效数据；0 使用了默认值
 */
uint8_t SERVO_LoadCfgFromFlash(void);

/**
 * @brief  从 Flash 载入动作表；无效则加载内置默认演示动作
 * @param  无
 * @retval 1 载入 Flash 中的有效数据；0 使用了默认值
 */
uint8_t SERVO_LoadActionsFromFlash(void);

/**
 * @brief  恢复出厂配置（限位/HOME/夹爪/动作表全部回到 main.h 的默认值并写 Flash）
 * @param  无
 * @retval 1 成功；0 Flash 写入失败
 */
uint8_t SERVO_ResetToDefault(void);

/**
 * @brief  查询某个动作槽的有效点位数
 * @param  slot_id 槽位编号 1..RUN_SLOT_COUNT
 * @retval 点位数；槽位非法返回 0
 */
uint8_t SERVO_GetSlotPointCount(uint8_t slot_id);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */
