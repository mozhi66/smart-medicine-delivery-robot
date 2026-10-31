/**
  ******************************************************************************
  * @file    servo.c
  * @brief   4 路舵机 PWM 驱动、同步插值、限位保护、动作序列与 Flash 持久化
  *          （Keil SPL 移植版）
  *
  * 模块实现要点：
 *   1) PWM：TIM2 四通道（CH1~CH4 = PA0~PA3），1 计数 = 1us，ARR = 19999 -> 20ms/50Hz。
 *      CCR 里直接写"微秒数"，角度到脉宽的换算只做一次浮点乘法。
  *   2) 同步插值：所有通道共用同一组 (elapsed, total)，
  *      cur = start + (target - start) * (elapsed/total)，
  *      因此同时启动、同时到达。推进动作在 1ms 中断里完成，绝不阻塞。
  *   3) 限位：任何运动启动前逐通道检查软限位 + 硬件绝对限位，
  *      越限直接拒绝，不做"截断后执行"。
  *   4) 动作序列：状态机由 (motion.running, seq.active) 两级驱动，
  *      一个点位 = 一次同步运动 + 一段停留；全部点位走完才上报一次 DONE。
  *   5) Flash：F103 只能半字编程且必须整页擦除，写前关中断、写完开中断。
  *
  * 移植说明（HAL -> SPL）：
  *   - HAL_GetTick()  -> GetTick()（my_lib/delay.c，SysTick 1ms 时基）
  *   - __HAL_TIM_SET_COMPARE() -> TIM_SetCompare1..4()
  *   - HAL_TIM_PWM_Start/Stop  -> TIM_Cmd() + TIM_CCxCmd()
  *   - HAL_FLASH_Unlock/Program/Lock -> FLASH_Unlock/ProgramHalfWord/Lock()
  *
  * @author  智能药箱毕业设计（Keil SPL 移植版）
  * @version V1.0
  ******************************************************************************
  */

#include "servo.h"
#include "protocol.h"
#include "string.h"
#include "delay.h"

/* ==========================================================================
 * 内部常量与静态变量
 * ========================================================================== */

/** 四路 PWM 通道对应的 SPL 通道常量。
 *  顺序与舵机编号一致：0=底座 1=大臂 2=小臂 3=夹爪。
 *  对应引脚为 TIM2_CH1~CH4 = PA0/PA1/PA2/PA3（见 main.h 第四节）。 */
static const uint16_t kTimChannel[SERVO_COUNT] =
{
    TIM_Channel_1, TIM_Channel_2, TIM_Channel_3, TIM_Channel_4
};

/** 单个运动过程的内部状态（不对外暴露） */
typedef struct
{
    uint8_t  running;      /**< 1 = 正在插值 */
    uint32_t elapsed_ms;   /**< 已经过的插值步数（每步 1ms） */
    uint32_t total_ms;     /**< 本次运动总时长(ms) */
    uint32_t start_tick;   /**< 本次运动起始的 GetTick() 值，用于超时判定 */
    char     src_cmd[8];   /**< 触发本次运动的指令名 */
} ServoMotion_t;

/** 动作序列执行器的内部状态 */
typedef struct
{
    uint8_t       active;       /**< 1 = 序列正在执行 */
    uint8_t       idx;          /**< 下一个待执行的点位下标 */
    uint8_t       count;        /**< 本序列点位数 */
    uint16_t      hold_remain;  /**< 当前点位到达后还需要停留的 ms 数 */
    ActionPoint_t pts[RUN_POINTS_PER_SLOT]; /**< 点位副本（从 Flash/默认值拷贝） */
} ServoSeq_t;

static ServoChannel_t  s_ch[SERVO_COUNT];   /**< 四路舵机运行时状态 */
static ServoCfgStore_t s_cfg;               /**< 配置的 RAM 镜像 */
static ActionStore_t   s_actions;           /**< 动作表的 RAM 镜像 */
static ServoMotion_t   s_motion;            /**< 当前运动 */
static ServoSeq_t      s_seq;               /**< 当前动作序列 */

static volatile uint8_t s_done_pending;     /**< 是否有一个待上报的 DONE 事件 */
static char             s_done_cmd[8];      /**< DONE 事件对应的指令名 */
static uint8_t          s_stall_cnt;        /**< 连续超时计数，用于判定堵转 */

/* ==========================================================================
 * 七·B 夹爪防烧：夹紧保活状态机（2026-09-29 新增，详见 main.h 七·B 注释）
 * -------------------------------------------------------------------------
 * 痛点：SG90 无反馈，夹住药瓶/顶到机械止点后舵机仍 100% 出力 → 发热烧机。
 * 方案：夹爪闭合到位后自动进入"补压/卸力"循环：
 *         补压相 GRIP_HOLD_ACTIVE_MS : 输出夹紧角（有力，夹住药瓶）
 *         卸力相 GRIP_HOLD_REST_MS   : 输出夹紧角+BACKOFF（脱力休息）
 *       占空比 = ACTIVE/(ACTIVE+REST) ≈ 17%，平均电流约 1/5，热量大幅下降。
 * ========================================================================== */
#if (GRIP_HOLD_ENABLE == 1u)
typedef struct
{
    uint8_t  active;        /**< 1 = 保活循环正在跑 */
    uint8_t  phase;         /**< 0 = 补压相（夹紧力） 1 = 卸力相（休息） */
    uint16_t phase_remain;  /**< 当前相位剩余 ms */
    float    close_deg;     /**< 夹紧角（进入保活时锁定） */
    float    back_deg;      /**< 卸力角 = 夹紧角 + GRIP_HOLD_BACKOFF_DEG */
} GripKeepAlive_t;

static GripKeepAlive_t s_grip_ka;           /**< 夹爪保活状态机 */
#endif  /* GRIP_HOLD_ENABLE */

/* ==========================================================================
 * 内置默认动作表（当 Flash 中无有效数据时使用）
 * 说明：这是一个"取药 -> 递药 -> 放回"的演示序列，角度均落在默认限位内。
 * ========================================================================== */

static const ActionStore_t kDefaultActions =
{
    ACT_STORE_MAGIC,
    ACT_STORE_VERSION,
    0u,                     /* checksum 会在加载默认值时重新计算 */
    RUN_SLOT_COUNT,
    {0u, 0u, 0u},
    {
        /* ---- slot 1：从 HOME 出发的"取药并递出"完整动作 ---- */
        {
            {
                {{ 90.0f,  60.0f, 110.0f, 70.0f},  900u, 300u},  /* 前伸，夹爪张开 */
                {{ 90.0f,  95.0f,  60.0f, 70.0f},  900u, 300u},  /* 下降到药盒 */
                {{ 90.0f,  95.0f,  60.0f, 15.0f},  500u, 400u},  /* 夹紧药盒 */
                {{ 90.0f,  60.0f, 110.0f, 15.0f},  900u, 300u},  /* 抬起 */
                {{ 45.0f,  80.0f,  90.0f, 15.0f}, 1000u, 300u},  /* 转到底座 45 度 */
                {{ 45.0f,  95.0f,  60.0f, 70.0f},  900u, 300u},  /* 放下并张开夹爪 */
                {{ 45.0f,  80.0f,  90.0f, 70.0f},  800u, 200u},  /* 抬起夹爪 */
                {{ 90.0f,  90.0f,  90.0f, 10.0f}, 1200u,   0u}   /* 回到 HOME */
            },
            8u, {0u, 0u, 0u}
        },
        /* ---- slot 2：左右摆动展示（视觉标定时用） ---- */
        {
            {
                {{ 45.0f,  90.0f,  90.0f, 10.0f},  800u, 200u},
                {{135.0f,  90.0f,  90.0f, 10.0f}, 1600u, 200u},
                {{ 90.0f,  90.0f,  90.0f, 10.0f},  800u,   0u},
                {{  0.0f,   0.0f,   0.0f,  0.0f},    0u,   0u},
                {{  0.0f,   0.0f,   0.0f,  0.0f},    0u,   0u},
                {{  0.0f,   0.0f,   0.0f,  0.0f},    0u,   0u},
                {{  0.0f,   0.0f,   0.0f,  0.0f},    0u,   0u},
                {{  0.0f,   0.0f,   0.0f,  0.0f},    0u,   0u}
            },
            3u, {0u, 0u, 0u}
        },
        /* ---- slot 3：夹爪开合测试 ---- */
        {
            {
                {{ 90.0f,  90.0f,  90.0f, 70.0f},  500u, 400u},
                {{ 90.0f,  90.0f,  90.0f, 15.0f},  500u, 400u},
                {{ 90.0f,  90.0f,  90.0f, 70.0f},  500u, 400u},
                {{ 90.0f,  90.0f,  90.0f, 10.0f},  500u,   0u},
                {{  0.0f,   0.0f,   0.0f,  0.0f},    0u,   0u},
                {{  0.0f,   0.0f,   0.0f,  0.0f},    0u,   0u},
                {{  0.0f,   0.0f,   0.0f,  0.0f},    0u,   0u},
                {{  0.0f,   0.0f,   0.0f,  0.0f},    0u,   0u}
            },
            4u, {0u, 0u, 0u}
        },
        /* ---- slot 4：单步回 HOME（最简序列，用于自检） ---- */
        {
            {
                {{ 90.0f,  90.0f,  90.0f, 10.0f}, 1000u,   0u},
                {{  0.0f,   0.0f,   0.0f,  0.0f},    0u,   0u},
                {{  0.0f,   0.0f,   0.0f,  0.0f},    0u,   0u},
                {{  0.0f,   0.0f,   0.0f,  0.0f},    0u,   0u},
                {{  0.0f,   0.0f,   0.0f,  0.0f},    0u,   0u},
                {{  0.0f,   0.0f,   0.0f,  0.0f},    0u,   0u},
                {{  0.0f,   0.0f,   0.0f,  0.0f},    0u,   0u},
                {{  0.0f,   0.0f,   0.0f,  0.0f},    0u,   0u}
            },
            1u, {0u, 0u, 0u}
        }
    }
};

/* ==========================================================================
 * 内部函数前置声明
 * ========================================================================== */

static void     Servo_FillDefaultCfg(void);
static void     Servo_FillDefaultActions(void);
static uint8_t  Servo_FlashWrite(uint32_t addr, const uint8_t *data, uint16_t len);
static void     Servo_FlashRead(uint32_t addr, uint8_t *data, uint16_t len);
static uint16_t Servo_CalcChecksum(const void *st, uint16_t struct_size);
static uint8_t  Servo_ClampMoveMs(uint32_t ms, uint32_t *out_ms);
static void     Servo_MotionStart(const float *target, uint32_t total_ms, const char *src_cmd);
static void     Servo_MotionFinish(void);
static void     Servo_EmitDone(const char *cmd);
static void     Servo_SeqStartPoint(void);
static void     Servo_GripKa_Stop(void);
static void     Servo_GripKa_TryEnter(void);

/* ---- PWM 硬件操作辅助（SPL 版） ---- */

static void Servo_SetCompare(uint8_t id, uint32_t pulse);
static void Servo_PwmStart(uint8_t id);
static void Servo_PwmStop(uint8_t id);

/* ==========================================================================
 * 一、内部：PWM 硬件操作辅助（SPL）
 * ========================================================================== */

/**
 * @brief  把脉宽写入 TIM2 对应通道的 CCR
 * @note   TIM2 已配置为 1 计数 = 1us，因此 CCR 值就是"微秒脉宽"。
 * @param  id    舵机编号 0..SERVO_COUNT-1
 * @param  pulse 脉宽(us)
 * @retval 无
 */
static void Servo_SetCompare(uint8_t id, uint32_t pulse)
{
    switch (id)
    {
        case 0u:  TIM_SetCompare1(TIM2, (uint16_t)pulse); break;
        case 1u:  TIM_SetCompare2(TIM2, (uint16_t)pulse); break;
        case 2u:  TIM_SetCompare3(TIM2, (uint16_t)pulse); break;
        default:  TIM_SetCompare4(TIM2, (uint16_t)pulse); break;
    }
}

/**
 * @brief  使能某一路 PWM 输出（等效 HAL_TIM_PWM_Start）
 * @note   计数器使能只做一次（id==0 时），四路 CCx 分别使能。
 *         注意：调用本函数之前必须已经写好 CCR（上电安全顺序）。
 * @param  id 舵机编号
 * @retval 无
 */
static void Servo_PwmStart(uint8_t id)
{
    if (id == 0u)
    {
        TIM_Cmd(TIM2, ENABLE);
    }
    TIM_CCxCmd(TIM2, kTimChannel[id], TIM_CCx_Enable);
}

/**
 * @brief  关闭某一路 PWM 输出（等效 HAL_TIM_PWM_Stop）
 * @param  id 舵机编号
 * @retval 无
 */
static void Servo_PwmStop(uint8_t id)
{
    TIM_CCxCmd(TIM2, kTimChannel[id], TIM_CCx_Disable);
}

/**
 * @brief  关闭全部 PWM 输出（初始化失败/错误处理时使用，等效原 HAL 版
 *         Error_Handler 中对四路 HAL_TIM_PWM_Stop 的调用）
 * @note   只关闭 CCx 输出，不动 CCR 与 TIM2 计数器。
 * @param  无
 * @retval 无
 */
void Servo_PwmStopAll(void)
{
    uint8_t i;

    for (i = 0u; i < SERVO_COUNT; i++)
    {
        Servo_PwmStop(i);
    }
}

/* ==========================================================================
 * 二、内部：默认值与校验和
 * ========================================================================== */

/**
 * @brief  用 main.h 中的默认宏填充配置结构体
 * @note   只在 Flash 中没有有效数据时调用。填完会重算校验和，
 *         但不会自动写入 Flash（避免每次上电都擦写，延长 Flash 寿命）；
 *         真正的落盘发生在第一次 $CFG 或调用 SERVO_ResetToDefault()。
 * @param  无
 * @retval 无
 */
static void Servo_FillDefaultCfg(void)
{
    s_cfg.magic   = CFG_STORE_MAGIC;
    s_cfg.version = CFG_STORE_VERSION;

    s_cfg.limit_min[SERVO_ID_YAW]      = SERVO0_LIMIT_MIN_DEG;
    s_cfg.limit_min[SERVO_ID_SHOULDER] = SERVO1_LIMIT_MIN_DEG;
    s_cfg.limit_min[SERVO_ID_ELBOW]    = SERVO2_LIMIT_MIN_DEG;
    s_cfg.limit_min[SERVO_ID_GRIPPER]  = SERVO3_LIMIT_MIN_DEG;

    s_cfg.limit_max[SERVO_ID_YAW]      = SERVO0_LIMIT_MAX_DEG;
    s_cfg.limit_max[SERVO_ID_SHOULDER] = SERVO1_LIMIT_MAX_DEG;
    s_cfg.limit_max[SERVO_ID_ELBOW]    = SERVO2_LIMIT_MAX_DEG;
    s_cfg.limit_max[SERVO_ID_GRIPPER]  = SERVO3_LIMIT_MAX_DEG;

    s_cfg.home[SERVO_ID_YAW]      = SERVO0_HOME_DEG;
    s_cfg.home[SERVO_ID_SHOULDER] = SERVO1_HOME_DEG;
    s_cfg.home[SERVO_ID_ELBOW]    = SERVO2_HOME_DEG;
    s_cfg.home[SERVO_ID_GRIPPER]  = SERVO3_HOME_DEG;

    s_cfg.grip_open  = GRIPPER_OPEN_DEG;
    s_cfg.grip_close = GRIPPER_CLOSE_DEG;

    s_cfg.trim_us[SERVO_ID_YAW]      = (int16_t)SERVO0_TRIM_US;
    s_cfg.trim_us[SERVO_ID_SHOULDER] = (int16_t)SERVO1_TRIM_US;
    s_cfg.trim_us[SERVO_ID_ELBOW]    = (int16_t)SERVO2_TRIM_US;
    s_cfg.trim_us[SERVO_ID_GRIPPER]  = (int16_t)SERVO3_TRIM_US;

    s_cfg.inverted[SERVO_ID_YAW]      = (uint8_t)SERVO0_INVERT;
    s_cfg.inverted[SERVO_ID_SHOULDER] = (uint8_t)SERVO1_INVERT;
    s_cfg.inverted[SERVO_ID_ELBOW]    = (uint8_t)SERVO2_INVERT;
    s_cfg.inverted[SERVO_ID_GRIPPER]  = (uint8_t)SERVO3_INVERT;

    s_cfg.checksum = Servo_CalcChecksum(&s_cfg, (uint16_t)sizeof(s_cfg));
}

/**
 * @brief  用内置默认动作表填充 RAM 镜像
 * @note   直接 memcpy 常量表，然后重算校验和。
 * @param  无
 * @retval 无
 */
static void Servo_FillDefaultActions(void)
{
    memcpy(&s_actions, &kDefaultActions, sizeof(s_actions));
    s_actions.checksum = Servo_CalcChecksum(&s_actions, (uint16_t)sizeof(s_actions));
}

/**
 * @brief  计算存储结构体的 16 位累加校验和
 * @note   校验范围 = 结构体第 8 字节到末尾（即跳过 magic/version/checksum
 *         这前 8 个字节）。用 offsetof 校验过：两个存储结构体的第 8 字节
 *         都正好是数据区起点。
 * @param  st           结构体首地址
 * @param  struct_size  结构体大小
 * @retval 累加和（模 65536）
 */
static uint16_t Servo_CalcChecksum(const void *st, uint16_t struct_size)
{
    const uint8_t *p   = (const uint8_t *)st;
    uint16_t       sum = 0u;
    uint16_t       i;

    /* 跳过 magic(4) + version(2) + checksum(2) = 8 字节 */
    for (i = 8u; i < struct_size; i++)
    {
        sum = (uint16_t)(sum + (uint16_t)p[i]);
    }
    return sum;
}

/* ==========================================================================
 * 三、内部：Flash 读写
 * ========================================================================== */

/**
 * @brief  从 Flash 读取一块数据
 * @note   Flash 是内存映射的，直接按字节拷贝即可，无需解锁。
 * @param  addr 起始地址（必须位于有效 Flash 范围内）
 * @param  data 目标缓冲区
 * @param  len  长度（字节）
 * @retval 无
 */
static void Servo_FlashRead(uint32_t addr, uint8_t *data, uint16_t len)
{
    uint16_t i;
    volatile const uint8_t *src = (volatile const uint8_t *)addr;

    for (i = 0u; i < len; i++)
    {
        data[i] = src[i];
    }
}

/**
 * @brief  向 Flash 写入一块数据（先整页擦除，再半字编程）
 * @note   STM32F1 的 Flash 编程单位是"半字(16bit)"，且写入前该地址必须
 *         已被擦除（0xFFFF），因此流程固定为：
 *             关中断 -> FLASH_Unlock -> 擦除 1 页 -> 逐半字 Program
 *             -> FLASH_Lock -> 开中断
 *         关中断的原因：擦写期间 CPU 会被 Flash 取指阻塞，
 *         若此时来了 1ms 中断也不会立刻响应，与其让它迟到不如屏蔽，
 *         避免时序抖动（整个擦写过程约 20~40ms）。
 * @note   Keil 仿真器把 IROM 区配置为只读，仿真时对 0x0800F800 的写入
 *         会被仿真器忽略（Command 窗口出现 access violation 提示），
 *         Flash 读回保持 0xFF/0x00，魔数校验失败 -> 自动使用默认值，
 *         不影响程序正常运行，这是符合预期的行为。
 * @param  addr 目标页起始地址
 * @param  data 源数据
 * @param  len  长度（必须为偶数且 > 0）
 * @retval 1 成功；0 失败（擦除或编程任一步失败都会返回 0）
 */
static uint8_t Servo_FlashWrite(uint32_t addr, const uint8_t *data, uint16_t len)
{
    uint16_t     i;
    FLASH_Status st;

    if ((data == NULL) || (len == 0u) || ((len & 1u) != 0u))
    {
        return 0u;   /* F1 只能半字编程，奇数长度直接拒绝 */
    }

    FLASH_Unlock();

    __disable_irq();

    /* ★ 检查擦除结果：擦除失败（读保护、掉电、目标页不可写）时必须报错，
     *   否则会继续往"没擦干净的页"里编程，写入结果不可预期。 */
    st = FLASH_ErasePage(addr);
    if (st != FLASH_COMPLETE)
    {
        __enable_irq();
        FLASH_Lock();
        return 0u;
    }

    /* ★ 检查每个半字的编程结果：F1 上只有"目标位为 1 时才能写成 0"，
     *   若目标地址残留 0 位（页没擦干净或跨页写），编程会失败。
     *   原实现忽略返回值，导致 Servo_FlashWrite 恒返回 1 ——
     *   $CFG 改完限位、$RST 复位后上位机会认为"已保存"，
     *   而实际 Flash 里是垃圾；下次上电校验和不符便静默退回默认值，
     *   用户完全不知道自己标定的限位丢了。 */
    for (i = 0u; i < len; i += 2u)
    {
        uint16_t hw = (uint16_t)(((uint16_t)data[i]) | ((uint16_t)((uint16_t)data[i + 1u]) << 8));

        st = FLASH_ProgramHalfWord((addr + (uint32_t)i), hw);
        if (st != FLASH_COMPLETE)
        {
            __enable_irq();
            FLASH_Lock();
            return 0u;
        }
    }

    __enable_irq();

    FLASH_Lock();

    return 1u;
}

/* ==========================================================================
 * 四、内部：运动控制辅助
 * ========================================================================== */

/**
 * @brief  把运动时长钳位到合法区间
 * @note   0 或非法值会被替换为 DEFAULT_MOVE_MS，
 *         小于 MIN_MOVE_MS 抬到 MIN，大于 MAX_MOVE_MS 压到 MAX。
 * @param  ms      原始时长
 * @param  out_ms  输出：钳位后的时长
 * @retval 恒为 1（保留返回值以便将来扩展）
 */
static uint8_t Servo_ClampMoveMs(uint32_t ms, uint32_t *out_ms)
{
    uint32_t v = ms;

    if (v == 0u)
    {
        v = DEFAULT_MOVE_MS;
    }
    if (v < (uint32_t)MIN_MOVE_MS)
    {
        v = (uint32_t)MIN_MOVE_MS;
    }
    if (v > (uint32_t)MAX_MOVE_MS)
    {
        v = (uint32_t)MAX_MOVE_MS;
    }
    *out_ms = v;
    return 1u;
}

/* ==========================================================================
 * 七·B 夹爪保活：退出 / 进入 / 推进（防堵转烧舵机，详见 main.h 七·B 注释）
 * ========================================================================== */

/**
 * @brief  退出夹爪保活循环（幂等，任何新运动/急停/复位都先调它）
 * @param  无
 * @retval 无
 */
static void Servo_GripKa_Stop(void)
{
#if (GRIP_HOLD_ENABLE == 1u)
    s_grip_ka.active = 0u;
    s_grip_ka.phase  = 0u;
    s_grip_ka.phase_remain = 0u;
#endif
}

/**
 * @brief  尝试进入夹爪保活循环
 * @note   进入条件（全部满足才进）：
 *           1) GRIP_HOLD_ENABLE 打开；
 *           2) 当前没有运动/序列在跑（刚 finish）；
 *           3) 夹爪目标角 ≈ grip_close（即"刚刚完成一次闭合动作"）。
 *         张开/复位/普通点位运动都不触发 —— 只有闭合到位才需要保活。
 *         进入时锁定当时的夹紧角并预写第一相（补压相）的 CCR。
 * @param  无
 * @retval 无
 */
static void Servo_GripKa_TryEnter(void)
{
#if GRIP_HOLD_ENABLE
    float diff;

    if (s_grip_ka.active != 0u) { return; }
    if ((s_motion.running != 0u) || (s_seq.active != 0u)) { return; }

    diff = s_ch[SERVO_ID_GRIPPER].angle_target - s_cfg.grip_close;
    if (diff < 0.0f) { diff = 0.0f - diff; }
    if (diff > (float)GRIP_HOLD_ANGLE_EPS) { return; }   /* 不是闭合动作 */

    s_grip_ka.active       = 1u;
    s_grip_ka.phase        = 0u;                          /* 先补压 */
    s_grip_ka.phase_remain = (uint16_t)GRIP_HOLD_ACTIVE_MS;
    s_grip_ka.close_deg    = s_cfg.grip_close;
    s_grip_ka.back_deg     = s_cfg.grip_close + (float)GRIP_HOLD_BACKOFF_DEG;
    /* 卸力角钳到张开方向限位内，防止 BACKOFF 把目标推过 MAX */
    if (s_grip_ka.back_deg > s_ch[SERVO_ID_GRIPPER].limit_max)
    {
        s_grip_ka.back_deg = s_ch[SERVO_ID_GRIPPER].limit_max;
    }
    /* 立即写一次补压相脉宽（保证与闭合到位角度连续） */
    Servo_SetCompare(SERVO_ID_GRIPPER,
                     SERVO_AngleToPulse(SERVO_ID_GRIPPER, s_grip_ka.close_deg));
#endif
}

/**
 * @brief  夹爪保活循环推进（SERVO_Update1ms 的 D 节，每 1ms 调一次）
 * @note   补压相结束 → 写卸力角 + 进入卸力相；
 *         卸力相结束 → 写夹紧角 + 回到补压相。
 *         只在相位切换瞬间写 CCR，其余时间不出力变化（省 CPU、少抖动）。
 * @param  无
 * @retval 无
 */
static void Servo_GripKa_Step(void)
{
#if GRIP_HOLD_ENABLE
    if (s_grip_ka.active == 0u) { return; }
    if (s_grip_ka.phase_remain > 0u)
    {
        s_grip_ka.phase_remain--;
        return;
    }

    if (s_grip_ka.phase == 0u)
    {
        /* 补压相 -> 卸力相：脱力休息 */
        s_grip_ka.phase        = 1u;
        s_grip_ka.phase_remain = (uint16_t)GRIP_HOLD_REST_MS;
        s_ch[SERVO_ID_GRIPPER].angle_target = s_grip_ka.back_deg;
        Servo_SetCompare(SERVO_ID_GRIPPER,
                         SERVO_AngleToPulse(SERVO_ID_GRIPPER, s_grip_ka.back_deg));
    }
    else
    {
        /* 卸力相 -> 补压相：重新夹紧 */
        s_grip_ka.phase        = 0u;
        s_grip_ka.phase_remain = (uint16_t)GRIP_HOLD_ACTIVE_MS;
        s_ch[SERVO_ID_GRIPPER].angle_target = s_grip_ka.close_deg;
        Servo_SetCompare(SERVO_ID_GRIPPER,
                         SERVO_AngleToPulse(SERVO_ID_GRIPPER, s_grip_ka.close_deg));
    }
#endif
}

/**
 * @brief  启动一次同步插值运动（内部公共入口）
 * @note   实现：把每个通道的 start 设为"当前角"、target 设为"目标角"，
 *         然后清零 elapsed、记录 total 与起始 tick、置 running=1。
 *         真正的插值在 SERVO_Update1ms() 里推进，本函数立即返回。
 *         启动的同一时刻会把目标角之外的状态一次写完整，避免中断里读到
 *         半新半旧的参数（关键点：先填参数，最后才置 running=1）。
 *         ★ 七·B：新运动开始前先退出夹爪保活循环（新运动优先）。
 * @param  target    长度为 SERVO_COUNT 的目标角数组
 * @param  total_ms  已钳位过的总时长
 * @param  src_cmd   指令名
 * @retval 无
 */
static void Servo_MotionStart(const float *target, uint32_t total_ms, const char *src_cmd)
{
    uint8_t i;

    Servo_GripKa_Stop();    /* 任何新运动都终止夹爪保活，干净交接 */

    for (i = 0u; i < SERVO_COUNT; i++)
    {
        s_ch[i].angle_start  = s_ch[i].angle_now;
        s_ch[i].angle_target = target[i];
    }

    s_motion.elapsed_ms = 0u;
    s_motion.total_ms   = total_ms;
    s_motion.start_tick = GetTick();

    if (src_cmd != NULL)
    {
        uint8_t k = 0u;
        while ((src_cmd[k] != '\0') && (k < (uint8_t)(sizeof(s_motion.src_cmd) - 1u)))
        {
            s_motion.src_cmd[k] = src_cmd[k];
            k++;
        }
        s_motion.src_cmd[k] = '\0';
    }
    else
    {
        s_motion.src_cmd[0] = '\0';
    }

    s_motion.running = 1u;   /* 最后置位，保证中断看到的是完整参数 */
}

/**
 * @brief  强制结束当前运动（不产生 DONE 事件）
 * @note   用于急停与超时恢复：把目标角冻结在当前角，清零序列状态。
 * @param  无
 * @retval 无
 */
static void Servo_MotionFinish(void)
{
    uint8_t i;

    for (i = 0u; i < SERVO_COUNT; i++)
    {
        s_ch[i].angle_target = s_ch[i].angle_now;
        s_ch[i].angle_start  = s_ch[i].angle_now;
    }
    s_motion.running    = 0u;
    s_motion.elapsed_ms = 0u;
}

/**
 * @brief  置位一个 DONE 事件（只保留最近一次）
 * @note   空字符串会被当作"没有指令名"直接忽略。
 *         这一点很重要：急停/超时恢复会把 s_motion.src_cmd 清成空串，
 *         若此时仍置 DONE 事件，主循环会发出 `$DONE,`（空指令名），
 *         PC 端匹配不到任何 cmd_tag，白等一次超时。
 * @param  cmd 指令名
 * @retval 无
 */
static void Servo_EmitDone(const char *cmd)
{
    uint8_t k = 0u;

    if ((cmd == NULL) || (cmd[0] == '\0'))
    {
        return;   /* 无指令名 -> 不产生事件 */
    }
    while ((cmd[k] != '\0') && (k < (uint8_t)(sizeof(s_done_cmd) - 1u)))
    {
        s_done_cmd[k] = cmd[k];
        k++;
    }
    s_done_cmd[k] = '\0';
    s_done_pending = 1u;
}

/**
 * @brief  启动动作序列中的下一个点位
 * @note   从 s_seq.pts[s_seq.idx] 取出角度与时长，调用 Servo_MotionStart，
 *         并把停留时间写入 hold_remain，然后 idx++。
 *         调用前必须保证 idx < count。
 * @param  无
 * @retval 无
 */
static void Servo_SeqStartPoint(void)
{
    const ActionPoint_t *pt = &s_seq.pts[s_seq.idx];
    uint32_t             ms;

    (void)Servo_ClampMoveMs((uint32_t)pt->move_ms, &ms);

    Servo_MotionStart(pt->ang, ms, "RUN");

    s_seq.hold_remain = pt->hold_ms;
    s_seq.idx++;
}

/* ==========================================================================
 * 五、对外：初始化与周期任务
 * ========================================================================== */

/**
 * @brief  舵机模块初始化
 * @note   实现顺序（顺序本身很重要）：
 *           1) 载入配置与动作表（Flash 无效则用默认值）；
 *           2) 把每个通道的"当前角"设为上电假定角 BOOT，并立即换算脉宽；
 *           3) **先写 CCR，再启动 PWM 输出** —— 这样定时器一使能，
 *              输出的就已经是正确脉宽，绝不会先冒出一个随机宽脉冲让舵机乱抖；
 *           4) 启动四路 PWM（TIM_Cmd + CCx 使能）；
 *           5) 启动一次从 BOOT 到 HOME 的平滑同步运动（默认两者相等，
 *              位移为 0，等效于静止不动，最安全）。
 * @param  无
 * @retval 无
 */
void SERVO_Init(void)
{
    uint8_t  i;
    float    boot[SERVO_COUNT];

    /* 1) 载入持久化数据 */
    (void)SERVO_LoadCfgFromFlash();
    (void)SERVO_LoadActionsFromFlash();

    /* 2) 通道初始化 + 上电假定角 */
    boot[SERVO_ID_YAW]      = SERVO0_BOOT_DEG;
    boot[SERVO_ID_SHOULDER] = SERVO1_BOOT_DEG;
    boot[SERVO_ID_ELBOW]    = SERVO2_BOOT_DEG;
    boot[SERVO_ID_GRIPPER]  = SERVO3_BOOT_DEG;

    for (i = 0u; i < SERVO_COUNT; i++)
    {
        s_ch[i].limit_min   = s_cfg.limit_min[i];
        s_ch[i].limit_max   = s_cfg.limit_max[i];
        s_ch[i].trim_us     = s_cfg.trim_us[i];
        s_ch[i].inverted    = s_cfg.inverted[i];

        /* 上电角度先钳到限位内，防止默认宏写错导致舵机憋死 */
        if (boot[i] < s_ch[i].limit_min) { boot[i] = s_ch[i].limit_min; }
        if (boot[i] > s_ch[i].limit_max) { boot[i] = s_ch[i].limit_max; }

        s_ch[i].angle_now    = boot[i];
        s_ch[i].angle_start  = boot[i];
        s_ch[i].angle_target = boot[i];
        s_ch[i].pulse_now    = SERVO_AngleToPulse(i, boot[i]);

        /* 3) 先写 CCR */
        Servo_SetCompare(i, (uint32_t)s_ch[i].pulse_now);
    }

    /* 4) 使能四路 PWM 输出 */
    for (i = 0u; i < SERVO_COUNT; i++)
    {
        Servo_PwmStart(i);
    }

    /* 5) 平滑归位 */
    s_motion.running    = 0u;
    s_seq.active        = 0u;
    s_done_pending      = 0u;
    s_stall_cnt         = 0u;
    s_motion.src_cmd[0] = '\0';
    Servo_GripKa_Stop();     /* 上电保活状态必须为干净初值 */

    (void)SERVO_StartSyncMove(s_cfg.home, (uint32_t)BOOT_HOME_MOVE_MS, "HOM");
}

/**
 * @brief  1ms 周期任务
 * @note   执行顺序：
 *           A. 推进插值（若正在运动）：
 *                 elapsed++；t = elapsed/total（>1 取 1）；
 *                 cur = start + (target-start)*t；写 CCR；
 *                 elapsed >= total 时结束本次运动并置"完成"标志。
 *           B. 推进动作序列（若序列激活且当前无运动）：
 *                 先倒计时停留，停留结束则启动下一个点位；
 *                 点位全部走完则结束序列。
 *           C. 若本次运动完成且没有序列在跑，则产生 DONE 事件。
 *              有序列在跑时不产生（中间点位不上报），由序列结束时统一上报。
 * @param  无
 * @retval 无
 */
void SERVO_Update1ms(void)
{
    uint8_t  i;
    uint8_t  finished = 0u;
    float    t;

    /* ---- A. 插值推进 ---- */
    if (s_motion.running != 0u)
    {
        s_motion.elapsed_ms++;

        if (s_motion.elapsed_ms >= s_motion.total_ms)
        {
            t = 1.0f;
            s_motion.running = 0u;
            finished = 1u;
        }
        else
        {
            t = (float)s_motion.elapsed_ms / (float)s_motion.total_ms;
        }

        for (i = 0u; i < SERVO_COUNT; i++)
        {
            s_ch[i].angle_now = s_ch[i].angle_start +
                                (s_ch[i].angle_target - s_ch[i].angle_start) * t;
            SERVO_ApplyOutput(i);
        }
    }

    /* ---- B. 动作序列推进 ---- */
    if ((s_seq.active != 0u) && (s_motion.running == 0u))
    {
        if (s_seq.hold_remain > 0u)
        {
            s_seq.hold_remain--;          /* 到达点位后的停留 */
        }
        else if (s_seq.idx < s_seq.count)
        {
            Servo_SeqStartPoint();        /* 启动下一个点位 */
        }
        else
        {
            s_seq.active = 0u;            /* 序列执行完毕 */
        }
    }

    /* ---- C. 完成事件上报 ---- */
    if (finished != 0u)
    {
        if (s_seq.active == 0u)
        {
            Servo_EmitDone(s_motion.src_cmd);
        }
        /* 有序列在跑时，"本次点位完成"不上报，等整条序列跑完再报一次 */
    }

    /* ---- D. 夹爪保活（七·B 防烧）----
     * ① 本次运动刚结束且最后动作是"夹爪闭合到位" → 进入保活循环；
     * ② 保活循环在跑 → 推进补压/卸力相位。
     *    注意顺序：先 TryEnter（可能刚激活）再 Step；刚进入的那 1ms
     *    直接走补压相倒计时即可，不会抢跑。 */
    if (finished != 0u)
    {
        Servo_GripKa_TryEnter();
    }
    Servo_GripKa_Step();
}

/**
 * @brief  超时与堵转检测（由主循环周期调用）
 * @note   正常情况下插值一定会按时结束，本函数是"1ms 中断挂了"或
 *         "GetTick 被阻塞"的最后兜底：
 *         若运动开始后经过 (total + MARGIN) 仍显示 busy，
 *         则认为执行超时，强制冻结并上报 $ERR,04；
 *         连续超时达到 MOTION_STALL_THRESHOLD 次，上报 $ERR,05 并终止序列。
 * @param  无
 * @retval 无
 */
void SERVO_CheckTimeout(void)
{
    uint32_t limit;

    if ((s_motion.running == 0u) && (s_seq.active == 0u))
    {
        s_stall_cnt = 0u;
        return;
    }

    limit = s_motion.total_ms + (uint32_t)MOTION_TIMEOUT_MARGIN_MS;

    if ((uint32_t)(GetTick() - s_motion.start_tick) > limit)
    {
        s_stall_cnt++;
        PROTO_SendErr((uint8_t)PROTO_ERR_TIMEOUT, "TMO");

        Servo_MotionFinish();   /* 冻结在当前位置，避免继续乱动 */

        if (s_stall_cnt >= (uint8_t)MOTION_STALL_THRESHOLD)
        {
            PROTO_SendErr((uint8_t)PROTO_ERR_SERVO, "STALL");
            s_seq.active = 0u;
            s_stall_cnt  = 0u;
        }
    }
}

/* ==========================================================================
 * 六、对外：角度/脉宽换算与限位
 * ========================================================================== */

/**
 * @brief  角度(度) -> PWM 脉宽(us)
 * @note   实现：
 *           pulse = PULSE_MIN + (ang / SPAN) * (PULSE_MAX - PULSE_MIN)
 *           若 inverted 则在 [MIN,MAX] 内镜像：pulse = MIN + MAX - pulse
 *           再叠加 trim_us，最后钳位到 [MIN,MAX] 并四舍五入到整数 us。
 *         因为 TIM2 已配置为 1 计数 = 1us，返回值可直接写进 CCR。
 * @param  id  舵机编号
 * @param  ang 角度
 * @retval 脉宽(us)
 */
uint16_t SERVO_AngleToPulse(uint8_t id, float ang)
{
    float pulse;

    if (id >= SERVO_COUNT)
    {
        id = (uint8_t)(SERVO_COUNT - 1u);
    }

    pulse = (float)SERVO_PULSE_MIN_US +
            (ang / SERVO_ANGLE_SPAN_DEG) *
            (float)((uint32_t)SERVO_PULSE_MAX_US - (uint32_t)SERVO_PULSE_MIN_US);

    if (s_ch[id].inverted != 0u)
    {
        pulse = (float)SERVO_PULSE_MIN_US + (float)SERVO_PULSE_MAX_US - pulse;
    }

    pulse += (float)s_ch[id].trim_us;

    if (pulse < (float)SERVO_PULSE_MIN_US) { pulse = (float)SERVO_PULSE_MIN_US; }
    if (pulse > (float)SERVO_PULSE_MAX_US) { pulse = (float)SERVO_PULSE_MAX_US; }

    return (uint16_t)(pulse + 0.5f);
}

/**
 * @brief  把某舵机的当前角度立刻刷新到 PWM 输出寄存器
 * @note   实现：换算脉宽 -> 写入 CCR -> 记录到 pulse_now 便于查询。
 * @param  id 舵机编号
 * @retval 无
 */
void SERVO_ApplyOutput(uint8_t id)
{
    uint16_t pulse;

    if (id >= SERVO_COUNT)
    {
        return;
    }

    pulse = SERVO_AngleToPulse(id, s_ch[id].angle_now);
    s_ch[id].pulse_now = pulse;

    Servo_SetCompare(id, (uint32_t)pulse);
}

/**
 * @brief  角度限位检查
 * @note   检查两个层次：
 *           - 硬件绝对限位 [SERVO_ABS_MIN_DEG, SERVO_ABS_MAX_DEG]（不可配置）
 *           - 该通道的软限位 [limit_min, limit_max]（$CFG 可改，存在 Flash）
 *         带 SERVO_LIMIT_EPS_DEG 容差，避免浮点与 PC 端取整带来误判。
 * @param  id  舵机编号
 * @param  ang 待检查角度
 * @retval 1 合法；0 越限
 */
uint8_t SERVO_CheckLimit(uint8_t id, float ang)
{
    float min_v;
    float max_v;

    if (id >= SERVO_COUNT)
    {
        return 0u;
    }

    min_v = s_ch[id].limit_min - SERVO_LIMIT_EPS_DEG;
    max_v = s_ch[id].limit_max + SERVO_LIMIT_EPS_DEG;

    if (min_v < SERVO_ABS_MIN_DEG) { min_v = SERVO_ABS_MIN_DEG; }
    if (max_v > SERVO_ABS_MAX_DEG) { max_v = SERVO_ABS_MAX_DEG; }

    if ((ang < min_v) || (ang > max_v))
    {
        return 0u;
    }
    return 1u;
}

/**
 * @brief  设置某舵机的软限位并写入 Flash
 * @note   校验规则：
 *           - id 合法；
 *           - min < max（且至少相差 1 度，避免限位区间退化成一个点）；
 *           - min/max 都落在硬件绝对限位内。
 *         成功后：更新 RAM -> 若当前角越界则钳位并立即刷新输出 -> 写 Flash。
 * @param  id      舵机编号
 * @param  min_deg 下限
 * @param  max_deg 上限
 * @retval 1 成功；0 失败
 */
uint8_t SERVO_SetLimits(uint8_t id, float min_deg, float max_deg)
{
    if (id >= SERVO_COUNT)
    {
        return 0u;
    }
    if ((min_deg + 1.0f) > max_deg)
    {
        return 0u;
    }
    if ((min_deg < SERVO_ABS_MIN_DEG) || (max_deg > SERVO_ABS_MAX_DEG))
    {
        return 0u;
    }

    s_ch[id].limit_min = min_deg;
    s_ch[id].limit_max = max_deg;

    s_cfg.limit_min[id] = min_deg;
    s_cfg.limit_max[id] = max_deg;

    /* 当前角度若落在新的限位外，立即钳位并输出，避免保持在危险位置 */
    if (s_ch[id].angle_now < min_deg)
    {
        s_ch[id].angle_now    = min_deg;
        s_ch[id].angle_target = min_deg;
        SERVO_ApplyOutput(id);
    }
    else if (s_ch[id].angle_now > max_deg)
    {
        s_ch[id].angle_now    = max_deg;
        s_ch[id].angle_target = max_deg;
        SERVO_ApplyOutput(id);
    }
    else
    {
        /* 角度在限位内，无需处理 */
    }

    return SERVO_SaveCfgToFlash();
}

/**
 * @brief  读取某舵机的软限位
 * @param  id      舵机编号
 * @param  min_deg 输出：下限
 * @param  max_deg 输出：上限
 * @retval 无
 */
void SERVO_GetLimits(uint8_t id, float *min_deg, float *max_deg)
{
    uint8_t k = (id >= SERVO_COUNT) ? (uint8_t)(SERVO_COUNT - 1u) : id;

    if (min_deg != NULL) { *min_deg = s_ch[k].limit_min; }
    if (max_deg != NULL) { *max_deg = s_ch[k].limit_max; }
}

/* ==========================================================================
 * 七、对外：运动控制
 * ========================================================================== */

/**
 * @brief  四路舵机同步插值运动（$SA）
 * @note   实现：先"全部校验通过再动作"—— 任一通道越限立即返回 0，
 *         且不会修改任何运动状态（不会出现"两路动了、两路没动"的半截状态）。
 * @param  ang     目标角数组
 * @param  ms      总时长
 * @param  src_cmd 指令名
 * @retval 1 已启动；0 参数越限
 */
uint8_t SERVO_StartSyncMove(const float *ang, uint32_t ms, const char *src_cmd)
{
    uint8_t  i;
    uint32_t total;

    if (ang == NULL)
    {
        return 0u;
    }

    for (i = 0u; i < SERVO_COUNT; i++)
    {
        if (SERVO_CheckLimit(i, ang[i]) == 0u)
        {
            return 0u;   /* 任一路越限，整条指令不执行 */
        }
    }

    (void)Servo_ClampMoveMs(ms, &total);

    /* 新的运动指令到来时，若正在执行动作序列，先终止序列 */
    s_seq.active = 0u;

    Servo_MotionStart(ang, total, src_cmd);
    return 1u;
}

/**
 * @brief  单舵机运动（$SS）
 * @note   实现：构造一个"四路目标角"数组，其中只有指定通道为目标角，
 *         其余三路保持当前角（start == target == now，位移为 0）。
 *         这样复用了同步插值框架，代码路径唯一，行为与其它指令完全一致。
 * @param  id      舵机编号
 * @param  ang     目标角
 * @param  ms      时长
 * @param  src_cmd 指令名
 * @retval 1 已启动；0 参数非法
 */
uint8_t SERVO_MoveSingle(uint8_t id, float ang, uint32_t ms, const char *src_cmd)
{
    float    target[SERVO_COUNT];
    uint8_t  i;
    uint32_t total;

    if (id >= SERVO_COUNT)
    {
        return 0u;
    }
    if (SERVO_CheckLimit(id, ang) == 0u)
    {
        return 0u;
    }

    for (i = 0u; i < SERVO_COUNT; i++)
    {
        target[i] = s_ch[i].angle_now;
    }
    target[id] = ang;

    (void)Servo_ClampMoveMs(ms, &total);

    s_seq.active = 0u;

    Servo_MotionStart(target, total, src_cmd);
    return 1u;
}

/**
 * @brief  夹爪开/合（$GC）
 * @note   实现：把 $GC,open / $GC,close 映射为 servo3 的两个预设角度，
 *         走 SERVO_MoveSingle，因此只有夹爪动、其余关节保持不动。
 * @param  is_open 1 = 张开
 * @param  ms      时长
 * @param  src_cmd 指令名
 * @retval 1 已启动；0 参数非法
 */
uint8_t SERVO_SetGripper(uint8_t is_open, uint32_t ms, const char *src_cmd)
{
    float ang = (is_open != 0u) ? s_cfg.grip_open : s_cfg.grip_close;
    uint32_t use_ms = ms;

    /* ★ 七·B 第一层防护（慢速扫角）：闭合方向强制拉长运动时间，
     *   平稳逼近机械止点，减小到位瞬间的冲击与过冲。
     *   张开方向不受限（离开止点是安全的）。 */
    if (is_open == 0u)
    {
        if (use_ms < (uint32_t)GRIP_SCAN_CLOSE_MS)
        {
            use_ms = (uint32_t)GRIP_SCAN_CLOSE_MS;
        }
    }

    return SERVO_MoveSingle(SERVO_ID_GRIPPER, ang, use_ms, src_cmd);
}

/**
 * @brief  回到 HOME 位（$HOM）
 * @param  src_cmd 指令名
 * @retval 1 已启动；0 失败
 */
uint8_t SERVO_GoHome(const char *src_cmd)
{
    return SERVO_StartSyncMove(s_cfg.home, (uint32_t)HOME_MOVE_MS, src_cmd);
}

/**
 * @brief  执行 Flash 中预存的动作序列（$RUN）
 * @note   实现：
 *           - 槽位号 1..RUN_SLOT_COUNT，越界返回 0；
 *           - 装载前遍历"所有有效点位的所有角度"做限位检查，
 *             任一点越限则整条序列不装载（避免执行到一半被拒）；
 *           - 把点位拷进内部序列状态（拷副本，避免运行中被 Flash 数据干扰）；
 *           - 置 seq.active = 1，第一个点位由 SERVO_Update1ms 自动启动；
 *           - 同时把当前正在跑的普通运动停掉。
 * @param  slot_id 槽位编号 1..RUN_SLOT_COUNT
 * @retval 1 已启动；0 失败
 */
uint8_t SERVO_StartSequence(uint8_t slot_id)
{
    const ActionSlot_t *slot;
    uint8_t             i, k;
    uint16_t            bytes;

    if ((slot_id < 1u) || (slot_id > (uint8_t)RUN_SLOT_COUNT))
    {
        return 0u;
    }

    slot = &s_actions.slots[slot_id - 1u];

    if ((slot->count == 0u) || (slot->count > (uint8_t)RUN_POINTS_PER_SLOT))
    {
        return 0u;
    }

    for (i = 0u; i < slot->count; i++)
    {
        for (k = 0u; k < SERVO_COUNT; k++)
        {
            if (SERVO_CheckLimit(k, slot->pts[i].ang[k]) == 0u)
            {
                return 0u;
            }
        }
    }

    bytes = (uint16_t)((uint16_t)slot->count * (uint16_t)sizeof(ActionPoint_t));
    memcpy(s_seq.pts, slot->pts, bytes);

    s_seq.count       = slot->count;
    s_seq.idx         = 0u;
    s_seq.hold_remain = 0u;

    s_motion.running  = 0u;     /* 停掉正在跑的普通运动 */
    s_seq.active      = 1u;     /* 序列在下一个 1ms 节拍启动首个点位 */

    return 1u;
}

/**
 * @brief  急停（$EST）
 * @note   实现要点（响应速度优先）：
 *           - 立刻清零 running 与 seq.active，插值不再推进；
 *           - 把每个通道的 target 冻结为当前角（等价于"目标=当前"）；
 *           - 保持当前 CCR 不变，舵机停在原地并维持力矩；
 *           - 若 ESTOP_RELEASE_PWM = 1，则额外关闭 PWM 输出（脱力）。
 *         因为本函数在正常上下文被调用（主循环解析到 EST 后立刻调用），
 *         从收到最后一个字节到冻结的延迟 < 1ms，远快于舵机响应时间。
 * @param  无
 * @retval 无
 */
void SERVO_StopNow(void)
{
    uint8_t i;

    Servo_GripKa_Stop();     /* 急停同时退出夹爪保活循环 */

    s_motion.running = 0u;
    s_seq.active     = 0u;

    /* ★ 清空"触发本次运动的指令名"与残留的序列进度。
     *   这两处如果不一起清，会留下两个隐患：
     *   1) src_cmd 残留：SERVO_StopNow 冻结了运动但没有产生 DONE，
     *      若之后又发生一次"无源指令"的完成事件，Servo_EmitDone(s_motion.src_cmd)
     *      会把这**上一个被急停的指令名**报上去，PC 端等待同名 DONE 时可能被误满足。
     *   2) idx / hold_remain 残留：虽然 SERVO_StartSequence 会重新赋值，
     *      但保持"急停后状态全清"更不容易出错。 */
    s_motion.src_cmd[0]  = '\0';
    s_seq.idx            = 0u;
    s_seq.count          = 0u;
    s_seq.hold_remain    = 0u;

    for (i = 0u; i < SERVO_COUNT; i++)
    {
        s_ch[i].angle_target = s_ch[i].angle_now;
        s_ch[i].angle_start  = s_ch[i].angle_now;

#if ESTOP_RELEASE_PWM
        Servo_PwmStop(i);
#endif
    }
}

/**
 * @brief  当前是否正在运动
 * @param  无
 * @retval 1 忙；0 空闲
 */
uint8_t SERVO_IsBusy(void)
{
    return ((s_motion.running != 0u) || (s_seq.active != 0u)) ? 1u : 0u;
}

/**
 * @brief  取走一次"运动完成"事件
 * @note   实现：**在临界区内**完成"读-清-拷"整个序列，避免被 1ms 中断打断。
 *
 *         为什么必须加临界区：Servo_EmitDone() 在 TIM3 的 1ms 中断里被调用，
 *         会改写 s_done_cmd[]。而本函数过去是
 *             判 pending -> 清 pending -> 逐字节拷 s_done_cmd
 *         若在第 3 步的拷贝过程中恰好被中断打断、且中断里又置了一次 DONE，
 *         拷出来的就会是**两条指令名的混合**（例如 "SA" 与 "RUN" 拼成 "SAN"），
 *         PC 端匹配不上任何 cmd_tag，只能白等到超时。
 *         窗口很小，但 $RUN 序列结束、连续运动相接的时刻正是 DONE 高发点。
 *
 *         只保留最近一次事件（正常通信节奏下不会堆积）。
 *         __disable_irq/__enable_irq 在此不嵌套（本函数只在主循环调用），
 *         因此直接开关中断即可，无需保存 PRIMASK。
 * @param  out      输出指令名
 * @param  out_size 缓冲区大小
 * @retval 1 取到；0 无事件
 */
uint8_t SERVO_TakeDoneEvent(char *out, uint8_t out_size)
{
    uint8_t k = 0u;
    uint8_t got;

    __disable_irq();

    got = s_done_pending;
    if (got != 0u)
    {
        s_done_pending = 0u;

        if ((out != NULL) && (out_size > 0u))
        {
            while ((s_done_cmd[k] != '\0') && (k < (uint8_t)(out_size - 1u)))
            {
                out[k] = s_done_cmd[k];
                k++;
            }
            out[k] = '\0';
        }
        /* 拷完即清空源串，避免"取过一次后又被拼上残留字节" */
        s_done_cmd[0] = '\0';
    }

    __enable_irq();

    return got;
}

/**
 * @brief  读取当前四个舵机的角度
 * @param  out 长度 >= SERVO_COUNT 的数组
 * @retval 无
 */
void SERVO_GetPose(float *out)
{
    uint8_t i;

    if (out == NULL)
    {
        return;
    }
    for (i = 0u; i < SERVO_COUNT; i++)
    {
        out[i] = s_ch[i].angle_now;
    }
}

/**
 * @brief  读取某个通道的运行时状态
 * @param  id 舵机编号
 * @retval 通道指针
 */
const ServoChannel_t *SERVO_GetChannel(uint8_t id)
{
    if (id >= SERVO_COUNT)
    {
        id = (uint8_t)(SERVO_COUNT - 1u);
    }
    return &s_ch[id];
}

/* ==========================================================================
 * 八、对外：Flash 持久化
 * ========================================================================== */

/**
 * @brief  把当前配置写入 Flash
 * @note   重算校验和 -> 整页擦除 -> 半字写入。
 *         只占用 1 页（1KB），擦写次数有限（约 1 万次），
 *         因此只有 $CFG 这种低频操作才会触发。
 * @param  无
 * @retval 1 成功；0 失败
 */
uint8_t SERVO_SaveCfgToFlash(void)
{
    s_cfg.magic    = CFG_STORE_MAGIC;
    s_cfg.version  = CFG_STORE_VERSION;
    s_cfg.checksum = Servo_CalcChecksum(&s_cfg, (uint16_t)sizeof(s_cfg));

    return Servo_FlashWrite(CFG_FLASH_ADDR,
                            (const uint8_t *)&s_cfg,
                            (uint16_t)sizeof(s_cfg));
}

/**
 * @brief  把当前 RAM 中的动作表写入 Flash
 * @param  无
 * @retval 1 成功；0 失败
 */
uint8_t SERVO_SaveActionsToFlash(void)
{
    s_actions.magic    = ACT_STORE_MAGIC;
    s_actions.version  = ACT_STORE_VERSION;
    s_actions.checksum = Servo_CalcChecksum(&s_actions, (uint16_t)sizeof(s_actions));

    return Servo_FlashWrite(ACT_FLASH_ADDR,
                            (const uint8_t *)&s_actions,
                            (uint16_t)sizeof(s_actions));
}

/**
 * @brief  从 Flash 载入配置
 * @note   三重校验：魔数 -> 版本号 -> 16 位累加和。任一不符即认为无效，
 *         填充默认值（但不自动写回，避免上电就擦写 Flash）。
 * @param  无
 * @retval 1 使用了 Flash 数据；0 使用了默认值
 */
uint8_t SERVO_LoadCfgFromFlash(void)
{
    ServoCfgStore_t tmp;

    Servo_FlashRead(CFG_FLASH_ADDR, (uint8_t *)&tmp, (uint16_t)sizeof(tmp));

    if (tmp.magic != CFG_STORE_MAGIC)
    {
        Servo_FillDefaultCfg();
        return 0u;
    }
    if (tmp.version != CFG_STORE_VERSION)
    {
        Servo_FillDefaultCfg();
        return 0u;
    }
    if (tmp.checksum != Servo_CalcChecksum(&tmp, (uint16_t)sizeof(tmp)))
    {
        Servo_FillDefaultCfg();
        return 0u;
    }

    memcpy(&s_cfg, &tmp, sizeof(s_cfg));
    return 1u;
}

/**
 * @brief  从 Flash 载入动作表
 * @param  无
 * @retval 1 使用了 Flash 数据；0 使用了内置默认动作
 */
uint8_t SERVO_LoadActionsFromFlash(void)
{
    ActionStore_t tmp;

    Servo_FlashRead(ACT_FLASH_ADDR, (uint8_t *)&tmp, (uint16_t)sizeof(tmp));

    if ((tmp.magic != ACT_STORE_MAGIC) ||
        (tmp.version != ACT_STORE_VERSION) ||
        (tmp.checksum != Servo_CalcChecksum(&tmp, (uint16_t)sizeof(tmp))))
    {
        Servo_FillDefaultActions();
        return 0u;
    }

    memcpy(&s_actions, &tmp, sizeof(s_actions));

    /* 防止 Flash 中的 count 越界（例如旧版本残留数据） */
    {
        uint8_t i;
        for (i = 0u; i < (uint8_t)RUN_SLOT_COUNT; i++)
        {
            if (s_actions.slots[i].count > (uint8_t)RUN_POINTS_PER_SLOT)
            {
                s_actions.slots[i].count = 0u;
            }
        }
        if ((s_actions.slot_count == 0u) || (s_actions.slot_count > (uint8_t)RUN_SLOT_COUNT))
        {
            Servo_FillDefaultActions();
            return 0u;
        }
    }

    return 1u;
}

/**
 * @brief  恢复出厂配置
 * @note   重新填充默认配置与默认动作表，并全部写入 Flash。
 * @param  无
 * @retval 1 成功；0 失败
 */
uint8_t SERVO_ResetToDefault(void)
{
    uint8_t ok = 1u;

    Servo_GripKa_Stop();          /* 复位退出保活，回到干净状态 */

    Servo_FillDefaultCfg();
    Servo_FillDefaultActions();

    if (SERVO_SaveCfgToFlash() == 0u)     { ok = 0u; }
    if (SERVO_SaveActionsToFlash() == 0u) { ok = 0u; }

    return ok;
}

/**
 * @brief  查询某个动作槽的有效点位数
 * @param  slot_id 1..RUN_SLOT_COUNT
 * @retval 点位数；非法返回 0
 */
uint8_t SERVO_GetSlotPointCount(uint8_t slot_id)
{
    if ((slot_id < 1u) || (slot_id > (uint8_t)RUN_SLOT_COUNT))
    {
        return 0u;
    }
    return s_actions.slots[slot_id - 1u].count;
}
