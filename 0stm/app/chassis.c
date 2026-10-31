/**
  ******************************************************************************
  * @file    chassis.c
  * @brief   智能药箱 麦轮底盘驱动实现（4 电机 / 2 片 TB6612FNG / TIM4 四路 PWM）
  *
 * 本文件由原独立底盘工程（0底盘/HC05控制小车/user/main.c）迁移而来，
 * 并入 0stm 机械臂固件后的调整（理由见 main.h 第十节）：
 *   1) 麦轮逆解公式（标准 FTC/ROS 惯例），改错会让斜走/横移方向乱；
 *   2) s_dir_pol[] 极性翻转机制 —— 装反了改这里，不用重新接线；
 *   - 定时器：PWM 由 TIM3 搬到 TIM4（TIM3 留给机械臂 1ms 插值）；
 *   - 串口：HC-05 由 USART1 搬到 USART3（USART1 保留给 PC 调试）；
 *   - 方向脚：全部重排，与舵机 PA0~PA3 错开（见 main.h 第十节）；
 *   - 安全：失联自停 + 运动到期双计时，且与机械臂动作互斥。
 *
 *   3) 单字符命令（F/B/L/R...）-> $MV 帧协议（统一校验、统一 ACK/DONE）；
 *   4) 速度上限百分比 $MC 可调；运动状态 $MQ 可查询。
 *
 * @author  智能药箱毕业设计（Keil SPL 移植版）
 * @version V1.0
  ******************************************************************************
  */

#include "chassis.h"
#include "servo.h"      /* 需要 SERVO_IsBusy()：底盘与机械臂的运动互斥 */
#include "delay.h"      /* 需要 GetTick()：基于 SysTick 的毫秒计时，比自己扣减准 */

/* ==========================================================================
 * 一、引脚表（改接线只改这里；宏定义在 main.h 第十节）
 * ========================================================================== */

/** 电机编号：0=FL(前左) 1=FR(前右) 2=RL(后左) 3=RR(后右) */
static GPIO_TypeDef *const s_dir_port[CHASSIS_MOTOR_COUNT] =
{
    CHASSIS_DIR_PORT_FL,   /* 0 = FL */
    CHASSIS_DIR_PORT_FR,   /* 1 = FR */
    CHASSIS_DIR_PORT_RL,   /* 2 = RL */
    CHASSIS_DIR_PORT_RR    /* 3 = RR */
};

static const uint16_t s_dir_in1[CHASSIS_MOTOR_COUNT] =
{
    CHASSIS_DIR_IN1_FL,
    CHASSIS_DIR_IN1_FR,
    CHASSIS_DIR_IN1_RL,
    CHASSIS_DIR_IN1_RR
};

static const uint16_t s_dir_in2[CHASSIS_MOTOR_COUNT] =
{
    CHASSIS_DIR_IN2_FL,
    CHASSIS_DIR_IN2_FR,
    CHASSIS_DIR_IN2_RL,
    CHASSIS_DIR_IN2_RR
};

/**
 * @brief 各电机方向极性。
 *        1  = IN1 高 / IN2 低 时该轮"正转（前进方向）"；
 *        -1 = 该轮实际转向相反，用 -1 翻转，**不需要改硬件接线**。
 *
 * 什么时候需要改成 -1：
 *   装好麦轮后发现"发 $MV 前进，某个轮子却在倒转"，
 *   或者"发横移，车却斜着跑" —— 那就是该轮接线反了（或辊子装反），
 *   把它对应的这一项改成 -1 即可。
 */
static const int s_dir_pol[CHASSIS_MOTOR_COUNT] = { 1, 1, 1, 1 };

/* ==========================================================================
 * 二、运行状态
 * ========================================================================== */

/** 当前四个轮子的有符号速度（[-1,1]），供上报与调试 */
static volatile float s_wheel[CHASSIS_MOTOR_COUNT];

/** 当前归一化运动分量（上次 CHASSIS_Move 的入参），供上报与调试 */
static volatile float s_cmd_y  = 0.0f;
static volatile float s_cmd_x  = 0.0f;
static volatile float s_cmd_rx = 0.0f;

/** 速度上限百分比（0~100），$MC 可调 */
static volatile uint8_t s_speed_pct = CHASSIS_SPEED_PCT_DEF;
static volatile uint8_t s_mode = (uint8_t)CHASSIS_MODE_MANUAL;
static volatile uint32_t s_last_cmd_tick = 0u;

/** 运动计时：本次运动的**到期时刻**(ms，GetTick 时间戳)。0 表示不限时或已停 */
static volatile uint32_t s_deadline_ms = 0u;

/** 是否处于"连续运动"模式（$MV 的 tt=0），此时不自动停车 */
static volatile uint8_t s_continuous = 0u;

/* ==========================================================================
 * 三、底层：GPIO / PWM
 * ========================================================================== */

/**
 * @brief  初始化 8 个电机的方向控制引脚
 * @note   全部设为推挽输出并**先拉低**（两个 IN 都是低电平时 TB6612 处于
 *         刹车/停止状态），保证上电瞬间电机不转。
 */
static void Chassis_GpioDirInit(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    /* PA4 / PA5 / PA6 / PA7 = FL_IN1 / FL_IN2 / FR_IN1 / FR_IN2 */
    gpio.GPIO_Pin = CHASSIS_DIR_IN1_FL | CHASSIS_DIR_IN2_FL |
                    CHASSIS_DIR_IN1_FR | CHASSIS_DIR_IN2_FR;
    GPIO_Init(GPIOA, &gpio);

    /* PB14 / PB15 / PB12 / PB13 = RL_IN1 / RL_IN2 / RR_IN1 / RR_IN2
     * ★ RL 方向脚用 PB14/PB15（不要用 PB0/PB1，见 main.h 注释） */
    gpio.GPIO_Pin = CHASSIS_DIR_IN1_RL | CHASSIS_DIR_IN2_RL |
                    CHASSIS_DIR_IN1_RR | CHASSIS_DIR_IN2_RR;
    GPIO_Init(GPIOB, &gpio);

    /* 上电先全部拉低：TB6612 两个输入同为低 -> 输出关闭（滑行/刹车），电机不转 */
    GPIO_ResetBits(GPIOA, CHASSIS_DIR_IN1_FL | CHASSIS_DIR_IN2_FL |
                           CHASSIS_DIR_IN1_FR | CHASSIS_DIR_IN2_FR);
    GPIO_ResetBits(GPIOB, CHASSIS_DIR_IN1_RL | CHASSIS_DIR_IN2_RL |
                           CHASSIS_DIR_IN1_RR | CHASSIS_DIR_IN2_RR);
}

/**
 * @brief  初始化 4 路 PWM 输出引脚
 * @note   TIM4 在 C8T6(LQFP48) 上只有默认映射：
 *             CH1 = PB6   CH2 = PB7   CH3 = PB8   CH4 = PB9
 *         部分重映射会搬到 PD12~PD15，而 LQFP48 没有 PD 口，故不可用。
 */
static void Chassis_GpioPwmInit(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;      /* 复用推挽：定时器输出必须用这个 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;

    GPIO_Init(GPIOB, &gpio);
}

/**
 * @brief  初始化 TIM4 时基与 4 通道 PWM 输出
 * 定时器选择（都不能复用）：
 *           TIM3 被机械臂拿去做 1ms 插值基准（PSC=71, ARR=999）；
 *           本模块若复用 TIM3 会打断舵机插值，故独占 TIM4；
 *           但注意 APB1 上多个定时器共享同一条总线时钟（都是 72MHz），
 *           分频系数各自独立（本模块 PSC=72 -> 1MHz 计数），互不影响；
 *
 * PWM 频率 = 1MHz / 1000 = 1kHz，占空比经 CCR 调节（0 ~ CHASSIS_PWM_MAX）。
 */
static void Chassis_Tim4PwmInit(void)
{
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef oc;

    TIM_DeInit(CHASSIS_PWM_TIM);
    TIM_InternalClockConfig(CHASSIS_PWM_TIM);

    tb.TIM_Prescaler     = CHASSIS_PWM_PSC;      /* 72MHz / 72 = 1MHz，1 计数 = 1us */
    tb.TIM_Period        = CHASSIS_PWM_ARR;      /* 1MHz / 1000 = 1kHz PWM */
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(CHASSIS_PWM_TIM, &tb);

    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_Pulse       = 0u;

    TIM_OC1Init(CHASSIS_PWM_TIM, &oc);   /* CH1 -> PB6 -> FL */
    TIM_OC2Init(CHASSIS_PWM_TIM, &oc);   /* CH2 -> PB7 -> FR */
    TIM_OC3Init(CHASSIS_PWM_TIM, &oc);   /* CH3 -> PB8 -> RL */
    TIM_OC4Init(CHASSIS_PWM_TIM, &oc);   /* CH4 -> PB9 -> RR */

    /* 预装载：ARR/CCR 的更新只在更新事件发生，避免刷新瞬间出现中间态 */
    TIM_OC1PreloadConfig(CHASSIS_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(CHASSIS_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(CHASSIS_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(CHASSIS_PWM_TIM, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(CHASSIS_PWM_TIM, ENABLE);
    TIM_Cmd(CHASSIS_PWM_TIM, ENABLE);
}

/* ==========================================================================
 * 四、接口实现（声明见 chassis.h：一 初始化 / 二 运动控制 / 三 底层单元）
 * ========================================================================== */

void CHASSIS_Init(void)
{
    uint8_t i;

    for (i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
    {
        s_wheel[i] = 0.0f;
        CHASSIS_SetPwm(i, 0u);           /* 先断 PWM：上电默认全停 */
        CHASSIS_SetDir(i, 1u);           /* 方向脚归到"正转"位，但 PWM=0 不会转 */
    }

    s_deadline_ms = 0u;
    s_continuous = 0u;
    s_speed_pct  = CHASSIS_SPEED_PCT_DEF;

    Chassis_GpioDirInit();     /* ① 方向脚先拉低 */
    Chassis_GpioPwmInit();     /* ② PWM 引脚 */
    Chassis_Tim4PwmInit();     /* ③ 定时器与通道（CCR 初值 0） */

    /* ④ 再确认一次四个比较值为 0 —— 双保险，上电绝对不窜车 */
    TIM_SetCompare1(CHASSIS_PWM_TIM, 0u);
    TIM_SetCompare2(CHASSIS_PWM_TIM, 0u);
    TIM_SetCompare3(CHASSIS_PWM_TIM, 0u);
    TIM_SetCompare4(CHASSIS_PWM_TIM, 0u);
}

void CHASSIS_SetDir(uint8_t w, uint8_t fwd)
{
    int in1, in2;

    if (w >= CHASSIS_MOTOR_COUNT)
    {
        return;
    }

    /* 由极性表决定实际电平：pol>0 时 fwd=1 -> IN1 高 */
    if (s_dir_pol[w] > 0)
    {
        in1 = (fwd != 0u) ? 1 : 0;
    }
    else
    {
        in1 = (fwd != 0u) ? 0 : 1;
    }
    in2 = 1 - in1;

    if (in1 != 0) { GPIO_SetBits(s_dir_port[w],   s_dir_in1[w]); }
    else          { GPIO_ResetBits(s_dir_port[w], s_dir_in1[w]); }

    if (in2 != 0) { GPIO_SetBits(s_dir_port[w],   s_dir_in2[w]); }
    else          { GPIO_ResetBits(s_dir_port[w], s_dir_in2[w]); }
}

void CHASSIS_SetPwm(uint8_t w, uint16_t v)
{
    if (w >= CHASSIS_MOTOR_COUNT)
    {
        return;
    }
    if (v > CHASSIS_PWM_MAX)
    {
        v = CHASSIS_PWM_MAX;
    }

    switch (w)
    {
        case 0u: TIM_SetCompare1(CHASSIS_PWM_TIM, v); break;  /* FL */
        case 1u: TIM_SetCompare2(CHASSIS_PWM_TIM, v); break;  /* FR */
        case 2u: TIM_SetCompare3(CHASSIS_PWM_TIM, v); break;  /* RL */
        default: TIM_SetCompare4(CHASSIS_PWM_TIM, v); break;  /* RR */
    }
}

void CHASSIS_SetWheel(uint8_t w, float spd)
{
    float    s;
    float    mag;
    uint16_t pwm;

    if (w >= CHASSIS_MOTOR_COUNT)
    {
        return;
    }

    /* 先按当前速度上限缩放，再限幅 —— 顺序不能反，
     * 否则"降低速度上限"对已经发出的运动不生效。 */
    s = spd * ((float)s_speed_pct / 100.0f);
    if (s >  1.0f) { s =  1.0f; }
    if (s < -1.0f) { s = -1.0f; }

    s_wheel[w] = s;

    if (s < 0.0f)
    {
        CHASSIS_SetDir(w, 0u);
        mag = -s;
    }
    else
    {
        CHASSIS_SetDir(w, 1u);
        mag = s;
    }

    pwm = (uint16_t)(mag * (float)CHASSIS_PWM_MAX + 0.5f);
    CHASSIS_SetPwm(w, pwm);
}

void CHASSIS_Move(float y, float x, float rx, uint32_t ms)
{
    /* ---------------------------------------------------------------
     * ★ 安全互斥：机械臂正在动作时，底盘一律不许动。
     *   这条检查放在最前面 —— 宁可拒绝一次指令，也不能带臂撞货架。
     *   （PC 端也会先等机械臂 DONE 再发底盘指令，这里是下位机的最后防线。）
     * --------------------------------------------------------------- */
    if (SERVO_IsBusy() != 0u)
    {
        CHASSIS_Stop();
        return;
    }

    /* 限幅三个速度分量 */
    if (y  >  1.0f) { y  =  1.0f; }
    if (y  < -1.0f) { y  = -1.0f; }
    if (x  >  1.0f) { x  =  1.0f; }
    if (x  < -1.0f) { x  = -1.0f; }
    if (rx >  1.0f) { rx =  1.0f; }
    if (rx < -1.0f) { rx = -1.0f; }

    s_cmd_y  = y;
    s_cmd_x  = x;
    s_cmd_rx = rx;
    s_last_cmd_tick = GetTick();   /* 失联自停节拍 */

    /* ---- 麦轮逆解（标准 X 型装法，公式不可随意改）---- */
    CHASSIS_SetWheel(0u, y + x + rx);   /* FL */
    CHASSIS_SetWheel(1u, y - x - rx);   /* FR */
    CHASSIS_SetWheel(2u, y - x + rx);   /* RL */
    CHASSIS_SetWheel(3u, y + x - rx);   /* RR */

    /* ---- 运动计时（基于 SysTick 的绝对到期时刻，不受主循环快慢影响）---- */
    if (ms == 0u)
    {
        s_continuous  = 1u;
        s_deadline_ms = 0u;
    }
    else
    {
        if (ms > CHASSIS_MAX_MOVE_MS)
        {
            ms = CHASSIS_MAX_MOVE_MS;
        }
        s_continuous  = 0u;
        s_deadline_ms = GetTick() + ms;
    }
}

void CHASSIS_Stop(void)
{
    uint8_t i;

    for (i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
    {
        s_wheel[i] = 0.0f;
        CHASSIS_SetPwm(i, 0u);          /* 先断 PWM */
        CHASSIS_SetDir(i, 1u);          /* 方向脚归到"正转"位，但 PWM=0 不会转 */
    }

    s_cmd_y      = 0.0f;
    s_cmd_x      = 0.0f;
    s_cmd_rx     = 0.0f;
    s_deadline_ms = 0u;
    s_continuous = 0u;
    s_last_cmd_tick = GetTick();
}

void CHASSIS_SetMode(ChassisMode_t mode)
{
    s_mode = (uint8_t)mode;
    CHASSIS_Stop();
}

ChassisMode_t CHASSIS_GetMode(void)
{
    return (s_mode != 0u) ? CHASSIS_MODE_AUTO : CHASSIS_MODE_MANUAL;
}

void CHASSIS_SetSpeedPct(uint8_t pct)
{
    if (pct > 100u)
    {
        pct = 100u;
    }
    s_speed_pct = pct;

    /* 立即按新上限重算四轮输出：调速对"正在走的车"要马上生效。
     * 注意重算时要沿用"剩余的"时间，否则改一次速度就把总时长重置了。 */
    if (s_continuous != 0u)
    {
        CHASSIS_Move(s_cmd_y, s_cmd_x, s_cmd_rx, 0u);
    }
    else if (s_deadline_ms != 0u)
    {
        uint32_t now  = GetTick();
        uint32_t left = (s_deadline_ms > now) ? (s_deadline_ms - now) : 1u;
        CHASSIS_Move(s_cmd_y, s_cmd_x, s_cmd_rx, left);
    }
}

uint8_t CHASSIS_GetSpeedPct(void)
{
    return s_speed_pct;
}

uint8_t CHASSIS_IsMoving(void)
{
    uint8_t i;

    for (i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
    {
        if (s_wheel[i] > 0.001f || s_wheel[i] < -0.001f)
        {
            return 1u;
        }
    }
    return 0u;
}

void CHASSIS_GetWheelSpeeds(float *out)
{
    uint8_t i;

    if (out == 0)
    {
        return;
    }
    for (i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
    {
        out[i] = s_wheel[i];
    }
}

void CHASSIS_GetMotion(float *y, float *x, float *rx)
{
    if (y  != 0) { *y  = s_cmd_y;  }
    if (x  != 0) { *x  = s_cmd_x;  }
    if (rx != 0) { *rx = s_cmd_rx; }
}

void CHASSIS_Update(void)
{
    /* ---- 失联自停（融合自文件夹 1）----
     * 只作用于「连续运动」(tt=0)：
     *   定时运动(tt>0) 由下面的 deadline 自停，deadman 不得提前掐断。 */
    if (s_continuous != 0u)
    {
        if (CHASSIS_IsMoving() != 0u)
        {
            uint32_t limit = (s_mode != 0u)
                                 ? (uint32_t)CHASSIS_DEADMAN_AUTO_MS
                                 : (uint32_t)CHASSIS_DEADMAN_MANUAL_MS;
            if ((uint32_t)(GetTick() - s_last_cmd_tick) >= limit)
            {
                CHASSIS_Stop();
            }
        }
        return;
    }

    if (s_deadline_ms == 0u)
    {
        return;
    }

    /* 用 GetTick()（SysTick 驱动，见 my_lib/delay.c）做绝对时刻比对：
     * 只要主循环在本函数两次调用之间没阻塞超过 1ms，计时就是准的。
     * ★ 这里绝不能用 Delay() 之类阻塞等待 —— 会卡住串口收发。
     *
     * 有符号比较是必要的：GetTick() 回绕时 unsigned 直接比较会瞬间"到期"。
     * 转成 int32_t 后，(int32_t)(now - deadline) >= 0 在回绕时同样成立。 */
    if ((int32_t)(GetTick() - s_deadline_ms) >= 0)
    {
        CHASSIS_Stop();
    }
}
