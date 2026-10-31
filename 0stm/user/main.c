/**
  ******************************************************************************
  * @file    main.c
  * @brief   智能药箱 STM32 下位机主程序（Keil SPL 移植版）
  *
  * 功能定位：
  *   本固件只负责"接收角度指令 -> 驱动 4 路舵机"，不做视觉、不做逆解、不做决策。
  *
  * 数据流：
  *   PC --USART1--> [DMA 环形缓冲(USE_RX_DMA=1) | RXNE 中断(USE_RX_DMA=0)]
  *      --> 软件字节 FIFO --> (主循环) --> 协议状态机 --> 指令处理
  *      --> 舵机插值(TIM3 1ms) --> TIM2 PWM（四路舵机）
  *
  * 中断资源（SPL 版）：
  *   USART1 全局中断 : IDLE 空闲中断（USE_RX_DMA=1，判定一帧接收结束）
  *                     或 RXNE 接收中断（USE_RX_DMA=0，逐字节入 FIFO）
  *   DMA1_Channel5    : USART1 RX 循环搬运（仅 USE_RX_DMA=1）
  *   TIM3 更新中断    : 1ms 时基，推进舵机插值
  *   SysTick          : 时基（my_lib/delay.c，GetTick / Delay）
  *
  * 移植说明（HAL -> SPL）：
  *   - HAL_Init()               -> NVIC_PriorityGroupConfig + Delay_Init()
  *   - HAL_GetTick()/HAL_Delay() -> GetTick()/Delay()（my_lib/delay.c）
  *   - HAL_UART / HAL_DMA / HAL_TIM 句柄与回调 -> SPL 外设函数 + 寄存器
  *   - SystemClock_Config()     -> 依赖启动文件 SystemInit()（HSE 8M*PLL9=72M）
  *   - 新增 USE_RX_DMA 编译开关：1=DMA+IDLE（原版方案，真机）；0=RXNE 直收
  *     （Keil 仿真器最可靠，默认值），协议解析逻辑完全一致。
  *
  * @author  智能药箱毕业设计（Keil SPL 移植版）
  * @version V1.0
  ******************************************************************************
  */

#include "main.h"
#include "protocol.h"
#include "servo.h"
#include "chassis.h"  /* ★ 麦轮底盘：CHASSIS_* 系列函数。
                       *   漏了它会让所有调用变成"隐式声明"，编译器按
                       *   int f() 处理 -> 返回值/浮点参数都会被错误传递，
                       *   是必须避免的（编译告警 #223-D 就是它在提醒）。 */
#include "delay.h"
#include "usart.h"
#include "string.h"   /* NULL 宏定义（protocol.h 中 NULL 校验用到） */

/* ==========================================================================
 * 文件内静态变量
 * ========================================================================== */

#if USE_RX_DMA
/** DMA 循环接收缓冲区。
 *  DMA 会一直往这里搬数据，搬满自动回到开头，因此是一个"硬件环形缓冲"。 */
static uint8_t g_rx_dma_buf[RX_DMA_BUF_SIZE];

/** 上一次已经取走的 DMA 写位置（0 ~ RX_DMA_BUF_SIZE-1）。
 *  由 IDLE 处理函数维护，用于计算"新增了多少字节"。
 *  加 volatile：它在中断上下文与主循环上下文中都会被访问。 */
static volatile uint16_t g_rx_dma_last_pos;
#endif /* USE_RX_DMA */

/** 软件字节 FIFO：中断里只做"搬运"，解析全部放在主循环，缩短中断占用时间 */
typedef struct
{
    volatile uint16_t head;             /**< 写索引（中断里推进） */
    volatile uint16_t tail;             /**< 读索引（主循环里推进） */
    uint8_t           buf[RX_FIFO_SIZE];/**< 数据区，大小为 2 的幂 */
} ByteFifo_t;

static ByteFifo_t g_rx_fifo;

/* ---- HC05 当前选中的舵机（1~4 -> 内部 0~3），'+'/'-' 作用在这只上 ---- */
static uint8_t g_servo_sel;

/* ---- HC05 蓝牙（USART3）接收 FIFO ----
 * 与 PC 的 g_rx_fifo 完全独立：手机蓝牙和 PC 可以同时在线互不干扰。
 * 手机发的命令很短（单字符 F/B/L/R 等），FIFO 小一点够用。 */
typedef struct
{
    volatile uint16_t head;
    volatile uint16_t tail;
    uint8_t           buf[HC05_FIFO_SIZE];
} Hc05Fifo_t;

static Hc05Fifo_t g_hc05_fifo;

/** PC 上一次下发底盘运动的时间戳：用于让"手机命令"不与 PC 抢方向盘。 */
static uint32_t g_last_pc_move_tick;

/** 是否已经发送过 $READY（PC 收到它之后才会开始下发业务指令） */
static uint8_t  g_ready_sent;

/** 心跳计数 */
static uint16_t g_hb_cnt;

/** 上一次发送心跳的 GetTick() 值 */
static uint32_t g_last_hb_tick;

/** 上一次闪烁 LED 的 GetTick() 值 */
static uint32_t g_last_led_tick;

/* ==========================================================================
 * 内部函数前置声明
 * ========================================================================== */

static void MX_GPIO_Init(void);
#if USE_RX_DMA
static void MX_DMA_Init(void);
#endif
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
#if USE_IWDG
static void MX_IWDG_Init(void);
#endif

static void     Fifo_Push(ByteFifo_t *f, uint8_t b);
static uint8_t  Fifo_Pop(ByteFifo_t *f, uint8_t *b);
#if USE_RX_DMA
static void     Uart_ClearIdleFlag(void);
#endif
void            GPIO_ToggleLed(void);   /* 定义在本文件后半部，供 main/Error_Handler 使用 */
static void     Protocol_Handle(const ProtoFrame_t *frame);
static void     HandleCmd_SA(const ProtoFrame_t *frame);
static void     HandleCmd_SS(const ProtoFrame_t *frame);
static void     HandleCmd_GC(const ProtoFrame_t *frame);
static void     HandleCmd_RUN(const ProtoFrame_t *frame);
static void     HandleCmd_HOM(const ProtoFrame_t *frame);
static void     HandleCmd_QRY(const ProtoFrame_t *frame);
static void     HandleCmd_EST(const ProtoFrame_t *frame);
static void     HandleCmd_CFG(const ProtoFrame_t *frame);
static void     HandleCmd_MD(const ProtoFrame_t *frame);
static void     HandleCmd_MV(const ProtoFrame_t *frame);
static void     HandleCmd_MS(const ProtoFrame_t *frame);
static void     HandleCmd_MC(const ProtoFrame_t *frame);
static void     HandleCmd_MQ(const ProtoFrame_t *frame);
static void     HC05_ProcessInput(void);
static void     HC05_HandleChar(uint8_t c);
static void     HC05_SelectServo(uint8_t id);
static void     HC05_MoveServoTo(uint8_t id, float ang);
static void     HC05_ServoUnit(uint8_t id, int8_t direction);
static void     HC05_Gripper(uint8_t is_open);
static void     HC05_GoHome(void);

/* ==========================================================================
 * main
 * ========================================================================== */

/**
 * @brief  程序入口
 * @note   初始化顺序说明：
 *           1) 中断优先级分组 + 时基初始化（等价原 HAL_Init/SystemClock_Config）；
 *           2) GPIO / DMA / USART1 / TIM2(舵机PWM) / TIM3：外设初始化；
 *           3) 启动串口接收（先于 SERVO_Init，这样即使归位过程中 PC 发来
 *              指令也不会丢）；
 *           4) 启动 TIM3 的 1ms 中断（插值时基）；
 *           5) SERVO_Init：装载配置 -> 写 CCR -> 开 PWM -> 平滑归位；
 *           6) 进入主循环：解析指令、推进事件、心跳、喂狗。
 * @param  无
 * @retval int（不会返回）
 */
int main(void)
{
    /* 4 位抢占优先级分组（与原 HAL_MspInit 一致） */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    /* SysTick 1ms 时基（my_lib/delay.c，GetTick()/Delay() 依赖它） */
    Delay_Init();

    SystemClock_Config();

    MX_GPIO_Init();
#if USE_RX_DMA
    MX_DMA_Init();
#endif
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();      /* ★ HC05 蓝牙（底盘手动遥控 / auto 模式备用通道） */
    MX_TIM2_Init();
    MX_TIM3_Init();
#if USE_IWDG
    MX_IWDG_Init();
#endif

    PROTO_Init();

    /* ---- 底盘初始化（TIM4 四路 PWM + 8 个方向脚，上电全停） ---- */
    CHASSIS_Init();
    g_hc05_fifo.head = 0u;
    g_hc05_fifo.tail = 0u;

    /* ---- 启动串口接收 ---- */
    g_rx_fifo.head = 0u;
    g_rx_fifo.tail = 0u;
#if USE_RX_DMA
    g_rx_dma_last_pos = 0u;
    /* DMA 循环搬运 + IDLE 空闲中断 */
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
#else
    /* RXNE 中断逐字节直收（Keil 仿真调试默认路径，真机同样可用） */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
#endif

    /* ---- 启动 1ms 插值时基（TIM3） ---- */
    TIM_Cmd(TIM3, ENABLE);

    /* ---- 舵机初始化 + 平滑归位（归位结束由主循环发 $READY） ---- */
    g_ready_sent    = 0u;
    g_hb_cnt        = 0u;
    g_last_hb_tick  = GetTick();
    g_last_led_tick = GetTick();

    SERVO_Init();

    /* ================= 主循环 ================= */
    while (1)
    {
        uint8_t      b;
        ProtoFrame_t frame;
        uint8_t      res;
        char         done_cmd[8];
        uint32_t     now;

        /* ---- 1. 兜底排空接收缓冲 ----
         * 正常情况下由 IDLE 中断（USE_RX_DMA=1）搬运；这里再调用一次是
         * 防止极端情况下 IDLE 事件丢失导致 DMA 覆盖未取走的数据。
         * 函数内部做了 cur == last 的判断，无新数据时立即返回，开销极小。
         * USE_RX_DMA=0 时本函数为空操作。 */
        USER_UART_IdleHandler();

        /* ---- 2. 逐字节喂给协议状态机 ---- */
        while (Fifo_Pop(&g_rx_fifo, &b) != 0u)
        {
            res = PROTO_FeedByte(b, &frame);
            if (res == PROTO_FEED_OK)
            {
                Protocol_Handle(&frame);
            }
            else if (res == PROTO_FEED_CS_ERR)
            {
                /* 校验失败：整帧丢弃，绝不"容错执行" */
                PROTO_SendErr((uint8_t)PROTO_ERR_CS, "CS");
            }
            else if (res == PROTO_FEED_FMT_ERR)
            {
                PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "FMT");
            }
            else
            {
                /* 继续接收 */
            }
        }

        /* ---- 3. 运动完成事件 -> $DONE ---- */
        if (SERVO_TakeDoneEvent(done_cmd, (uint8_t)sizeof(done_cmd)) != 0u)
        {
            if (g_ready_sent == 0u)
            {
                /* 上电归位运动完成，此时才允许 PC 开始工作 */
                PROTO_SendReady();
                g_ready_sent = 1u;
            }
            else
            {
                PROTO_SendDone(done_cmd);
            }
        }

        /* ---- 4. READY 兜底：若归位异常地没有产生 DONE，也保证 PC 不会被卡死 ---- */
        if ((g_ready_sent == 0u) && (SERVO_IsBusy() == 0u))
        {
            PROTO_SendReady();
            g_ready_sent = 1u;
        }

        /* ---- 5. 超时 / 堵转检测 ---- */
        SERVO_CheckTimeout();

        /* ---- 5.1 底盘：处理 HC05(USART3) 收到的手机蓝牙命令 ----
         * manual_phone 模式下，人用手机连 HC05 手动推杆控车；
         * auto 模式下 HC05 一般没人发命令，这段是空转（FIFO 为空立即返回）。
         * ★ 手机命令【优先级低于 PC】：PC 正在下发底盘运动时，
         *   手机命令被忽略，避免两方抢方向盘。 */
        HC05_ProcessInput();

        /* ---- 5.2 底盘：运动计时到期自动停车 ---- */
        CHASSIS_Update();

        /* ---- 6. 1Hz 心跳 + LED 指示 ---- */
        now = GetTick();
        if ((uint32_t)(now - g_last_hb_tick) >= (uint32_t)HB_PERIOD_MS)
        {
            g_last_hb_tick = now;
            g_hb_cnt++;
            PROTO_SendHb(g_hb_cnt);
        }
        if ((uint32_t)(now - g_last_led_tick) >= 500u)
        {
            g_last_led_tick = now;
            GPIO_ToggleLed();
        }

        /* ---- 7. 喂狗 ---- */
#if USE_IWDG
        IWDG_ReloadCounter();
#endif
    }
}

/* ==========================================================================
 * 一、串口发送与接收
 * ========================================================================== */

/**
 * @brief  阻塞式串口发送（protocol.c 上报帧统一走这里）
 * @note   复用 my_lib/usart.c 的 My_USART_SendBytes：
 *         USART1 已由 MX_USART1_UART_Init() 初始化，
 *         my_lib 的 usart_init() 不可重复调用，因此只借用其"发送"函数。
 *         发送期间 RX 仍由 DMA/RXNE 中断后台填充 FIFO，不会丢数据。
 * @param  data 数据首地址
 * @param  len  数据长度
 * @retval 无
 */
void App_Uart_Send(const uint8_t *data, uint16_t len)
{
    My_USART_SendBytes(USART1, data, len);
}

/**
 * @brief  覆盖 my_lib/usart.c 中的 __weak My_USART_SendBytes —— 给 TXE/TC 轮询加超时
 * @note   **为什么必须覆盖**：my_lib 原版是
 *             while(TXE == RESET);  ...  while(TC == RESET);
 *         两个循环都没有退出条件。而本函数由 App_Uart_Send 在**主循环**里调用
 *         （$READY / $ACK / $DONE / $ERR / 1Hz $HB）。一旦 TX 引脚被拉低、对端短接
 *         或 USART 外设异常，TC 永不复位 -> 卡死在 while 里：
 *           - USE_IWDG=1 时：主循环无法喂狗 -> 约 2s 后看门狗复位（表现为"反复重启"）
 *           - USE_IWDG=0 时（当前默认）：**整机直接死机**，机械臂保持在最后位置，
 *             且 PC 端因为收不到 $HB 只能超时
 *         加超时后最坏情况只是"这一帧发不出去"，主循环继续跑，能正常报错/承接急停。
 *
 *         **超时用计数而不是 GetTick()**：GetTick() 依赖 SysTick 中断，
 *         本函数要能在"SysTick 也异常"的极端情况下退出，所以不能反向依赖它。
 *         按 72MHz 主频估算，1 亿次空循环远超任何一帧的发送时间（最长帧约 3ms
 *         @115200），实际正常发送时循环计数只有几百次，**不产生额外开销**。
 *
 * @param  USARTx 目标串口
 * @param  pData  数据首地址
 * @param  Size   数据长度
 * @retval 无（失败时静默丢弃该帧，由上层超时机制兜底）
 */
void My_USART_SendBytes(USART_TypeDef *USARTx, const uint8_t *pData, uint16_t Size)
{
    /* 每个字节的 TXE 等待上限；整帧的 TC 等待上限。
     * 取值远大于实际需要（一个字节 @115200 约 87us，72MHz 下约 6000 次循环），
     * 只用于"硬件挂了"时兜底退出。 */
    #define UART_TXE_WAIT_LIMIT   2000000u
    #define UART_TC_WAIT_LIMIT    2000000u

    uint16_t i;
    uint32_t guard;

    if ((pData == NULL) || (Size == 0u))
    {
        return;
    }

    for (i = 0u; i < Size; i++)
    {
        guard = UART_TXE_WAIT_LIMIT;
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
        {
            if (--guard == 0u)
            {
                return;   /* 发送通道异常，放弃本帧，避免卡死主循环 */
            }
        }
        USART_SendData(USARTx, pData[i]);
    }

    guard = UART_TC_WAIT_LIMIT;
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
    {
        if (--guard == 0u)
        {
            return;
        }
    }

    #undef UART_TXE_WAIT_LIMIT
    #undef UART_TC_WAIT_LIMIT
}


#if USE_RX_DMA
/**
 * @brief  清 USART1 的 IDLE 中断标志
 * @note   **F1 的关键坑**：IDLE 标志不能像其它标志那样用赋值 0 的方式清除，
 *         必须"先读 SR、再读 DR"这一固定序列才能清除。
 *         由于 RX 由 DMA 搬运，DR 中的数据已被 DMA 取走，此处读 DR 不会
 *         丢掉有效数据（这是 STM32 社区的标准做法）。
 * @param  无
 * @retval 无
 */
static void Uart_ClearIdleFlag(void)
{
    volatile uint32_t tmp_sr;
    volatile uint32_t tmp_dr;

    tmp_sr = USART1->SR;   /* 第 1 步：读 SR */
    tmp_dr = USART1->DR;   /* 第 2 步：读 DR —— 两步合起来才清除 IDLE */

    (void)tmp_sr;
    (void)tmp_dr;
}
#endif /* USE_RX_DMA */

/**
 * @brief  USART1 IDLE 中断的"完整处理"入口（USE_RX_DMA=1 时由
 *         stm32f10x_it.c 中的 USART1_IRQHandler 调用）
 * @note   做两件事：
 *           1) 按 F1 的固定序列清除 IDLE 标志（先读 SR 再读 DR）；
 *           2) 调用 USER_UART_IdleHandler() 搬运新增字节。
 *         把它放在 main.c 而不是中断文件里，是为了让"清标志"这段
 *         与硬件强相关的代码和"搬运缓冲"的代码紧挨在一起，便于维护。
 * @param  无
 * @retval 无
 */
void USER_UART_IdleIrqHandler(void)
{
#if USE_RX_DMA
    Uart_ClearIdleFlag();
    USER_UART_IdleHandler();
#else
    /* USE_RX_DMA=0 时本函数不应被调用；保留以防误触发 */
#endif
}

/**
 * @brief  USART1 IDLE 事件处理：把 DMA 环形缓冲中的新增字节搬进软件 FIFO
 * @note   DMA 写位置的计算：
 *             write_pos = RX_DMA_BUF_SIZE - CNDTR
 *         CNDTR 是"剩余待搬运字节数"，随接收递减、回绕时自动重载。
 *         与上次位置 last 比较，分两种情况（**回绕处理**）：
 *             cur > last  : 数据连续，取 [last, cur)
 *             cur < last  : 发生过回绕，先取 [last, SIZE)，再取 [0, cur)
 *             cur == last : 无新数据，直接返回
 *         USE_RX_DMA=0 时本函数是空操作（数据由 RXNE 中断直接入 FIFO）。
 * @param  无
 * @retval 无
 */
void USER_UART_IdleHandler(void)
{
#if USE_RX_DMA
    uint16_t cur;
    uint16_t last;
    uint16_t i;

    /* 读取 DMA 当前写位置（CNDTR = 剩余计数） */
    cur  = (uint16_t)(RX_DMA_BUF_SIZE -
                      (DMA_GetCurrDataCounter(DMA1_Channel5) & 0xFFFFu));
    last = (uint16_t)g_rx_dma_last_pos;

    if (cur == last)
    {
        return;   /* 没有新数据 */
    }

    if (cur > last)
    {
        for (i = last; i < cur; i++)
        {
            Fifo_Push(&g_rx_fifo, g_rx_dma_buf[i]);
        }
    }
    else
    {
        /* DMA 指针已回绕：先取尾段，再取头段 */
        for (i = last; i < (uint16_t)RX_DMA_BUF_SIZE; i++)
        {
            Fifo_Push(&g_rx_fifo, g_rx_dma_buf[i]);
        }
        for (i = 0u; i < cur; i++)
        {
            Fifo_Push(&g_rx_fifo, g_rx_dma_buf[i]);
        }
    }

    g_rx_dma_last_pos = cur;
#else
    /* 空操作：RXNE 路径下数据已在中断里入 FIFO */
#endif
}

/**
 * @brief  把一字节压入接收 FIFO（USE_RX_DMA=0 时由 RXNE 中断调用）
 * @param  b 接收到的字节
 * @retval 无
 */
void UART_RxBytePush(uint8_t b)
{
    Fifo_Push(&g_rx_fifo, b);
}

/**
 * @brief  串口错误恢复（ORE/FE/NE 错误标志置位时调用）
 * @note   在 DMA 循环接收模式下，错误不会停止 DMA。
 *         恢复动作（**刻意保守**）：
 *           1) 清掉错误标志（读 SR + 读 DR 的固定序列）；
 *           2) 重新使能 IDLE 中断；
 *           3) 只复位"协议解析状态机"，**不动接收 FIFO**。
 *
 *         **为什么不再清空 FIFO**：原实现执行
 *             g_rx_fifo.head = 0; g_rx_fifo.tail = 0;
 *         线上一个电气噪声字节就会置位 ORE，从而把 FIFO 里**已经收完、
 *         只等主循环解析的完整合法帧全部丢掉**。而 USART1_IRQHandler 是在
 *         处理完 RXNE 之后无条件检查错误标志，这条路径相当容易被触发 ——
 *         表现为"偶发丢指令，PC 端报等待 ACK 超时"，且很难复现定位。
 *
 *         代价：出错的那个字节仍留在 FIFO 里，可能让**当前这一帧**解析失败。
 *         但 PROTO_FeedByte 本身对畸形字节有容错（重新同步 '$'、校验和拦截），
 *         最坏只是这一帧被丢弃，**比丢掉后续所有好帧划算得多**。
 * @param  无
 * @retval 无
 */
void UART_RxErrorRecover(void)
{
    volatile uint32_t tmp_sr = USART1->SR;
    volatile uint32_t tmp_dr = USART1->DR;

    (void)tmp_sr;
    (void)tmp_dr;

#if USE_RX_DMA
    /* 让 DMA 读指针与当前位置对齐（历史数据在 DMA 缓冲里，不影响软件 FIFO） */
    g_rx_dma_last_pos = (uint16_t)(RX_DMA_BUF_SIZE -
                                   (DMA_GetCurrDataCounter(DMA1_Channel5) & 0xFFFFu));
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
#endif

    /* 只复位协议解析状态机：丢弃"半截帧"，从下一个 '$' 重新开始。
     * 注意 PROTO_Init() 只清行缓冲与同步标志，不碰 FIFO。 */
    PROTO_Init();
}

/**
 * @brief  字节 FIFO 入队
 * @note   索引回绕用 "& (SIZE-1)"（要求 SIZE 是 2 的幂），
 *         避免在中断里做除法。缓冲满时丢弃新字节（宁可丢一帧，
 *         也不能破坏 FIFO 结构）。
 * @param  f FIFO
 * @param  b 待入队字节
 * @retval 无
 */
static void Fifo_Push(ByteFifo_t *f, uint8_t b)
{
    uint16_t next;

    next = (uint16_t)((f->head + 1u) & (uint16_t)RX_FIFO_MASK);
    if (next == (uint16_t)f->tail)
    {
        return;   /* 满：丢弃 */
    }
    f->buf[f->head] = b;
    f->head = next;
}

/**
 * @brief  字节 FIFO 出队
 * @param  f FIFO
 * @param  b 输出字节
 * @retval 1 取到；0 空
 */
static uint8_t Fifo_Pop(ByteFifo_t *f, uint8_t *b)
{
    if (f->head == f->tail)
    {
        return 0u;
    }
    *b = f->buf[f->tail];
    f->tail = (uint16_t)((f->tail + 1u) & (uint16_t)RX_FIFO_MASK);
    return 1u;
}

/* ==========================================================================
 * 二、指令分发与处理
 * ========================================================================== */

/**
 * @brief  指令分发
 * @note   流程：解析指令名 -> 未识别则报 $ERR,02 -> 否则进入对应处理函数。
 * @param  frame 已解析的帧
 * @retval 无
 */
static void Protocol_Handle(const ProtoFrame_t *frame)
{
    if (frame == NULL)
    {
        return;
    }

    switch (frame->cmd)
    {
        case PROTO_CMD_SA:  HandleCmd_SA(frame);  break;
        case PROTO_CMD_SS:  HandleCmd_SS(frame);  break;
        case PROTO_CMD_GC:  HandleCmd_GC(frame);  break;
        case PROTO_CMD_RUN: HandleCmd_RUN(frame); break;
        case PROTO_CMD_HOM: HandleCmd_HOM(frame); break;
        case PROTO_CMD_QRY: HandleCmd_QRY(frame); break;
        case PROTO_CMD_EST: HandleCmd_EST(frame); break;
        case PROTO_CMD_CFG: HandleCmd_CFG(frame); break;

        /* ---- 麦轮底盘 ---- */
        case PROTO_CMD_MD:  HandleCmd_MD(frame);  break;
        case PROTO_CMD_MV:  HandleCmd_MV(frame);  break;
        case PROTO_CMD_MS:  HandleCmd_MS(frame);  break;
        case PROTO_CMD_MC:  HandleCmd_MC(frame);  break;
        case PROTO_CMD_MQ:  HandleCmd_MQ(frame);  break;
        default:
            PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "CMD");
            break;
    }
}

/**
 * @brief  $SA,a1,a2,a3,a4,tt —— 四路舵机同步插值运动
 * @note   处理顺序：取参数 -> 角度越限检查(any) -> 发 $ACK,SA -> 启动运动。
 *         注意"先全部检查再动作"，任一角度越限就整条指令拒绝，
 *         不会出现部分舵机动了、部分没动的半截状态。
 * @param  frame 已解析帧
 * @retval 无
 */
static void HandleCmd_SA(const ProtoFrame_t *frame)
{
    /* ★ 底盘运动中锁臂（融合自文件夹 1）：$ERR,06 */
    if (CHASSIS_IsMoving() != 0u)
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_LOCKED, "CHS");
        return;
    }
    char     arg[PROTO_ARG_MAX];
    float    ang[SERVO_COUNT];
    uint32_t ms;
    uint8_t  i;

    for (i = 0u; i < SERVO_COUNT; i++)
    {
        if (PROTO_GetArg(frame->payload, (uint8_t)(i + 1u), arg, (uint16_t)sizeof(arg)) == 0u)
        {
            PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ARG");
            return;
        }
        ang[i] = PROTO_ArgToFloat(arg);
    }

    if (PROTO_GetArg(frame->payload, 5u, arg, (uint16_t)sizeof(arg)) == 0u)
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ARG");
        return;
    }
    ms = (uint32_t)PROTO_ArgToInt(arg);

    /* 限位：STM32 侧最后一道防线 */
    for (i = 0u; i < SERVO_COUNT; i++)
    {
        if (SERVO_CheckLimit(i, ang[i]) == 0u)
        {
            PROTO_SendErr((uint8_t)PROTO_ERR_LIMIT, "LIM");
            return;
        }
    }

    PROTO_SendAck("SA");
    if (SERVO_StartSyncMove(ang, ms, "SA") == 0u)
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_LIMIT, "LIM");
    }
}

/**
 * @brief  $SS,id,ang,tt —— 单舵机运动（调试用）
 * @param  frame 已解析帧
 * @retval 无
 */
static void HandleCmd_SS(const ProtoFrame_t *frame)
{
    char     arg[PROTO_ARG_MAX];
    int32_t  id;
    float    ang;
    uint32_t ms;

    if (PROTO_GetArg(frame->payload, 1u, arg, (uint16_t)sizeof(arg)) == 0u) { goto arg_err; }
    id = PROTO_ArgToInt(arg);

    if (PROTO_GetArg(frame->payload, 2u, arg, (uint16_t)sizeof(arg)) == 0u) { goto arg_err; }
    ang = PROTO_ArgToFloat(arg);

    if (PROTO_GetArg(frame->payload, 3u, arg, (uint16_t)sizeof(arg)) == 0u) { goto arg_err; }
    ms = (uint32_t)PROTO_ArgToInt(arg);

    if ((id < 0) || (id >= (int32_t)SERVO_COUNT))
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ID");
        return;
    }
    if (SERVO_CheckLimit((uint8_t)id, ang) == 0u)
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_LIMIT, "LIM");
        return;
    }

    PROTO_SendAck("SS");
    if (SERVO_MoveSingle((uint8_t)id, ang, ms, "SS") == 0u)
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_LIMIT, "LIM");
    }
    return;

arg_err:
    PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ARG");
}

/**
 * @brief  $GC,open|close —— 夹爪开/合
 * @note   内部映射为 servo3 的两个预设角度（GRIPPER_OPEN_DEG / GRIPPER_CLOSE_DEG），
 *         Flash 中也存了一份。
 * @param  frame 已解析帧
 * @retval 无
 */
static void HandleCmd_GC(const ProtoFrame_t *frame)
{
    char    arg[PROTO_ARG_MAX];
    uint8_t is_open;

    if (PROTO_GetArg(frame->payload, 1u, arg, (uint16_t)sizeof(arg)) == 0u)
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ARG");
        return;
    }

    if (PROTO_StrCaseEq(arg, "OPEN") != 0u)
    {
        is_open = 1u;
    }
    else if (PROTO_StrCaseEq(arg, "CLOSE") != 0u)
    {
        is_open = 0u;
    }
    else
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ARG");
        return;
    }

    PROTO_SendAck("GC");
    if (SERVO_SetGripper(is_open, (uint32_t)DEFAULT_MOVE_MS, "GC") == 0u)
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_LIMIT, "LIM");
    }
}

/**
 * @brief  $RUN,slot_id —— 执行 Flash 中预存的动作序列
 * @note   装载前会对整条序列的所有角度做限位检查，任一点越限则整条不执行。
 *         序列执行完毕后才发一次 $DONE,RUN。
 * @param  frame 已解析帧
 * @retval 无
 */
static void HandleCmd_RUN(const ProtoFrame_t *frame)
{
    char    arg[PROTO_ARG_MAX];
    int32_t slot;

    if (PROTO_GetArg(frame->payload, 1u, arg, (uint16_t)sizeof(arg)) == 0u)
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ARG");
        return;
    }
    slot = PROTO_ArgToInt(arg);

    if ((slot < 1) || (slot > (int32_t)RUN_SLOT_COUNT))
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "SLOT");
        return;
    }

    PROTO_SendAck("RUN");
    if (SERVO_StartSequence((uint8_t)slot) == 0u)
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_LIMIT, "LIM");
    }
}

/**
 * @brief  $HOM —— 回 HOME 位
 * @param  frame 已解析帧（无参数）
 * @retval 无
 */
static void HandleCmd_HOM(const ProtoFrame_t *frame)
{
    (void)frame;

    PROTO_SendAck("HOM");
    if (SERVO_GoHome("HOM") == 0u)
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_LIMIT, "LIM");
    }
}

/**
 * @brief  $QRY,POSE —— 查询当前四路角度
 * @note   先回 $ACK,QRY 再回 $POSE,a1,a2,a3,a4（与协议"每条指令都有 ACK"一致）。
 *         参数缺省或不为 POSE 时也照样回复位姿，提高鲁棒性。
 * @param  frame 已解析帧
 * @retval 无
 */
static void HandleCmd_QRY(const ProtoFrame_t *frame)
{
    float pose[SERVO_COUNT];

    (void)frame;

    PROTO_SendAck("QRY");

    SERVO_GetPose(pose);
    PROTO_SendPose(pose);
}

/**
 * @brief  $EST —— 急停
 * @note   直接调用 SERVO_StopNow() 冻结目标角，响应在收到帧的同一主循环
 *         周期内完成（< 1ms），不会被正在进行的插值延迟。
 * @param  frame 已解析帧
 * @retval 无
 */
static void HandleCmd_EST(const ProtoFrame_t *frame)
{
    (void)frame;

    SERVO_StopNow();
    PROTO_SendAck("EST");
}

/**
 * @brief  $CFG,id,min,max —— 配置某舵机限位并写入 Flash
 * @note   校验：id 合法 -> min<max -> 落在硬件绝对限位内 -> 写入 Flash。
 *         失败时不改动任何已有的限位值。
 * @param  frame 已解析帧
 * @retval 无
 */
static void HandleCmd_CFG(const ProtoFrame_t *frame)
{
    char    arg[PROTO_ARG_MAX];
    int32_t id;
    float   min_v;
    float   max_v;

    if (PROTO_GetArg(frame->payload, 1u, arg, (uint16_t)sizeof(arg)) == 0u) { goto arg_err; }
    id = PROTO_ArgToInt(arg);

    if (PROTO_GetArg(frame->payload, 2u, arg, (uint16_t)sizeof(arg)) == 0u) { goto arg_err; }
    min_v = PROTO_ArgToFloat(arg);

    if (PROTO_GetArg(frame->payload, 3u, arg, (uint16_t)sizeof(arg)) == 0u) { goto arg_err; }
    max_v = PROTO_ArgToFloat(arg);

    if ((id < 0) || (id >= (int32_t)SERVO_COUNT))
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ID");
        return;
    }

    PROTO_SendAck("CFG");

    if (SERVO_SetLimits((uint8_t)id, min_v, max_v) == 0u)
    {
        /* 参数非法或 Flash 写入失败 */
        PROTO_SendErr((uint8_t)PROTO_ERR_LIMIT, "LIM");
    }
    return;

arg_err:
    PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ARG");
}

/* ==========================================================================
 * 二·B、麦轮底盘指令处理（$MD / $MV / $MS / $MC / $MQ）
 *
 * ★ 命名速记（融合版，不再用易记反的 MVS/MVC）：
 *     MD=Mode 模式   MV=Move 运动   MS=Stop 停车   MC=Cap 限速   MQ=Query 查询
 *
 * ★ 底盘与机械臂的运动互斥（安全要点）：
 *     - $MV 下发时，若机械臂正在运动（SERVO_IsBusy()），直接拒绝并回 $ERR,06。
 *       理由：底盘带着伸出去的机械臂移动，极易撞到货架。
 *     - CHASSIS_Move() 内部还会再查一次，形成双保险。
 *     - 正确顺序永远是：机械臂先回安全位(收到 DONE) -> 底盘再动。
 *     - 反向：底盘运动中臂指令也拒绝（$ERR,06，融合自文件夹 1）。
 * ========================================================================== */

/**
 * @brief  发送底盘状态 $CHS（MD/MV/MS/MC/MQ 共用）
 */
static void SendChassisState(void)
{
    float y, x, rx;
    CHASSIS_GetMotion(&y, &x, &rx);
    PROTO_SendChassis((uint8_t)((CHASSIS_GetMode() == CHASSIS_MODE_AUTO) ? 1u : 0u),
                      CHASSIS_IsMoving(),
                      y * 100.0f, x * 100.0f, rx * 100.0f,
                      CHASSIS_GetSpeedPct());
}

/**
 * @brief  $MD,AUTO|MAN|Q —— 工作模式（融合自文件夹 1）
 */
static void HandleCmd_MD(const ProtoFrame_t *frame)
{
    char arg[PROTO_ARG_MAX];

    if (PROTO_GetArg(frame->payload, 1u, arg, (uint16_t)sizeof(arg)) == 0u)
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ARG");
        return;
    }
    PROTO_SendAck("MD");
    if ((arg[0] == 'A') || (arg[0] == 'a'))
    {
        CHASSIS_SetMode(CHASSIS_MODE_AUTO);
    }
    else if ((arg[0] == 'M') || (arg[0] == 'm'))
    {
        CHASSIS_SetMode(CHASSIS_MODE_MANUAL);
    }
    SendChassisState();
}

/**
 * @brief  $MV,y,x,rx[,tt] —— 麦轮运动
 * @note   参数语义（见 chassis.h）：
 *           y  前后速度，+ 前进 / - 后退，[-1,1]
 *           x  横移速度，+ 右移 / - 左移，[-1,1]
 *           rx 自转速度，+ 顺时针 / - 逆时针，[-1,1]
 *           tt 持续时长(ms)，可省略；0/省略 = 连续直到 $MS 或失联自停
 *         ★ 固件只回 $ACK + $CHS，**不回 $DONE**（PC 别死等）。
 */
static void HandleCmd_MV(const ProtoFrame_t *frame)
{
    char    arg[PROTO_ARG_MAX];
    float   y, x, rx;
    int32_t ms = 0;

    if (PROTO_GetArg(frame->payload, 1u, arg, (uint16_t)sizeof(arg)) == 0u) { goto arg_err; }
    y = PROTO_ArgToFloat(arg);

    if (PROTO_GetArg(frame->payload, 2u, arg, (uint16_t)sizeof(arg)) == 0u) { goto arg_err; }
    x = PROTO_ArgToFloat(arg);

    if (PROTO_GetArg(frame->payload, 3u, arg, (uint16_t)sizeof(arg)) == 0u) { goto arg_err; }
    rx = PROTO_ArgToFloat(arg);

    /* tt 可选 */
    if (PROTO_GetArg(frame->payload, 4u, arg, (uint16_t)sizeof(arg)) != 0u)
    {
        ms = PROTO_ArgToInt(arg);
        if (ms < 0) { ms = 0; }
    }

    /* ★ 机械臂正在动 -> 拒绝底盘运动请求（下位机侧的最后防线） */
    if (SERVO_IsBusy() != 0u)
    {
        PROTO_SendAck("MV");
        PROTO_SendErr((uint8_t)PROTO_ERR_LOCKED, "ARMBUSY");
        return;
    }

    PROTO_SendAck("MV");

    g_last_pc_move_tick = GetTick();      /* 记录：让手机命令别跟 PC 抢 */
    CHASSIS_Move(y, x, rx, (uint32_t)ms);

    /* 立即上报一次状态，PC 可据此确认"确实收到并动了" */
    SendChassisState();
    return;

arg_err:
    PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ARG");
}

/**
 * @brief  $MS —— 底盘立即停车
 */
static void HandleCmd_MS(const ProtoFrame_t *frame)
{
    (void)frame;

    CHASSIS_Stop();
    PROTO_SendAck("MS");
}

/**
 * @brief  $MC,pct —— 设定速度上限百分比（0=锁死底盘）
 */
static void HandleCmd_MC(const ProtoFrame_t *frame)
{
    char    arg[PROTO_ARG_MAX];
    int32_t pct;

    if (PROTO_GetArg(frame->payload, 1u, arg, (uint16_t)sizeof(arg)) == 0u)
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ARG");
        return;
    }
    pct = PROTO_ArgToInt(arg);
    if (pct < 0)   { pct = 0;   }
    if (pct > 100) { pct = 100; }

    PROTO_SendAck("MC");
    CHASSIS_SetSpeedPct((uint8_t)pct);
    SendChassisState();
}

/**
 * @brief  $MQ —— 查询底盘状态
 */
static void HandleCmd_MQ(const ProtoFrame_t *frame)
{
    (void)frame;

    PROTO_SendAck("MQ");
    SendChassisState();
}

/* ==========================================================================
 * 二·C、HC05 蓝牙输入（USART3）—— 手机手动控车
 *
 * 命令集与原底盘工程保持一致，方便手机端 App / 串口助手沿用：
 *     F 前进    B 后退    L 左转    R 右转
 *     Q 左前斜  E 右前斜  Z 左后斜  C 右后斜
 *     G 左平移  H 右平移
 *     + 加速    - 减速    S 停止
 *
 * ★ 优先级规则：PC 的 $MV 优先级高于手机。
 *   若 PC 在最近 300ms 内下发过底盘运动（g_last_pc_move_tick），
 *   手机命令一律忽略 —— 否则 auto 模式跑到一半被人碰手机就撞架了。
 * ========================================================================== */

/** PC 命令的"独占窗口"(ms)：这段时间内手机命令无效 */
#define HC05_PC_PRIORITY_MS     300u

/**
 * @brief  处理一个手机蓝牙命令字符
 * @param  c 命令字符
 * @retval 无
 */
static void HC05_SelectServo(uint8_t id)
{
    float pose[SERVO_COUNT];
    if (id >= SERVO_COUNT) { PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ID"); return; }
    SERVO_GetPose(pose);
    g_servo_sel = id;
    PROTO_SendPose(pose);
}

static void HC05_MoveServoTo(uint8_t id, float ang)
{
    if (id >= SERVO_COUNT) { PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ID"); return; }
    if (CHASSIS_IsMoving() != 0u) { PROTO_SendErr((uint8_t)PROTO_ERR_LOCKED, "CHS"); return; }
    if (SERVO_CheckLimit(id, ang) == 0u) { PROTO_SendErr((uint8_t)PROTO_ERR_LIMIT, "LIM"); return; }
    if (SERVO_MoveSingle(id, ang, (uint32_t)JOG_MOVE_MS, "SA") == 0u)
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_LIMIT, "LIM");
        return;
    }
    PROTO_SendAck("SA");
}

static void HC05_ServoUnit(uint8_t id, int8_t direction)
{
    float pose[SERVO_COUNT];
    float lim_min = 0.0f, lim_max = 0.0f, target, delta, diff;
    if (id >= SERVO_COUNT) { PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, "ID"); return; }
    delta = (direction >= 0) ? SERVO_UNIT_DEG : (0.0f - SERVO_UNIT_DEG);
    SERVO_GetPose(pose);
    SERVO_GetLimits(id, &lim_min, &lim_max);
    target = pose[id] + delta;
    if (target < lim_min) { target = lim_min; }
    if (target > lim_max) { target = lim_max; }
    diff = target - pose[id];
    if (diff < 0.0f) { diff = 0.0f - diff; }
    if (diff < SERVO_LIMIT_EPS_DEG) { PROTO_SendPose(pose); return; }
    HC05_MoveServoTo(id, target);
}

static void HC05_Gripper(uint8_t is_open)
{
    if (CHASSIS_IsMoving() != 0u) { PROTO_SendErr((uint8_t)PROTO_ERR_LOCKED, "CHS"); return; }
    if (SERVO_SetGripper(is_open, (uint32_t)DEFAULT_MOVE_MS, "GC") == 0u)
    {
        PROTO_SendErr((uint8_t)PROTO_ERR_LIMIT, "LIM");
        return;
    }
    PROTO_SendAck("GC");
}

static void HC05_GoHome(void)
{
    if (CHASSIS_IsMoving() != 0u) { PROTO_SendErr((uint8_t)PROTO_ERR_LOCKED, "CHS"); return; }
    if (SERVO_GoHome("HOM") == 0u) { PROTO_SendErr((uint8_t)PROTO_ERR_LIMIT, "LIM"); return; }
    PROTO_SendAck("HOM");
}

static void HC05_HandleChar(uint8_t c)
{
    if ((c == (uint8_t)'\r') || (c == (uint8_t)'\n')) { return; }
    if ((c >= 'a') && (c <= 'z')) { c = (uint8_t)(c - 'a' + 'A'); }

    /* ★ 急停 'S' 永远放行：即使正处 PC 命令独占窗内（auto 流程跑一半，
       手机急停也必须能刹住车 —— 安全优先于互斥）。 */
    if (c == 'S')
    {
        CHASSIS_Stop();
        SERVO_StopNow();
        PROTO_SendAck("S");
        return;
    }

    /* PC 命令独占窗：PC 刚下发过 $MV 的 300ms 内，手机其它命令一律忽略 */
    if ((uint32_t)(GetTick() - g_last_pc_move_tick) < (uint32_t)HC05_PC_PRIORITY_MS)
    {
        return;
    }

    if (SERVO_IsBusy() == 0u)
    {
        switch (c)
        {
            case 'F': CHASSIS_Move( 1.0f,  0.0f,  0.0f, 0u); return;
            case 'B': CHASSIS_Move(-1.0f,  0.0f,  0.0f, 0u); return;
            case 'L': CHASSIS_Move( 0.0f,  0.0f, -1.0f, 0u); return;
            case 'R': CHASSIS_Move( 0.0f,  0.0f,  1.0f, 0u); return;
            case 'Q': CHASSIS_Move( 1.0f, -1.0f,  0.0f, 0u); return;
            case 'E': CHASSIS_Move( 1.0f,  1.0f,  0.0f, 0u); return;
            case 'Z': CHASSIS_Move(-1.0f, -1.0f,  0.0f, 0u); return;
            case 'C': CHASSIS_Move(-1.0f,  1.0f,  0.0f, 0u); return;
            case 'G': CHASSIS_Move( 0.0f, -1.0f,  0.0f, 0u); return;
            case 'H': CHASSIS_Move( 0.0f,  1.0f,  0.0f, 0u); return;
            case '.':
            {
                uint8_t p = CHASSIS_GetSpeedPct();
                CHASSIS_SetSpeedPct((uint8_t)((p + 10u <= 100u) ? (p + 10u) : 100u));
                return;
            }
            case ',':
            {
                uint8_t p = CHASSIS_GetSpeedPct();
                CHASSIS_SetSpeedPct((uint8_t)((p >= 10u) ? (p - 10u) : 0u));
                return;
            }
            default:
                break;
        }
    }

    switch (c)
    {
        case '1': HC05_SelectServo(0u); return;
        case '2': HC05_SelectServo(1u); return;
        case '3': HC05_SelectServo(2u); return;
        case '4': HC05_SelectServo(3u); return;
        case '+': HC05_ServoUnit(g_servo_sel,  1); return;
        case '-': HC05_ServoUnit(g_servo_sel, -1); return;
        case 'O': HC05_Gripper(1u); return;
        case 'K': HC05_Gripper(0u); return;
        case '=': HC05_GoHome(); return;
        case '?':
        {
            float pose[SERVO_COUNT];
            SERVO_GetPose(pose);
            PROTO_SendPose(pose);
            return;
        }
        default:
            break;
    }

    if (((c >= 'A') && (c <= 'Z')) || ((c >= '0') && (c <= '9')))
    {
        char bad[2];
        bad[0] = (char)c;
        bad[1] = '\0';
        PROTO_SendErr((uint8_t)PROTO_ERR_UNKNOWN, bad);
    }
}

static void HC05_ProcessInput(void)
{
    uint8_t c;

    while (HC05_FifoPop(&c) != 0u)
    {
        HC05_HandleChar(c);
    }
}

/* ==========================================================================
 * 三、外设初始化（SPL）
 * ========================================================================== */

/**
 * @brief  系统时钟配置
 * @note   SPL 工程由启动文件中的 SystemInit() 完成时钟初始化
 *         （HSE 8MHz -> PLL x9 -> SYSCLK 72MHz，HCLK=72M，PCLK1=36M，PCLK2=72M，
 *         Flash 等待 2），本函数只负责刷新 SystemCoreClock 全局变量，
 *         供 RCC_GetClocksFreq / delay 库等使用。
 *         若你的 Blue Pill 上没有 8MHz 晶振（极少见），需要修改
 *         system_stm32f10x.c 中的 SetSysClockTo72 或改用 HSI
 *         （同时把 main.h 中的 SERVO_TIM_CLK_HZ 改成 64000000）。
 * @param  无
 * @retval 无
 */
void SystemClock_Config(void)
{
    SystemCoreClockUpdate();
}

/**
 * @brief  GPIO 初始化：LED(PC13) + AFIO 时钟
 * @note   舵机与串口的 GPIO 在各自的初始化函数中配置（SPL 无 MSP 机制，
 *         直接集中配置，效果等价）。
 * @param  无
 * @retval 无
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* AFIO 时钟：复用功能 IO 必须（本工程虽未用重映射，但保持与 HAL 版一致） */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);

    /* LED：推挽输出，初始熄灭（PC13 低电平点亮） */
    GPIO_WriteBit(LED_GPIO_PORT, LED_GPIO_PIN, (BitAction)LED_OFF);

    GPIO_InitStruct.GPIO_Pin   = LED_GPIO_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief  LED 翻转（PC13，低电平点亮）
 * @note   SPL 无 TogglePin 接口，直接对 ODR 异或，原子性足够（读-改-写
 *         期间不会被中断破坏，因为中断不碰 PC13）。
 * @param  无
 * @retval 无
 */
void GPIO_ToggleLed(void)
{
    LED_GPIO_PORT->ODR ^= LED_GPIO_PIN;
}

#if USE_RX_DMA
/**
 * @brief  DMA 初始化：仅开时钟（通道配置在 MX_USART1_UART_Init 中完成）
 * @param  无
 * @retval 无
 */
static void MX_DMA_Init(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}
#endif /* USE_RX_DMA */

/**
 * @brief  USART1 初始化：115200 / 8 / N / 1 + RX 接收路径
 * @note   PA9 = TX（复用推挽），PA10 = RX（上拉输入）。
 *         USE_RX_DMA=1：DMA1_Channel5 循环模式 + IDLE 中断（原版方案）；
 *         USE_RX_DMA=0：仅开 RXNE 中断，逐字节入 FIFO（Keil 仿真路径）。
 * @param  无
 * @retval 无
 */
static void MX_USART1_UART_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
#if USE_RX_DMA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif

    /* ---- PA9 = USART1_TX：复用推挽 ---- */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ---- PA10 = USART1_RX：上拉输入（与原 HAL 版 GPIO_PULLUP 一致） ---- */
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ---- USART1：115200 / 8 / N / 1 ---- */
    USART_InitStruct.USART_BaudRate            = USART1_BAUDRATE;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStruct);

    USART_Cmd(USART1, ENABLE);

#if USE_RX_DMA
    /* ---- DMA1_Channel5 = USART1_RX：循环模式，字节宽度 ---- */
    {
        DMA_InitTypeDef DMA_InitStruct;

        DMA_DeInit(DMA1_Channel5);

        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(USART1->DR));
        DMA_InitStruct.DMA_MemoryBaseAddr     = (uint32_t)g_rx_dma_buf;
        DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralSRC;
        DMA_InitStruct.DMA_BufferSize         = (uint32_t)RX_DMA_BUF_SIZE;
        DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
        DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
        DMA_InitStruct.DMA_Mode                = DMA_Mode_Circular;   /* 环形，永不停止 */
        DMA_InitStruct.DMA_Priority            = DMA_Priority_High;
        DMA_InitStruct.DMA_M2M                 = DMA_M2M_Disable;
        DMA_Init(DMA1_Channel5, &DMA_InitStruct);
    }
#endif /* USE_RX_DMA */

    /* ---- NVIC ---- */
    /* 优先级分组为 4 位抢占优先级（main 中已配置）。
       TIM3=0（最高，保证 1ms 节拍）；USART1=0（原 HAL 版取值）；DMA1_Ch5=1 */
    NVIC_SetPriority(TIM3_IRQn, 0u);
    NVIC_EnableIRQ(TIM3_IRQn);

#if USE_RX_DMA
    NVIC_SetPriority(DMA1_Channel5_IRQn, 1u);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);
#endif

    NVIC_SetPriority(USART1_IRQn, 0u);
    NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief  USART3 初始化：HC05 蓝牙（9600 / 8 / N / 1）
 * @note   PB10 = USART3_TX（复用推挽），PB11 = USART3_RX（上拉输入）。
 *         USART3 默认映射就是 PB10/PB11，**不需要重映射**。
 *
 *         ★ 为什么不用 USART2：USART2 固定占 PA2/PA3，而 PA2/PA3 是
 *           舵机 s2/s3 的 TIM2_CH3/CH4 输出。用 USART2 就得把整个舵机
 *           PWM 迁到 TIM4 并重新标定；而 PB10/PB11 在本工程里完全空闲，
 *           用 USART3 可以让舵机【一根线都不动】，已标定的点位表继续有效。
 *
 *         接收用 RXNE 中断 + 软件 FIFO，与 USART1 的 USE_RX_DMA=0 路径同构，
 *         但两者用的是**各自的 FIFO**，互不干扰（手机与 PC 可同时在线）。
 * @param  无
 * @retval 无
 */
static void MX_USART3_UART_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* ---- PB10 = USART3_TX：复用推挽 ---- */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ---- PB11 = USART3_RX：上拉输入（HC05 未上电时不会读到随机低电平） ---- */
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ---- USART3：9600 / 8 / N / 1（HC05 出厂默认） ---- */
    USART_InitStruct.USART_BaudRate            = USART3_BAUDRATE;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStruct);

    /* 先清一次接收标志，避免上电残留的 ORE 立刻触发中断 */
    (void)USART_ReceiveData(USART3);
    USART_ClearFlag(USART3, USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_PE);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);

    /* 优先级 1（低于 USART1/TIM3 的 0）：手机命令不着急，让位给运动节拍 */
    NVIC_SetPriority(USART3_IRQn, 1u);
    NVIC_EnableIRQ(USART3_IRQn);
}

/* ==========================================================================
 * 二·D、HC05 FIFO 底层（供 USART3 中断与主循环使用）
 * ========================================================================== */

/**
 * @brief  把一字节压入 HC05 FIFO
 * @note   与 PC 的 Fifo_Push 同构：索引回绕用 "& (SIZE-1)"（要求 SIZE 是 2 的幂）。
 *         满时丢弃新字节，不破坏 FIFO 结构。
 * @param  b 待入队字节
 * @retval 无
 */
void HC05_RxBytePush(uint8_t b)
{
    uint16_t next;

    next = (uint16_t)((g_hc05_fifo.head + 1u) & (uint16_t)HC05_FIFO_MASK);
    if (next == (uint16_t)g_hc05_fifo.tail)
    {
        return;   /* 满：丢弃 */
    }
    g_hc05_fifo.buf[g_hc05_fifo.head] = b;
    g_hc05_fifo.head = next;
}

/**
 * @brief  从 HC05 FIFO 取出一个字节
 * @param  b 输出字节（可为 NULL，仅查询时用）
 * @retval 1 取到；0 空
 */
uint8_t HC05_FifoPop(uint8_t *b)
{
    if (g_hc05_fifo.head == g_hc05_fifo.tail)
    {
        return 0u;
    }
    if (b != NULL)
    {
        *b = g_hc05_fifo.buf[g_hc05_fifo.tail];
    }
    g_hc05_fifo.tail = (uint16_t)((g_hc05_fifo.tail + 1u) & (uint16_t)HC05_FIFO_MASK);
    return 1u;
}

/**
 * @brief  USART3 错误恢复：清错误标志 + 清 FIFO
 * @note   手机蓝牙偶尔会因距离抖动产生噪声字节（ORE/FE）。
 *         这里把 FIFO 也清掉是合理的 —— 手机命令是"当前状态"式的，
 *         丢掉几个残留字符不影响使用（人松手后还会再发 S 停机）。
 *         这一点与 USART1 不同：PC 的帧有校验、不能随便丢，所以那边
 *         只复位解析状态机、不清 FIFO。
 * @param  无
 * @retval 无
 */
void HC05_RxErrorRecover(void)
{
    volatile uint32_t tmp;

    tmp = USART3->SR;
    tmp = USART3->DR;
    (void)tmp;

    USART_ClearFlag(USART3, USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_PE);

    g_hc05_fifo.head = 0u;
    g_hc05_fifo.tail = 0u;

    /* 重新使能 RXNE 中断（ORE 会把它关掉） */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

/**
 * @brief  TIM2 初始化：4 路 50Hz PWM（舵机）
 * @note   PSC = 71 -> 1MHz（1 计数 = 1us），ARR = 19999 -> 20ms。
 *         4 个通道共用同一个计数器，天然同相位，适合多路舵机同步。
 *         初始 Pulse 设为中位 1500us；**这里不启动计数器、不使能 CCx 输出**，
 *         输出的使能在 SERVO_Init() 中写完 CCR 之后统一进行，
 *         防止上电瞬间输出随机宽脉冲（这是与原 HAL 版一致的安全顺序）。
 * @param  无
 * @retval 无
 */
static void MX_TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_BaseInitStruct;
    TIM_OCInitTypeDef       TIM_OCInitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* ---- PA0/PA1/PA2/PA3 = TIM2 CH1~CH4：复用推挽 ---- */
    {
        GPIO_InitTypeDef GPIO_InitStruct;

        GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
        GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;   /* 50Hz 无需高速翻转 */
        GPIO_Init(GPIOA, &GPIO_InitStruct);
    }

    /* ---- 时基：1us 计数 / 20ms 周期 ---- */
    TIM_BaseInitStruct.TIM_Period        = SERVO_PWM_ARR;
    TIM_BaseInitStruct.TIM_Prescaler     = SERVO_PWM_PSC;
    TIM_BaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseInitStruct.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_BaseInitStruct);

    /* ---- 四个通道：PWM1 模式，初始中位 1500us，但先不使能输出 ---- */
    TIM_OCInitStruct.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_Pulse       = 1500u;   /* 初始中位 1.5ms */
    TIM_OCInitStruct.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Disable;  /* 输出留给 SERVO_Init */

    TIM_OC1Init(TIM2, &TIM_OCInitStruct);
    TIM_OC2Init(TIM2, &TIM_OCInitStruct);
    TIM_OC3Init(TIM2, &TIM_OCInitStruct);
    TIM_OC4Init(TIM2, &TIM_OCInitStruct);

    /* 不使能预装载：CCR 直接生效，与 HAL 版 AutoReloadPreload/OCFast 关闭一致 */
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);

    /* 不调用 TIM_Cmd(TIM2, ENABLE) —— 见函数头注释 */
}

/**
 * @brief  TIM3 初始化：1ms 更新中断（插值时基）
 * @note   PSC = 71 -> 1MHz，ARR = 999 -> 1kHz。不占用任何引脚。
 *         计数器使能放在 main()（等价原 HAL_TIM_Base_Start_IT）。
 * @param  无
 * @retval 无
 */
static void MX_TIM3_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_BaseInitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_BaseInitStruct.TIM_Period        = CTRL_TIM_ARR;
    TIM_BaseInitStruct.TIM_Prescaler     = CTRL_TIM_PSC;
    TIM_BaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseInitStruct.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_BaseInitStruct);

    /* 更新中断：1ms 一次（中断服务在 stm32f10x_it.c 的 TIM3_IRQHandler） */
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

#if USE_IWDG
/**
 * @brief  独立看门狗初始化
 * @note   IWDG 由独立的 40kHz 内部 RC 供电，主时钟挂了它也能复位芯片。
 *         预分频 64 -> 625Hz，重载 1250 -> 约 2s 超时。
 *         主循环每圈都会 IWDG_ReloadCounter()，因此正常运行时不会复位；
 *         一旦程序跑飞或死循环，2s 后自动复位，舵机会回到上电安全流程。
 *         !! Keil 仿真调试时保持 USE_IWDG=0，否则单步调试会被复位打断 !!
 * @param  无
 * @retval 无
 */
static void MX_IWDG_Init(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_PRESCALER);
    IWDG_SetReload(IWDG_RELOAD);
    IWDG_ReloadCounter();
    IWDG_Enable();
}
#endif

/**
 * @brief  错误处理：点亮/熄灭 LED 快速闪烁并停在此处
 * @note   初始化失败才会进入。死循环前先关掉 PWM 输出，避免舵机被锁在
 *         某个位置持续受力（堵转发热）。
 * @param  无
 * @retval 无
 */
void Error_Handler(void)
{
    /* 注意：不能关中断，因为 Delay() 依赖 SysTick 中断递增的时基；
       初始化失败时程序本来也不再驱动舵机，保持中断开启反而更安全。 */
    Servo_PwmStopAll();

    while (1)
    {
        GPIO_ToggleLed();
        Delay(100u);
    }
}
