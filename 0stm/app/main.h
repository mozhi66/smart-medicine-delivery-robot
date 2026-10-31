/**
  ******************************************************************************
  * @file    main.h
  * @brief   智能药箱 STM32 下位机 —— 引脚/外设/全局参数宏定义（标准外设库版）
  *
  * @note    本文件是整个固件的"硬件与参数配置中心"。所有与芯片引脚、定时器通道、
  *          舵机参数、通信缓冲、Flash 地址相关的可调量都集中在这里，改板子或改舵机
  *          型号时只需要改本文件，其它 .c 文件无需改动。
  *
  * @note    本文件由原 HAL 版（stm32/Core/Inc/main.h）移植而来：
  *          库从 STM32F1xx HAL 换成标准外设库（StdPeriph Driver, SPL），
  *          硬件引脚/定时器/参数全部保持一致。
  *
  * @author  智能药箱毕业设计（Keil SPL 移植版）
  * @version V1.0
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* 标准外设库头文件（SPL 3.6，本工程 std_periph_driver/inc 下） */
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_iwdg.h"
#include "misc.h"

/* ==========================================================================
 * 一、固件版本信息
 * ========================================================================== */

/** 固件版本字符串，随 $READY,VERx.y 上发给 PC，PC 端可做兼容性判断 */
#define FW_VERSION_STRING       "VER1.0"

/* ==========================================================================
 * 二、串口（USART1）与接收缓冲配置
 *
 *  物理层：USART1 / 115200 / 8 / N / 1
 *  TX = PA9 (复用推挽)   RX = PA10 (上拉输入)
 *  RX 有两种接收方式（见 USE_RX_DMA）：
 *     =1  DMA1_Channel5 循环模式 + IDLE 中断（原 HAL 版方案，适合真机）
 *     =0  RXNE 中断逐字节直收（Keil 仿真器支持最可靠，真机同样可用）
 * ========================================================================== */

/** 串口波特率，必须与 PC 端一致 */
#define USART1_BAUDRATE         115200u

/**
 * 串口接收方式开关：
 *   1 = DMA1_Channel5 循环搬运 + IDLE 空闲中断 + 软件 FIFO（原版方案）
 *   0 = USART1 RXNE 中断逐字节入 FIFO（Keil 仿真调试用默认值）
 *
 * 说明：Keil 仿真器对 USART 的 RXNE 中断模拟最可靠；DMA+IDLE 方案在仿真器里
 *       可能不会被完整模拟（IDLE 标志、外设 DMA 请求）。两者协议解析完全一致，
 *       真机需要原版 DMA 方案时把这里改回 1 即可。
 */
#define USE_RX_DMA              0u

/** DMA 循环接收缓冲区大小（字节）。
 *  取 256 的原因：单帧最大 128 字节，留出 2 帧余量，即使主循环被
 *  短暂的 Flash 写操作阻塞也不会覆盖未取走的数据。 */
#define RX_DMA_BUF_SIZE         256u

/** 软件字节 FIFO 大小（字节）。
 *  !! 必须是 2 的幂 !! 因为 FIFO 的读写索引回绕用 "& (SIZE-1)" 实现，
 *  在中断里避免除法运算。512 字节足够缓存 4 帧以上。 */
#define RX_FIFO_SIZE            512u
#define RX_FIFO_MASK            (RX_FIFO_SIZE - 1u)

/** 单帧正文最大长度（不含 '$'、'*'、校验和、CRLF）。
 *  目前最长的指令 $SA,180.0,180.0,180.0,180.0,60000*XX 也只有 ~40 字节，
 *  留足余量。超长帧会被丢弃并上报 $ERR,02。 */
#define PROTO_LINE_MAX          128u

/* ==========================================================================
 * 三、舵机数量与分工
 *
 *  ★★ 本机械臂是【平面 3R 串联臂】：3 个关节全部是【俯仰】摆动（绕同一
 *     方向的水平轴），末端被限制在【单一竖直平面】内，工作空间是该平面内的
 *     一个扇形。**没有底座方位角（yaw）自由度** —— 所以所有药槽必须排在同一
 *     个竖直平面里，"靠底座转角区分槽位"的做法在本结构下不成立。
 *     （详见 智能药箱系统/pc/core/kinematics.py 顶部说明）
 *
 *  servo0 = 底座关节俯仰（带动整条臂前后倾）—— 不是 yaw 旋转！
 *  servo1 = 大臂俯仰（肩）
 *  servo2 = 小臂俯仰（肘）
 *  servo3 = 夹爪开合（不参与定位）
 *
 *  平面 3R 的好处：平面内自由度 = 3（位置 2 + 末端姿态 1），
 *  因此【位置和姿态可以同时指定】—— 抓竖直药瓶时能锁定夹爪竖直朝下。
 * ========================================================================== */

/** 舵机通道总数，改这个数需要同步改下面的角度宏与 TIM 通道表 */
#define SERVO_COUNT             4u

/** 各舵机的通道编号（语义别名，提高可读性）。
 *  ⚠ 宏名 SERVO_ID_YAW 是移植前遗留：原设计曾把它当方位角用，
 *    实际功能是【底座俯仰】。为保持与 servo.c / 已标定的配置兼容，
 *    这里【保留宏名不改】，只更正语义 —— 读代码时把它当"底座关节"即可。 */
#define SERVO_ID_YAW            0u
#define SERVO_ID_SHOULDER       1u
#define SERVO_ID_ELBOW          2u
#define SERVO_ID_GRIPPER        3u

/* ==========================================================================
 * 四、PWM 定时器与通道、引脚映射
 *
 *  使用 TIM2 的 4 个通道同时输出 50Hz PWM，四个通道共享同一个计数器，
 *  因此天然同步（不会出现相位差），非常适合多路舵机。
 *
 *  ----------------------------------------------------------------------
 *  通道       定时器      引脚      复用映射          说明
 *  ----------------------------------------------------------------------
 *  servo0     TIM2_CH1    PA0       AFIO 无重映射     底座俯仰
 *  servo1     TIM2_CH2    PA1       AFIO 无重映射     大臂俯仰
 *  servo2     TIM2_CH3    PA2       AFIO 无重映射     小臂俯仰
 *  servo3     TIM2_CH4    PA3       AFIO 无重映射     夹爪开合
 *  ----------------------------------------------------------------------
 *  注意 1：TIM2 的四个通道在 PA0~PA3 上**没有可用的重映射**
 *          （TIM2 部分重映射 1 是 PA15/PB3/PA2/PA3，会把引脚拆散，
 *            且 PB3/PA15 是 JTAG 脚，本工程采用无重映射的默认映射）。
 *  注意 2：PA0~PA3 在本工程里只被 TIM2 的 PWM 输出占用，
 *          不与 USART1（PA9/PA10）、LED（PC13）冲突。
 *
 *  1ms 插值基准定时器：TIM3（仅内部计数，不占用引脚）
 * ========================================================================== */

/** PWM 定时器输入时钟频率(Hz)。
 *  = SystemCoreClock（因为 APB1 二分频后定时器时钟被自动倍频回 72MHz） */
#define SERVO_TIM_CLK_HZ        72000000u

/** PWM 输出频率(Hz)，标准舵机 50Hz；部分数字舵机可到 333Hz，
 *  改这里即可（同时要注意舵机是否支持）。 */
#define SERVO_PWM_FREQ_HZ       50u

/** PWM 定时器预分频值：72MHz / 72 = 1MHz，即 1 个计数 = 1us，
 *  这样 CCR 寄存器里直接填"微秒数"，换算最直观、分辨率 1us。 */
#define SERVO_PWM_PSC           ((uint16_t)(SERVO_TIM_CLK_HZ / 1000000u - 1u))

/** PWM 自动重装载值：1MHz / 50Hz = 20000 计数 = 20ms 周期 */
#define SERVO_PWM_ARR           ((uint16_t)(1000000u / SERVO_PWM_FREQ_HZ - 1u))

/** 1ms 定时中断所用定时器 TIM3 的分频与周期（TIM3CLK 同为 72MHz） */
#define CTRL_TIM_CLK_HZ         72000000u
#define CTRL_TIM_PSC            ((uint16_t)(CTRL_TIM_CLK_HZ / 1000000u - 1u))  /* 1MHz */
#define CTRL_TIM_ARR            ((uint16_t)(1000u - 1u))                       /* 1ms  */

/* ==========================================================================
 * 五、舵机脉宽与角度范围
 *
 *  角度 -> 脉宽：pulse_us = PULSE_MIN + (angle / SPAN) * (PULSE_MAX - PULSE_MIN)
 * ========================================================================== */

/** 0 度对应脉宽(us)，标准 500us */
#define SERVO_PULSE_MIN_US      500u
/** 满量程对应脉宽(us)，标准 2500us */
#define SERVO_PULSE_MAX_US      2500u

/** 舵机满量程角度(度)。
 *  普通 180 度舵机保持 180.0f；
 *  若使用 270 度舵机（2500us 对应 270 度），改为 270.0f 即可，
 *  此时所有限位/指令角度都按 0~270 解释。 */
#define SERVO_ANGLE_SPAN_DEG    180.0f

/** 硬件绝对下限/上限：任何情况下（包括 CFG 配置的限位）都不能超过这个范围，
 *  防止配置错误导致舵机打到机械死点。取 0 ~ 满量程。 */
#define SERVO_ABS_MIN_DEG       0.0f
#define SERVO_ABS_MAX_DEG       SERVO_ANGLE_SPAN_DEG

/** 限位判定容差(度)。
 *  浮点运算与 PC 端四舍五入可能带来极小误差，角度在 [min-eps, max+eps]
 *  内仍视为合法，超出才算越限。取 0.1 度不会影响任何实际动作。 */
#define SERVO_LIMIT_EPS_DEG     0.1f

/* ==========================================================================
 * 六、默认角度：HOME 位、夹爪预设、默认软限位
 *
 *  这些默认值只在 Flash 中还没有有效配置时（首次烧录）被写入；
 *  一旦通过 $CFG 修改过，就以 Flash 中的值为准。
 * ========================================================================== */

/** 各舵机 HOME（零位）角度，上电后平滑运动到这里。
 *  ★ S0~S2 对齐 PC 端 poses.json 的 home 姿态：180 / 120 / 120；
 *  ★ 夹爪 S4（servo3）按 2026-10-22 实机约定：180°=复位/完全张开，
 *    140°=抓药夹紧；135° 以下越过机械止点（堵转卡死烧舵机）。
 *    （旧 2026-09-30 的 90/45 体系与当前实机不符，已整体切换。） */
#define SERVO0_HOME_DEG         180.0f  /* 姿态 A：底座 */
#define SERVO1_HOME_DEG         120.0f  /* 姿态 A：大臂 */
#define SERVO2_HOME_DEG         120.0f  /* 姿态 A：小臂 */
#define SERVO3_HOME_DEG         180.0f  /* 夹爪复位=完全张开（140°为抓药夹紧位） */

/** 夹爪（servo3）两个预设角度：$GC,open / $GC,close 使用。
 *  ★ 2026-10-22 实机约定：180°=完全张开（复位），140°=抓药夹紧。
 *  抓取时先 $GC,open 再 $GC,close（见 state_machine.pick_from_slot）。
 *  ★ 注意：夹爪机械行程只有 140~180°，135° 以下没有行程（顶死机械止点）——
 *    继续往小打就是堵转发热烧舵机。LIMIT_MIN 已锁死 140，任何越界指令
 *    都会被拒绝（$ERR,03）。想加大夹持力只能微调 CLOSE（如 141~142），
 *    并同步放宽 SERVO3_LIMIT_MIN_DEG，改完务必重新扫角验证。 */
#define GRIPPER_OPEN_DEG        180.0f  /* 完全张开（复位位，实机调参值） */
#define GRIPPER_CLOSE_DEG       140.0f  /* 抓药夹紧（实机复测：135 会堵转卡死，140 恰好） */

/** 各舵机默认软限位最小值(度) */
#define SERVO0_LIMIT_MIN_DEG    0.0f
#define SERVO1_LIMIT_MIN_DEG    15.0f   /* 大臂不要压到桌面以下 */
#define SERVO2_LIMIT_MIN_DEG    0.0f
#define SERVO3_LIMIT_MIN_DEG    140.0f  /* 140 = 抓药夹紧角（实测 135 堵转）；再小禁止 → 防堵转烧舵机 */

/** 各舵机默认软限位最大值(度)
 *  ★ 修复：融合版原 main.h 缺这 3 个宏，servo.c 引用时编译报
 *    "error: #20: identifier ... is undefined"。数值取自 HC05 独占版
 *    （与 PC 端 poses.json 的姿态值域一致：S0 最大 180 / S1 最大 165 / S2 最大 150）。 */
#define SERVO0_LIMIT_MAX_DEG    180.0f
#define SERVO1_LIMIT_MAX_DEG    165.0f
#define SERVO2_LIMIT_MAX_DEG    150.0f
#define SERVO3_LIMIT_MAX_DEG    180.0f  /* 180 = 完全张开/复位（实机调参值） */

/** 各舵机中位微调(us)，用于补偿舵机个体差异与安装偏心。
 *  正值 = 脉宽加大。装机时若发现某关节偏了几度，改这里最省事。 */
#define SERVO0_TRIM_US          0
#define SERVO1_TRIM_US          0
#define SERVO2_TRIM_US          0
#define SERVO3_TRIM_US          0

/** 方向取反标志：1 = 该舵机安装方向相反，输出脉宽在 [MIN,MAX] 内镜像。
 *  注意镜像只作用于"脉宽输出"，限位仍按真实角度解释。 */
#define SERVO0_INVERT           0u
#define SERVO1_INVERT           0u
#define SERVO2_INVERT           0u
#define SERVO3_INVERT           0u

/** 上电时"假定"舵机所处的角度。
 *  舵机没有角度反馈，STM32 上电时无法知道真实位置，因此必须假定一个值。
 *  默认等于 HOME，这样上电后 HOME 运动位移量为 0，舵机纹丝不动 —— 最安全。
 *  如果希望在演示时看到明显的"归位"动作，可以把这里改成 0，
 *  固件会从 0 度平滑插值到 HOME 位。 */
#define SERVO0_BOOT_DEG         SERVO0_HOME_DEG
#define SERVO1_BOOT_DEG         SERVO1_HOME_DEG
#define SERVO2_BOOT_DEG         SERVO2_HOME_DEG
#define SERVO3_BOOT_DEG         SERVO3_HOME_DEG

/* ==========================================================================
 * 七、运动参数
 * ========================================================================== */

/** 上电归位运动时长(ms)：从 BOOT 角平滑移动到 HOME 角的时间 */
#define BOOT_HOME_MOVE_MS       1200u

/** $HOM 指令的归位运动时长(ms) */
#define HOME_MOVE_MS            1500u

/** 若指令给出的运动时长为 0 或不合法，使用的默认时长(ms) */
#define DEFAULT_MOVE_MS         800u
#define JOG_MOVE_MS             400u    /* HC05 点动一格的时长（短、跟手） */
#define SERVO_UNIT_DEG          10.0f   /* HC05 '+'/'-' 每次步进角度 */

/** 运动时长下限(ms)。
 *  插值周期为 1ms，时长小于 2ms 时插值步数不足，容易一步跳到位。 */
#define MIN_MOVE_MS             20u

/** 运动时长上限(ms)，防止 PC 端误发超大值导致长时间堵占 */
#define MAX_MOVE_MS             60000u

/** 执行超时判定：实际耗时超过"理论时长 + 该值"仍未结束，上报 $ERR,04。
 *  正常情况下插值一定会按时结束，这个只是软件异常的最后兜底。 */
#define MOTION_TIMEOUT_MARGIN_MS 2000u

/** 连续超时次数达到该值后，认为舵机异常/堵转，上报 $ERR,05 */
#define MOTION_STALL_THRESHOLD  3u

/** 心跳帧周期(ms)，1Hz */
#define HB_PERIOD_MS            1000u

/** 急停后是否立即停止 PWM 输出（0 = 保持输出当前脉宽，舵机有力矩保持位置；
 *  1 = 关闭 PWM 输出，舵机脱力）。一般机械臂建议 0，避免掉臂砸东西。 */
#define ESTOP_RELEASE_PWM       0u

/* ==========================================================================
 * 七·B、夹爪防烧参数（扫角 + 夹紧保活）—— 2026-09-29 新增
 *
 *  痛点：夹爪(S4/SERVO3)夹住药瓶或顶到机械卡位后，舵机仍在向目标角出力
 *  （SG90 无反馈，固件无法知道"已经到位"），持续堵转大电流 -> 发热烧机。
 *
 *  防烧原理（纯软件，无需改动硬件）：
 *    1) 慢速扫角：闭合动作强制拉长到 GRIP_SCAN_CLOSE_MS，平稳逼近少冲击；
 *    2) 夹紧保活：凡运动结束时夹爪停在【夹紧角】（含 $GC,close、$SA 序列的
 *       lift/deliver 点、HC05 手动点动到闭合），自动进入"补压/卸力"循环：
 *         补压相 GRIP_HOLD_ACTIVE_MS  : 输出夹紧角（有力，夹住药瓶）
 *         卸力相 GRIP_HOLD_REST_MS    : 输出夹紧角+GRIP_HOLD_BACKOFF_DEG
 *                                       （几乎不受力，舵机散热）
 *       占空比 = 250/(250+1250) = 17%，平均电流降到约 1/5，杜绝烧机；
 *       周期性补压保证药瓶不会蠕动松脱。
 *    3) 任何新运动（$SA/$SS/$GC/$HOM/$RUN/HC05 点动）自动退出保活，
 *       与现有插值/序列机制零冲突。
 * ========================================================================== */

/** 夹爪保活总开关（1 = 启用；0 = 恢复旧的"持续顶死"行为）。
 *  ★ 2026-10-22 改 0：用户要求抓药时 S4 到位后【定住不动】——保活的
 *    "补压/卸力"循环会让夹爪每 1.5s 在 140°↔144° 来回摆动，不符合本机流程
 *    （流程：夹住药 -> 保持姿态 -> 人工 HC05 接管）。
 *    关闭后夹紧角持续出力（近似堵转电流）：★ 夹持时间别太长，舵机发烫就停。 */
#define GRIP_HOLD_ENABLE        0u

/** 闭合扫角最短时长(ms)：$GC,close / HC05 'K' 的实际运动时长不会短于该值。
 *  90°/1200ms = 75°/s，平稳逼近；撞上药瓶时角速度低、冲击小。 */
#define GRIP_SCAN_CLOSE_MS      1200u

/** 夹紧保活：补压相时长(ms)——输出完整夹紧力，把药瓶夹稳 */
#define GRIP_HOLD_ACTIVE_MS     250u

/** 夹紧保活：卸力相时长(ms)——回退一点角度，舵机几乎不受力、散热 */
#define GRIP_HOLD_REST_MS       1250u

/** 卸力相回退角(度)：从夹紧角往张开方向回退多少度算"卸力"。
 *  只回退一点点：药瓶靠夹爪摩擦仍保持姿态，但舵机齿轮卸载。
 *  若发现药瓶在保活期会松脱打滑，减小该值（如 2.0f）。 */
#define GRIP_HOLD_BACKOFF_DEG   4.0f

/** 判定"夹爪停在夹紧角"的角度容差(度)：运动完成时
 *  |angle_target - grip_close| <= 该值 才进入保活。 */
#define GRIP_HOLD_ANGLE_EPS     1.0f

/* ==========================================================================
 * 八、动作序列（$RUN）存储参数
 * ========================================================================== */

/** Flash 中预存的动作序列槽位数（$RUN,slot_id 的 slot_id 取值 1..N） */
#define RUN_SLOT_COUNT          4u

/** 每个动作序列最多包含的点位数。
 *  单点 = 20 字节（4 个 float + 2 个 uint16），
 *  4 槽 x 8 点 + 头部 ≈ 668 字节，刚好放得进 1KB 的 Flash 页。 */
#define RUN_POINTS_PER_SLOT     8u

/** 单槽默认点数不足时，未使用的点位被视为无效（以 count 字段为准） */

/* ==========================================================================
 * 九、Flash 持久化配置（STM32F103C8T6：64KB，页大小 1KB，共 64 页）
 *
 *  页 62 : 0x0800F800  舵机限位/HOME/夹爪/微调等配置
 *  页 63 : 0x0800FC00  动作点位表
 *
 *  !! 注意 !!
 *  1) 必须保证链接脚本里 FLASH 的实际使用量不要落到最后一页，
 *     即程序大小 < 63KB，否则会擦掉自己的代码。
 *  2) 若你的芯片实际是 128KB（很多 C8T6 实际是 CB，128KB），
 *     页 62/63 的地址会变化（128KB 时最后一页是 0x0801FC00），
 *     此时请把下面两个地址改成最后两页，或保持 C8 的 64KB 地址（一样可用，
 *     因为 128KB 芯片的低 64KB 地址映射完全相同）。
 * ========================================================================== */

/** F103 小容量/中容量产品页大小 1KB */
#define MB_FLASH_PAGE_SIZE      1024u

/** 配置区所在页起始地址（页 62） */
#define CFG_FLASH_ADDR          (0x08000000u + 62u * MB_FLASH_PAGE_SIZE)

/** 动作表所在页起始地址（页 63） */
#define ACT_FLASH_ADDR          (0x08000000u + 63u * MB_FLASH_PAGE_SIZE)

/** 配置区魔数：读出不相等则认为数据无效，加载默认值。
 *  取值 "MBC1"（MedBox Config 1）的 ASCII 码组合 */
#define CFG_STORE_MAGIC         0x3163424Du

/** 动作表魔数，取值 "MBA1"（MedBox Action 1） */
#define ACT_STORE_MAGIC         0x3141424Du

/** 存储结构版本号，结构体布局变化时递增，用于自动失效旧数据。
 *  ★ 2026-09-29 升到 2u：夹爪默认限位/预设角改为实机调参值（夹爪 0~90）。
 *  ★ 2026-09-30 升到 3u：实测 45°=恰好闭合，夹爪体系改为 SERVO3 限位 45~90、
 *    GRIPPER 90/45、HOME 90。升版本让上电自动忽略 Flash 里的 2u 旧配置
 *    （旧下限 0 会放行 45 以下的堵转角度），改用新宏默认值。
 *  ★ 2026-10-22 升到 4u：夹爪实机重新调装，体系改为 140~180（180=复位/张开、
 *    140=抓药夹紧、135 以下堵转死区）。升版本作废 Flash 里的 3u 旧配置
 *    （旧值域与实机不符：上电归位即堵转 + 拒绝 140~180 的合法指令）。
 *    不升级的话，旧 Flash 配置会被当成有效数据继续用，新限位不生效。 */
#define CFG_STORE_VERSION       4u
#define ACT_STORE_VERSION       1u

/* ==========================================================================
 * 十、麦轮底盘（4 电机 / 2 片 TB6612FNG）
 *
 *  ----------------------------------------------------------------------
 *  ★★ 为什么底盘能用这些引脚（资源分配的全过程，改引脚前必读）★★
 *
 *  固定不可动的：
 *      USART1 = PA9/PA10 @115200  -> PC（机械臂协议帧，必须保留）
 *      TIM3   = 1ms 插值基准（内部计数，不占引脚）
 *      PC13   = LED
 *
 *  冲突点与解法：
 *      底盘原本用 TIM3 出 4 路电机 PWM —— 但 TIM3 已被机械臂拿去当 1ms 基准，
 *      两者无法共存（一个定时器只有一套 PSC/ARR）。故电机 PWM 改到 **TIM4**。
 *
 *      TIM4 在 C8T6(LQFP48) 上**只有默认映射**可用：
 *          TIM4_CH1 = PB6   TIM4_CH2 = PB7
 *          TIM4_CH3 = PB8   TIM4_CH4 = PB9
 *      （TIM4 部分重映射会把通道搬到 PD12~PD15，而 LQFP48 封装没有 PD 口，
 *        所以重映射在本芯片上不可用。这 4 个脚在本工程里全部空闲。）
 *
 *      底盘原本的 8 个方向脚里，PA2/PA3 撞了舵机 s2/s3(TIM2_CH3/CH4)，
 *      PB8/PB9/PB10/PB11 又被 TIM4 的 PWM 与 HC05 串口占走，
 *      所以方向脚【整体重排】到下面这些纯 GPIO 脚：
 *          PA4/PA5/PA6/PA7  —— 不参与任何复用
 *          PB12/PB13        —— 底盘 RR 方向（BIN1/BIN2）
 *          PB14/PB15        —— 底盘 RL 方向（AIN1/AIN2）
 *          ★ 不要用 PB0/PB1：与旧底盘工程 TIM3 PWM 脚冲突，
 *            实机出现过「左后轮单向不动/正转异常」。
 *
 *  HC05 蓝牙走 **USART3**（PB10=TX / PB11=RX），不走 USART2。
 *  原因：USART2 固定占 PA2/PA3，会把舵机 s2/s3 挤走，导致 4 路舵机 PWM
 *        必须整体迁移 + 重新标定；而 USART3 的 PB10/PB11 是纯空闲脚，
 *        用它可以做到【舵机一根线都不动】，已标定好的 poses.json 完全有效。
 *
 *  ----------------------------------------------------------------------
 *  电机编号与引脚（0=FL 前左 / 1=FR 前右 / 2=RL 后左 / 3=RR 后右）
 *
 *    电机        TB6612 信号   STM32 引脚        说明
 *    ---------------------------------------------------------------
 *    FL(0)       AIN1          PA4              方向 1
 *    FL(0)       AIN2          PA5              方向 2
 *    FL(0)       PWMA          PB6 (TIM4_CH1)   速度 PWM
 *    FR(1)       BIN1          PA6              方向 1
 *    FR(1)       BIN2          PA7              方向 2
 *    FR(1)       PWMB          PB7 (TIM4_CH2)   速度 PWM
 *    RL(2)       AIN1          PB14             方向 1
 *    RL(2)       AIN2          PB15             方向 2
 *    RL(2)       PWMA          PB8 (TIM4_CH3)   速度 PWM
 *    RR(3)       BIN1          PB12             方向 1
 *    RR(3)       BIN2          PB13             方向 2
 *    RR(3)       PWMB          PB9 (TIM4_CH4)   速度 PWM
 *    ---------------------------------------------------------------
 *    两片 TB6612 的 STBY 都接高电平(3.3V/5V)使能。
 *    TB6612 逻辑电源 VCC->3.3V，GND 共地；电机电源 VM->7~12V。
 *
 *  ----------------------------------------------------------------------
 *  麦轮安装（标准"X"型，装反了运动方向会错）
 *    左前(FL) + 右后(RR) 的辊子朝同一个 45° 方向；
 *    右前(FR) + 左后(RL) 的辊子朝另一个 45° 方向。
 *    若实际发现某个方向反了，把 chassis.c 里的 s_dir_pol[] 对应项改成 -1 即可，
 *    不用重新接线。
 * ========================================================================== */

/** 底盘电机数量（麦轮四驱） */
#define CHASSIS_MOTOR_COUNT     4u

/** 电机 PWM 定时器：TIM4（TIM3 已被机械臂 1ms 基准占用，不能共用） */
#define CHASSIS_PWM_TIM         TIM4

/** PWM 定时器输入时钟频率(Hz)。
 *  TIM4 挂在 APB1 上，APB1 二分频后定时器时钟被自动倍频回 72MHz，
 *  与 TIM2/TIM3 完全一致。 */
#define CHASSIS_TIM_CLK_HZ      72000000u

/** 电机 PWM 频率(Hz)。TB6612 可听声以外的频率都行；1000Hz 常见且平顺。
 *  ★ 注意：原底盘工程用的是 100Hz（TIM_Init(TIM3,10000,72)），
 *    100Hz 会有明显的低频顿挫与噪声。这里提到 1kHz，电机运转更平顺。 */
#define CHASSIS_PWM_FREQ_HZ     1000u

/** 预分频：72MHz / 72 = 1MHz，1 计数 = 1us（与原 TIM3 方案一致，直观） */
#define CHASSIS_PWM_PSC         ((uint16_t)(CHASSIS_TIM_CLK_HZ / 1000000u - 1u))

/** 自动重装载值：1MHz / 1000Hz = 1000 计数 */
#define CHASSIS_PWM_ARR         ((uint16_t)(1000000u / CHASSIS_PWM_FREQ_HZ - 1u))

/** PWM 满速比较值上限。占空比 = pwm_val / (ARR+1)。
 *  取 ARR 本身（1000）即 100% 占空比。 */
#define CHASSIS_PWM_MAX         ((uint16_t)(CHASSIS_PWM_ARR + 1u))

/** 上电默认速度上限（百分比 0~100）。运行时可用 $MC 调。 */
#define CHASSIS_SPEED_PCT_DEF   60u

/* ---- 融合版：双工作模式 + 失联自停（取自文件夹 1）---- */
/** AUTO：PC 经 USART1 发 $MV 自动对列；MANUAL：手机 HC-05 单字符控车 */
#define CHASSIS_DEADMAN_AUTO_MS     800u
#define CHASSIS_DEADMAN_MANUAL_MS   3000u
/** HC-05 波特率 */
#define CHASSIS_BT_BAUD             9600u

/** 底盘方向脚：4 个电机 x 2 个 IN 脚 */
#define CHASSIS_DIR_PORT_FL     GPIOA
#define CHASSIS_DIR_IN1_FL      GPIO_Pin_4      /* PA4 */
#define CHASSIS_DIR_IN2_FL      GPIO_Pin_5      /* PA5 */

#define CHASSIS_DIR_PORT_FR     GPIOA
#define CHASSIS_DIR_IN1_FR      GPIO_Pin_6      /* PA6 */
#define CHASSIS_DIR_IN2_FR      GPIO_Pin_7      /* PA7 */

#define CHASSIS_DIR_PORT_RL     GPIOB
#define CHASSIS_DIR_IN1_RL      GPIO_Pin_14     /* PB14（★ 曾用 PB0，易与旧工程冲突） */
#define CHASSIS_DIR_IN2_RL      GPIO_Pin_15     /* PB15（★ 曾用 PB1） */

#define CHASSIS_DIR_PORT_RR     GPIOB
#define CHASSIS_DIR_IN1_RR      GPIO_Pin_12     /* PB12 */
#define CHASSIS_DIR_IN2_RR      GPIO_Pin_13     /* PB13 */

/** 底盘运动超时上限(ms)：防止 PC 端误发超大值导致轮子长时间狂转 */
#define CHASSIS_MAX_MOVE_MS     30000u

/* ==========================================================================
 * 十一、HC05 蓝牙串口（USART3）
 *
 *  为什么用 USART3 而不是 USART2：
 *    USART2 固定占 PA2/PA3，而 PA2/PA3 是舵机 s2/s3 的 TIM2_CH3/CH4 输出，
 *    用它就得把整个舵机 PWM 迁到 TIM4 并重新标定。USART3 的 PB10/PB11
 *    在本工程里完全空闲，用它可以让舵机【一根线都不动】。
 *
 *  物理连接：
 *    STM32 PB10 (USART3_TX) -> HC05 的 RXD
 *    STM32 PB11 (USART3_RX) <- HC05 的 TXD
 *    HC05 VCC->3.3V/5V，GND 共地
 *
 *  与 USART1(PC) 完全独立：手机蓝牙和 PC 可以同时在线，互不干扰。
 *  这一点对 manual_phone 模式尤其重要 —— 那时正是"PC 在跑流程、
 *  人用手机操控底盘"，两路串口都必须活着。
 * ========================================================================== */

/** HC05 波特率。HC05 出厂/AT 默认多为 9600，与原底盘工程一致。 */
#define USART3_BAUDRATE         9600u

/** USART3 接收 FIFO 大小（必须是 2 的幂）。手机发的是单字节命令，64 足够。 */
#define HC05_FIFO_SIZE          64u
#define HC05_FIFO_MASK          (HC05_FIFO_SIZE - 1u)

/* ==========================================================================
 * 十二、其它硬件
 * ========================================================================== */

/** 板载 LED（Blue Pill 的 PC13，低电平点亮，只能灌电流） */
#define LED_GPIO_PORT           GPIOC
#define LED_GPIO_PIN            GPIO_Pin_13
#define LED_ON                  0u          /* 低电平点亮 */
#define LED_OFF                 1u          /* 高电平熄灭 */
#define LED_GPIO_CLK_ENABLE()   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE)

/** 独立看门狗开关。
 *  1 = 启用（超时约 2s，主循环中喂狗）；0 = 关闭。
 *  !! 默认关闭 !! —— 本工程目标是 Keil 仿真调试：仿真/单步调试时若看门狗
 *  开着，2 秒后会自动复位，无法正常打断点。真机使用请改回 1。 */
#define USE_IWDG                0u

/** 看门狗超时时间(ms)，IWDG 时钟 40kHz / 64 = 625Hz */
#define IWDG_TIMEOUT_MS         2000u
#define IWDG_PRESCALER          IWDG_Prescaler_64
#define IWDG_RELOAD             ((uint16_t)(IWDG_TIMEOUT_MS * 625u / 1000u))

/* ==========================================================================
 * 十三、跨文件调用的函数声明
 * ========================================================================== */

/** 系统时钟配置：SPL 版依赖启动文件中的 SystemInit()（HSE 8M * PLL9 = 72MHz），
 *  本函数只负责刷新 SystemCoreClock 变量。 */
void SystemClock_Config(void);

/** 错误处理：初始化失败时 LED 快速闪烁并停在此处 */
void Error_Handler(void);

/** 串口发送（阻塞式），由 protocol.c 调用，实现在 main.c（封装 my_lib/usart.c） */
void App_Uart_Send(const uint8_t *data, uint16_t len);

/** USART1 IDLE 中断的"业务处理"入口（USE_RX_DMA=1 时由 stm32f10x_it.c 调用，
 *  同时主循环周期性调用作为兜底）。 */
void USER_UART_IdleHandler(void);

/** USART1 发生 IDLE 中断时的完整处理入口（清标志 + 搬运数据） */
void USER_UART_IdleIrqHandler(void);

/** 把一字节压入接收 FIFO（USE_RX_DMA=0 时由 RXNE 中断调用） */
void UART_RxBytePush(uint8_t b);

/** 串口错误恢复：清错误标志、复位 FIFO 与解析状态（ORE/FE/NE 时调用） */
void UART_RxErrorRecover(void);

/* ---------- 底盘 / HC05 相关（实现见 user/main.c、app/chassis.c） ---------- */

/** 把一字节压入 HC05(USART3) 接收 FIFO（USART3 RXNE 中断里调用） */
void HC05_RxBytePush(uint8_t b);

/** 从 HC05 FIFO 取出一个字节（主循环调用），无数据返回 0 */
uint8_t HC05_FifoPop(uint8_t *b);

/** USART3 错误恢复（ORE/FE/NE 时调用） */
void HC05_RxErrorRecover(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
