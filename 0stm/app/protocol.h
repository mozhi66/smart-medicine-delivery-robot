/**
  ******************************************************************************
  * @file    protocol.h
  * @brief   智能药箱上位机<->下位机通信协议（NMEA 风格 ASCII 帧）
  *
  * 帧格式：
  *     $<CMD>[,<arg1>,<arg2>,...]*<CS>\r\n
  *
  *   '$'   帧头
  *   CMD   指令名（大写），其后可跟若干以 ',' 分隔的参数
  *   '*'   校验和分隔符
  *   CS    校验和：从 '$' 之后到 '*' 之前"所有字符"的按字节 XOR，
  *         以 2 位大写十六进制表示，例如 *5E
  *   \r\n  帧尾
  *
  * 示例：  $SA,90,75,60,40,800*5E\r\n
  *
  * 设计原则：
  *   1) 解析采用"字节流状态机"，以 \r\n（或单独 \n）作为帧边界，
  *      不依赖 DMA 一次收到完整帧；
  *   2) 校验失败一律丢弃，绝不"容错执行"，并上报 $ERR,01；
  *   3) 本模块只负责"帧的拆装与校验"，不涉及任何业务动作，
  *      业务由 main.c 的指令处理函数完成。
  *
  * @author  智能药箱毕业设计（Keil SPL 移植版）
  * @version V1.0
  ******************************************************************************
  */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

/* ==========================================================================
 * 一、常量定义
 * ========================================================================== */

/** 发送缓冲区大小。
 *  需要容纳：$ + 正文 + '*' + 2 位校验 + \r\n + '\0'。
 *  $POSE,180.0,180.0,180.0,180.0*5E\r\n 约 40 字节，160 绰绰有余。 */
#define PROTO_TX_BUF_SIZE       160u

/** 指令名字段最大长度（不含结束符）。目前最长为 "READY"/"POSE" 等 5 字符，
 *  留 8 字节余量。 */
#define PROTO_CMD_NAME_MAX      8u

/** 单个参数字符串最大长度（含结束符）。角度最多 "-180.0" = 7 字符，
 *  时长最多 "60000" = 6 字符，取 20 足够。 */
#define PROTO_ARG_MAX           20u

/** 最大参数个数（含指令名本身）。$SA 有 5 个参数 + 指令名 = 6，取 8。 */
#define PROTO_ARG_COUNT_MAX     8u

/* ==========================================================================
 * 二、字节流喂入结果
 * ========================================================================== */

/**
 * @brief PROTO_FeedByte() 的返回值
 */
typedef enum
{
    PROTO_FEED_CONT    = 0u, /**< 继续接收，尚未凑齐一帧 */
    PROTO_FEED_OK      = 1u, /**< 收到一帧且校验通过 */
    PROTO_FEED_CS_ERR  = 2u, /**< 收到一帧但校验失败 -> 上报 $ERR,01 */
    PROTO_FEED_FMT_ERR = 3u  /**< 帧格式错误（空帧/缺 '*'/超长/HEX 非法）-> 上报 $ERR,02 */
} ProtoFeedResult_t;

/* ==========================================================================
 * 三、指令枚举（PC -> STM32）
 * ========================================================================== */

/**
 * @brief PC 下发指令编号
 * @note  枚举值仅用于内部 switch 分支；上报帧里用的是 PROTO_CmdName() 返回的
 *        字符串，保证与协议文本完全一致。
 */
typedef enum
{
    PROTO_CMD_SA  = 0,  /**< SA  : $SA,a1,a2,a3,a4,tt   四路舵机同步插值运动 */
    PROTO_CMD_SS  = 1,  /**< SS  : $SS,id,ang,tt        单舵机运动（调试用） */
    PROTO_CMD_GC  = 2,  /**< GC  : $GC,open|close       夹爪开/合 */
    PROTO_CMD_RUN = 3,  /**< RUN : $RUN,slot_id         执行 Flash 中预存动作序列 */
    PROTO_CMD_HOM = 4,  /**< HOM : $HOM                 回 HOME 位 */
    PROTO_CMD_QRY = 5,  /**< QRY : $QRY,POSE            查询当前四路角度 */
    PROTO_CMD_EST = 6,  /**< EST : $EST                 急停，冻结在当前位置 */
    PROTO_CMD_CFG = 7,  /**< CFG : $CFG,id,min,max      配置某舵机限位并写入 Flash */

    /* ---- 麦轮底盘（融合版：1 的命名 + 0 的定时运动，见 app/chassis.c）----
     * ★ 语义速记：MS=Stop 停车，MC=Cap 限速，MQ=Query 查询，MD=Mode 模式。
     *   不再使用易记反的 MVS/MVC。 */
    PROTO_CMD_MD  = 8,  /**< MD  : $MD,AUTO|MAN|Q       工作模式；Q 只查询 */
    PROTO_CMD_MV  = 9,  /**< MV  : $MV,y,x,rx[,tt]      麦轮运动 y/x/rx∈[-1,1]；tt=ms（省略/0=连续） */
    PROTO_CMD_MS  = 10, /**< MS  : $MS                  底盘立即停车 */
    PROTO_CMD_MC  = 11, /**< MC  : $MC,pct              设定速度上限百分比(0~100) */
    PROTO_CMD_MQ  = 12, /**< MQ  : $MQ                  查询底盘 -> $CHS,mode,moving,y,x,rx,pct */

    PROTO_CMD_NONE = 0xFF /**< 未识别的指令 */
} ProtoCmd_t;

/* ==========================================================================
 * 四、错误码定义（$ERR,code,msg）
 * ========================================================================== */

/**
 * @brief 错误码，协议规定为 2 位十进制（01~05）
 */
typedef enum
{
    PROTO_ERR_CS      = 1u, /**< 01 校验失败（XOR 不匹配或校验位不是合法 HEX） */
    PROTO_ERR_UNKNOWN = 2u, /**< 02 未知指令 / 参数格式错误 / 帧过长 */
    PROTO_ERR_LIMIT   = 3u, /**< 03 角度越限（超过该舵机 min/max 软限位或硬件绝对限位） */
    PROTO_ERR_TIMEOUT = 4u, /**< 04 执行超时 */
    PROTO_ERR_SERVO   = 5u, /**< 05 舵机异常/堵转（连续多次超时判定） */
    PROTO_ERR_LOCKED  = 6u  /**< 06 底盘运动中锁臂 / 机械臂忙锁底盘（融合自文件夹 1） */
} ProtoErr_t;

/* ==========================================================================
 * 五、数据结构
 * ========================================================================== */

/**
 * @brief 一帧已解析完成的报文
 */
typedef struct
{
    ProtoCmd_t  cmd;                    /**< 指令编号，未知时为 PROTO_CMD_NONE */
    char        payload[PROTO_LINE_MAX];/**< 帧正文：不含 '$'、'*'、校验和、CRLF，
                                             以 '\0' 结尾，例如 "SA,90,75,60,40,800" */
    uint16_t    len;                    /**< 正文长度（不含结束符） */
} ProtoFrame_t;

/* ==========================================================================
 * 六、对外函数
 * ========================================================================== */

/* ---------- 6.1 初始化与字节流解析 ---------- */

/**
 * @brief  协议模块初始化（清空字节流状态机内部缓冲）
 * @param  无
 * @retval 无
 */
void PROTO_Init(void);

/**
 * @brief  向解析状态机喂入一个字节
 * @note   由主循环从软件 FIFO 中逐字节取出后调用。
 *          '$' 之前的所有字节都会被丢弃（用于重新同步帧边界）；
 *          遇到 '\n' 认为一帧结束，随后做 XOR 校验。
 * @param  b    接收到的一个字节
 * @param  out  当返回 PROTO_FEED_OK 时，这里存放解析好的完整帧
 * @retval PROTO_FEED_CONT(0)    继续接收，尚未凑齐一帧
 *         PROTO_FEED_OK(1)      收到一帧且校验通过，内容在 *out
 *         PROTO_FEED_CS_ERR(2)  收到一帧但校验失败（应上报 $ERR,01）
 *         PROTO_FEED_FMT_ERR(3) 帧格式错误：空帧 / 缺 '*' / 超长 / HEX 非法
 *                               （应上报 $ERR,02）
 */
uint8_t PROTO_FeedByte(uint8_t b, ProtoFrame_t *out);

/* ---------- 6.2 校验与帧构建 ---------- */

/**
 * @brief  计算一段正文的 XOR 校验值
 * @param  payload 正文首地址（'$' 之后、'*' 之前的内容，不需要 '\0' 结尾）
 * @param  len     正文长度
 * @retval 按字节 XOR 的结果（0x00~0xFF）
 */
uint8_t PROTO_XorCheck(const char *payload, uint16_t len);

/**
 * @brief  根据正文构建完整帧（含 '$'、'*'、2 位大写 HEX 校验、CRLF）
 * @param  payload   正文字符串，以 '\0' 结尾
 * @param  payload_len 正文长度（可为 0xFFFF 表示自动按 strlen 计算，
 *                     这里不使用 strlen，调用方必须给出长度）
 * @param  out       输出缓冲区
 * @param  out_size  输出缓冲区大小
 * @retval 成功返回帧字节数（不含字符串结束符）；空间不足返回 0
 */
uint16_t PROTO_BuildFrame(const char *payload, uint16_t payload_len,
                          char *out, uint16_t out_size);

/* ---------- 6.3 发送 ---------- */

/**
 * @brief  发送一段正文（自动加帧头/校验/帧尾）
 * @param  payload       以 '\0' 结尾的正文
 * @param  payload_len   正文长度（不含结束符）
 * @retval 无
 * @note   发送使用阻塞式串口发送（App_Uart_Send -> my_lib/usart.c 的
 *         My_USART_SendBytes）。原因是：
 *           - 上报帧都很短（<=40 字节，115200bps 下约 3ms）；
 *           - 阻塞发送天然不会与自身产生 DMA 竞争，实现最简单可靠；
 *           - 发送期间 RX 仍由 DMA/中断后台填充环形缓冲，主循环结束后再取，
 *             不会丢数据（RX FIFO 有 512 字节余量）。
 */
void PROTO_SendPayload(const char *payload, uint16_t payload_len);

/**
 * @brief  上电就绪帧：  $READY,VER1.0*CS
 * @param  无
 * @retval 无
 */
void PROTO_SendReady(void);

/**
 * @brief  指令确认帧：  $ACK,<cmd>*CS
 * @param  cmd 指令名字符串，如 "SA"（不带 '$'）
 * @retval 无
 */
void PROTO_SendAck(const char *cmd);

/**
 * @brief  执行完成帧：  $DONE,<cmd>*CS
 * @param  cmd 指令名字符串，如 "SA"
 * @retval 无
 */
void PROTO_SendDone(const char *cmd);

/**
 * @brief  位姿上报帧：  $POSE,a1,a2,a3,a4*CS
 * @param  ang 长度为 SERVO_COUNT 的角度数组（单位：度，上报保留 1 位小数）
 * @retval 无
 */
void PROTO_SendPose(const float *ang);

/**
 * @brief  错误帧：      $ERR,code,msg*CS
 * @param  code 错误码（1~5，见 ProtoErr_t，输出为 2 位十进制）
 * @param  msg  简短错误说明（纯大写字母，不含逗号），可为 NULL
 * @retval 无
 */
void PROTO_SendErr(uint8_t code, const char *msg);

/**
 * @brief  心跳帧：      $HB,cnt*CS
 * @param  cnt 递增计数（0~65535 回绕）
 * @retval 无
 */
void PROTO_SendHb(uint16_t cnt);

/**
 * @brief  底盘状态帧：  $CHS,mode,moving,y,x,rx,pct*CS
 * @param  mode    0=MAN  1=AUTO（帧内 'M'/'A'）
 * @param  moving  是否正在运动（帧内 '1'/'0'）
 * @param  y_pct   前后分量 -100..100
 * @param  x_pct   横移分量 -100..100
 * @param  rx_pct  自转分量 -100..100
 * @param  pct     速度上限百分比 0~100
 * @retval 无
 * @note   供 PC 在 auto 模式下发底盘指令后确认"到底动没动"，
 *         也便于 manual_phone 模式判断人工是否还在推杆。
 *         ★ 底盘运动帧只回 $ACK + $CHS，**不回 $DONE**。
 */
void PROTO_SendChassis(uint8_t mode, uint8_t moving,
                       float y_pct, float x_pct, float rx_pct,
                       uint8_t pct);

/* ---------- 6.4 文本工具 ---------- */

/**
 * @brief  从正文中取出指令名（第一个 ',' 之前的部分）
 * @param  payload 正文
 * @param  cmd     输出：指令编号，无法识别时为 PROTO_CMD_NONE
 * @retval 1 识别成功；0 无法识别
 * @note   指令名比较不区分大小写，容错 PC 端写出 "Sa"、"sa"。
 */
uint8_t PROTO_ParseCmd(const char *payload, ProtoCmd_t *cmd);

/**
 * @brief  由指令编号得到协议中使用的指令名字符串（大写）
 * @param  cmd 指令编号
 * @retval 指令名字符串；未知指令返回 "UNK"
 */
const char *PROTO_CmdName(ProtoCmd_t cmd);

/**
 * @brief  取正文中的第 n 个字段
 * @param  payload  正文
 * @param  index    0 = 指令名，1 = 第 1 个参数，以此类推
 * @param  out      输出字段（会补 '\0'）
 * @param  out_size 输出缓冲区大小
 * @retval 1 取到；0 该字段不存在或缓冲区太小
 */
uint8_t PROTO_GetArg(const char *payload, uint8_t index,
                     char *out, uint16_t out_size);

/**
 * @brief  统计正文中的字段个数（含指令名）
 * @param  payload 正文
 * @retval 字段个数（至少为 1）
 */
uint8_t PROTO_ArgCount(const char *payload);

/**
 * @brief  字符串转整数（支持可选正负号，不支持科学计数法）
 * @param  s 字符串
 * @retval 解析结果；非法输入返回 0
 */
int32_t PROTO_ArgToInt(const char *s);

/**
 * @brief  字符串转浮点数（支持可选正负号与一位以上小数）
 * @param  s 字符串
 * @retval 解析结果；非法输入返回 0.0f
 */
float PROTO_ArgToFloat(const char *s);

/**
 * @brief  忽略大小写的字符串比较
 * @param  a 字符串 A
 * @param  b 字符串 B
 * @retval 1 相等；0 不相等
 */
uint8_t PROTO_StrCaseEq(const char *a, const char *b);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H */
