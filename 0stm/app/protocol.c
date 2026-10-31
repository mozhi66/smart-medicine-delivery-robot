/**
  ******************************************************************************
  * @file    protocol.c
  * @brief   NMEA 风格 ASCII 通信协议的解析与帧构建实现（Keil SPL 移植版）
  *
  * 实现思路：
  *   1) 解析：一个"字节流状态机"。'$' 之前全部丢弃；收到 '$' 后开始把字符
  *      攒进行缓冲；遇到 '\n' 认为一帧结束（兼容 "\r\n" 与裸 "\n"）；
  *      然后检查倒数第 3 个字符是否是 '*'，取出后面 2 位 HEX 作为接收校验，
  *      与本地对正文计算的 XOR 比较。不同则整帧丢弃。
  *   2) 构建：把正文拼成 "正文*XX\r\n" 再加上 '$'，XOR 只覆盖正文。
  *   3) 发送：统一走 PROTO_SendPayload()，用阻塞串口发送
  *      （App_Uart_Send -> my_lib/usart.c 的 My_USART_SendBytes）。
  *
  * 数值格式化不使用 sprintf/snprintf：
  *   - 避免引入较大的 printf 家族代码（F103C8 只有 64KB Flash）；
  *   - 浮点输出只用到"保留 1 位小数"，自己写更快也更可控。
  *
  * @author  智能药箱毕业设计（Keil SPL 移植版）
  * @version V1.0
  ******************************************************************************
  */

#include "protocol.h"
#include "string.h"

/* ==========================================================================
 * 内部静态变量：字节流解析状态机
 * ========================================================================== */

static char     s_line[PROTO_LINE_MAX];  /**< 行缓冲：暂存 '$' 之后的内容 */
static uint16_t s_line_len;              /**< 行缓冲当前长度 */
static uint8_t  s_synced;                /**< 1 = 已收到 '$'，正在收帧 */
static uint8_t  s_overflow;              /**< 1 = 本帧超长，剩余字节全部丢弃 */

/* 指令名表：顺序必须与 ProtoCmd_t 枚举一致 */
static const char *const s_cmd_names[] =
{
    "SA", "SS", "GC", "RUN", "HOM", "QRY", "EST", "CFG",
    "MD", "MV", "MS", "MC", "MQ"
};
#define CMD_NAME_COUNT  (sizeof(s_cmd_names) / sizeof(s_cmd_names[0]))

/* ==========================================================================
 * 内部工具函数
 * ========================================================================== */

/**
 * @brief  把一个半字节(0~15)转成大写十六进制字符
 * @param  v 0~15
 * @retval '0'~'9' 或 'A'~'F'
 */
static char HexNibbleToChar(uint8_t v)
{
    return (v < 10u) ? (char)('0' + v) : (char)('A' + (v - 10u));
}

/**
 * @brief  把一个十六进制字符转成数值
 * @param  c 字符
 * @retval 0~15；非法字符返回 0xFF
 */
static uint8_t HexCharToVal(char c)
{
    if ((c >= '0') && (c <= '9')) { return (uint8_t)(c - '0'); }
    if ((c >= 'A') && (c <= 'F')) { return (uint8_t)(c - 'A' + 10u); }
    if ((c >= 'a') && (c <= 'f')) { return (uint8_t)(c - 'a' + 10u); }
    return 0xFFu;
}

/**
 * @brief  向缓冲区尾部追加一个字符串
 * @param  dst 目标缓冲区
 * @param  pos 当前写入位置（会被更新并返回）
 * @param  s   源字符串（以 '\0' 结尾）
 * @retval 新的写入位置
 * @note   调用方负责保证目标缓冲区足够大（本文件内所有调用点都已核算）
 */
static uint16_t ApStr(char *dst, uint16_t pos, const char *s)
{
    while (*s != '\0')
    {
        dst[pos++] = *s++;
    }
    return pos;
}

/**
 * @brief  向缓冲区尾部追加一个无符号十进制整数（不补零）
 * @param  dst 目标缓冲区
 * @param  pos 当前写入位置
 * @param  v   待写入数值
 * @retval 新的写入位置
 */
static uint16_t ApU32(char *dst, uint16_t pos, uint32_t v)
{
    char tmp[12];
    uint8_t n = 0u;

    if (v == 0u)
    {
        dst[pos++] = '0';
        return pos;
    }
    while (v > 0u)
    {
        tmp[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    while (n > 0u)
    {
        dst[pos++] = tmp[--n];
    }
    return pos;
}

/**
 * @brief  向缓冲区尾部追加一个"保留 1 位小数"的浮点数
 * @param  dst 目标缓冲区
 * @param  pos 当前写入位置
 * @param  v   待写入数值
 * @retval 新的写入位置
 * @note   手工做四舍五入：先取整数部分，再把小数部分 *10 后四舍五入，
 *         若产生进位（例如 9.99 -> 10.0）则整数部分 +1。
 *         支持 0 位前置补零（如 0.5 输出 "0.5"）。
 */
static uint16_t ApFloat1(char *dst, uint16_t pos, float v)
{
    uint32_t ipart;
    uint32_t fpart;
    float    absv;

    if (v < 0.0f)
    {
        dst[pos++] = '-';
        absv = -v;
    }
    else
    {
        absv = v;
    }

    ipart = (uint32_t)absv;
    fpart = (uint32_t)((absv - (float)ipart) * 10.0f + 0.5f);
    if (fpart >= 10u)   /* 四舍五入进位 */
    {
        fpart = 0u;
        ipart++;
    }

    pos = ApU32(dst, pos, ipart);
    dst[pos++] = '.';
    dst[pos++] = (char)('0' + fpart);
    return pos;
}

/* ==========================================================================
 * 一、解析状态机
 * ========================================================================== */

/**
 * @brief  协议模块初始化（清空字节流状态机内部缓冲）
 * @note   实现：把行缓冲长度清零、清同步标志与溢出标志。
 *         开机调用一次即可，之后除非通信异常需要强制重新同步，不必再次调用。
 * @param  无
 * @retval 无
 */
void PROTO_Init(void)
{
    s_line_len = 0u;
    s_synced   = 0u;
    s_overflow = 0u;
    s_line[0]  = '\0';
}

/**
 * @brief  向解析状态机喂入一个字节
 * @note   实现要点：
 *           - 未同步（还没收到 '$'）时丢弃一切字节，用于自动重新同步帧边界；
 *           - 帧中途又收到 '$' 说明上一帧残缺，直接丢弃旧内容重新开始，
 *             避免"粘包"造成解析错位；
 *           - 超长帧不立即报错，而是继续吞掉直到 '\n'，然后报 FMT_ERR，
 *             这样不会把超长帧的尾巴误当成新帧；
 *           - 结束判定：收到 '\n' 即认为帧结束（'\r' 只做记录、不参与判定，
 *             因此 "\r\n" 与裸 "\n" 都能正确处理）。
 * @param  b    接收到的一个字节
 * @param  out  当返回 PROTO_FEED_OK 时，这里存放解析好的完整帧
 * @retval 见头文件中 PROTO_FeedByte 的说明
 */
uint8_t PROTO_FeedByte(uint8_t b, ProtoFrame_t *out)
{
    uint16_t payload_len;
    uint8_t  hi, lo, cs_recv, cs_calc;

    /* ---- 1. 尚未同步：等待帧头 '$' ---- */
    if (s_synced == 0u)
    {
        if (b == (uint8_t)'$')
        {
            s_synced   = 1u;
            s_line_len = 0u;
            s_overflow = 0u;
        }
        return PROTO_FEED_CONT;
    }

    /* ---- 2. 本帧已超长：吞掉直到帧尾 ---- */
    if (s_overflow != 0u)
    {
        if (b == (uint8_t)'\n')
        {
            s_synced = 0u;
            return PROTO_FEED_FMT_ERR;
        }
        return PROTO_FEED_CONT;
    }

    /* ---- 3. 帧尾判定 ---- */
    if (b == (uint8_t)'\r')
    {
        /* '\r' 不结束帧，等 '\n'；单独出现也不会破坏解析 */
        return PROTO_FEED_CONT;
    }

    if (b == (uint8_t)'\n')
    {
        s_synced = 0u;   /* 无论成功与否，帧都结束了，回到未同步态 */

        if (s_line_len < 4u)     /* 最短合法帧："A*00" 也要 4 个字符 */
        {
            return PROTO_FEED_FMT_ERR;
        }
        if (s_line[s_line_len - 3u] != '*')
        {
            return PROTO_FEED_FMT_ERR;
        }

        payload_len = (uint16_t)(s_line_len - 3u);

        hi = HexCharToVal(s_line[s_line_len - 2u]);
        lo = HexCharToVal(s_line[s_line_len - 1u]);
        if ((hi == 0xFFu) || (lo == 0xFFu))
        {
            return PROTO_FEED_CS_ERR;   /* 校验位不是合法 HEX，按校验失败处理 */
        }
        cs_recv = (uint8_t)((hi << 4) | lo);

        cs_calc = PROTO_XorCheck(s_line, payload_len);
        if (cs_calc != cs_recv)
        {
            return PROTO_FEED_CS_ERR;
        }

        /* ---- 校验通过，交给上层 ---- */
        if (out != NULL)
        {
            if (payload_len >= PROTO_LINE_MAX)
            {
                payload_len = (uint16_t)(PROTO_LINE_MAX - 1u);
            }
            memcpy(out->payload, s_line, payload_len);
            out->payload[payload_len] = '\0';
            out->len = payload_len;
            (void)PROTO_ParseCmd(out->payload, &out->cmd);
        }
        return PROTO_FEED_OK;
    }

    /* ---- 4. 帧中又出现 '$'：上一帧残缺，重新开始 ---- */
    if (b == (uint8_t)'$')
    {
        s_line_len = 0u;
        return PROTO_FEED_CONT;
    }

    /* ---- 5. 普通字符入缓冲 ---- */
    if (s_line_len >= (uint16_t)(PROTO_LINE_MAX - 1u))
    {
        s_overflow = 1u;    /* 保留最后一字节给结束符 */
        return PROTO_FEED_CONT;
    }
    s_line[s_line_len++] = (char)b;

    return PROTO_FEED_CONT;
}

/* ==========================================================================
 * 二、校验与帧构建
 * ========================================================================== */

/**
 * @brief  计算一段正文的 XOR 校验值
 * @note   实现：逐字节异或。校验范围是 '$' 之后到 '*' 之前的所有字符，
 *         不含 '$'、'*'、校验和本身与 CRLF —— 与 PC 端实现严格一致。
 * @param  payload 正文首地址
 * @param  len     正文长度
 * @retval XOR 结果
 */
uint8_t PROTO_XorCheck(const char *payload, uint16_t len)
{
    uint8_t  cs = 0u;
    uint16_t i;

    for (i = 0u; i < len; i++)
    {
        cs ^= (uint8_t)payload[i];
    }
    return cs;
}

/**
 * @brief  根据正文构建完整帧（含 '$'、'*'、2 位大写 HEX 校验、CRLF）
 * @note   实现：先写 '$'，再逐字节写正文并同时累加 XOR，最后追加
 *         '*' + 2 位 HEX + "\r\n" + '\0'。
 *         每一步都检查剩余空间，空间不足则直接返回 0，绝不越界写。
 * @param  payload     正文字符串（不需要 '\0' 结尾）
 * @param  payload_len 正文长度
 * @param  out         输出缓冲区
 * @param  out_size    输出缓冲区大小
 * @retval 帧字节数（不含结束符）；失败返回 0
 */
uint16_t PROTO_BuildFrame(const char *payload, uint16_t payload_len,
                          char *out, uint16_t out_size)
{
    uint16_t n = 0u;
    uint16_t i;
    uint8_t  cs = 0u;

    if ((out == NULL) || (payload == NULL))
    {
        return 0u;
    }
    if (out_size < (uint16_t)(payload_len + 6u))   /* $ + 正文 + *XX + CR + LF + '\0' */
    {
        return 0u;
    }

    out[n++] = '$';

    for (i = 0u; i < payload_len; i++)
    {
        out[n++] = payload[i];
        cs ^= (uint8_t)payload[i];
    }

    out[n++] = '*';
    out[n++] = HexNibbleToChar((uint8_t)((cs >> 4) & 0x0Fu));
    out[n++] = HexNibbleToChar((uint8_t)(cs & 0x0Fu));
    out[n++] = '\r';
    out[n++] = '\n';
    out[n]   = '\0';

    return n;
}

/* ==========================================================================
 * 三、发送
 * ========================================================================== */

/**
 * @brief  发送一段正文（自动加帧头/校验/帧尾）
 * @note   实现：先在栈上拼出完整帧（长度上限 PROTO_TX_BUF_SIZE），
 *         再用阻塞式串口发送（App_Uart_Send）。
 *         若正文过长导致构建失败，则直接放弃这一帧（宁可不发错帧）。
 * @param  payload     正文
 * @param  payload_len 正文长度
 * @retval 无
 */
void PROTO_SendPayload(const char *payload, uint16_t payload_len)
{
    char     frame[PROTO_TX_BUF_SIZE];
    uint16_t n;

    if (payload == NULL)
    {
        return;
    }

    n = PROTO_BuildFrame(payload, payload_len, frame, (uint16_t)sizeof(frame));
    if (n == 0u)
    {
        return;
    }

    App_Uart_Send((const uint8_t *)frame, n);
}

/**
 * @brief  上电就绪帧：$READY,VER1.0*CS
 * @note   实现：正文 = "READY," + FW_VERSION_STRING。
 *         PC 端应等待收到本帧后再下发业务指令。
 * @param  无
 * @retval 无
 */
void PROTO_SendReady(void)
{
    char     pay[PROTO_TX_BUF_SIZE];
    uint16_t n = 0u;

    n = ApStr(pay, n, "READY,");
    n = ApStr(pay, n, FW_VERSION_STRING);

    PROTO_SendPayload(pay, n);
}

/**
 * @brief  指令确认帧：$ACK,<cmd>*CS
 * @note   含义：指令已收到、校验通过、已开始执行（注意不是"执行完毕"）。
 * @param  cmd 指令名字符串
 * @retval 无
 */
void PROTO_SendAck(const char *cmd)
{
    char     pay[PROTO_TX_BUF_SIZE];
    uint16_t n = 0u;

    n = ApStr(pay, n, "ACK,");
    n = ApStr(pay, n, (cmd != NULL) ? cmd : "UNK");

    PROTO_SendPayload(pay, n);
}

/**
 * @brief  执行完成帧：$DONE,<cmd>*CS
 * @param  cmd 指令名字符串
 * @retval 无
 */
void PROTO_SendDone(const char *cmd)
{
    char     pay[PROTO_TX_BUF_SIZE];
    uint16_t n = 0u;

    n = ApStr(pay, n, "DONE,");
    n = ApStr(pay, n, (cmd != NULL) ? cmd : "UNK");

    PROTO_SendPayload(pay, n);
}

/**
 * @brief  位姿上报帧：$POSE,a1,a2,a3,a4*CS
 * @note   角度保留 1 位小数，四个数值以 ',' 分隔。
 * @param  ang 长度为 SERVO_COUNT 的角度数组
 * @retval 无
 */
void PROTO_SendPose(const float *ang)
{
    char     pay[PROTO_TX_BUF_SIZE];
    uint16_t n = 0u;
    uint8_t  i;

    if (ang == NULL)
    {
        return;
    }

    n = ApStr(pay, n, "POSE,");
    for (i = 0u; i < SERVO_COUNT; i++)
    {
        if (i > 0u)
        {
            n = ApStr(pay, n, ",");
        }
        n = ApFloat1(pay, n, ang[i]);
    }

    PROTO_SendPayload(pay, n);
}

/**
 * @brief  错误帧：$ERR,code,msg*CS
 * @note   错误码固定输出 2 位十进制（不足补 '0'），与协议表一致。
 * @param  code 1~5
 * @param  msg  简短说明，可为 NULL
 * @retval 无
 */
void PROTO_SendErr(uint8_t code, const char *msg)
{
    char     pay[PROTO_TX_BUF_SIZE];
    uint16_t n = 0u;

    n = ApStr(pay, n, "ERR,");
    if (code < 10u)
    {
        pay[n++] = '0';
    }
    n = ApU32(pay, n, (uint32_t)code);
    n = ApStr(pay, n, ",");
    n = ApStr(pay, n, (msg != NULL) ? msg : "");

    PROTO_SendPayload(pay, n);
}

/**
 * @brief  心跳帧：$HB,cnt*CS
 * @param  cnt 递增计数
 * @retval 无
 */
void PROTO_SendHb(uint16_t cnt)
{
    char     pay[PROTO_TX_BUF_SIZE];
    uint16_t n = 0u;

    n = ApStr(pay, n, "HB,");
    n = ApU32(pay, n, (uint32_t)cnt);

    PROTO_SendPayload(pay, n);
}

/**
 * @brief  底盘状态帧：$CHS,mode,moving,y,x,rx,pct*CS
 *         mode: 0/1 → 帧内 'M'/'A'；moving: 0/1；y/x/rx: -100..100
 */
void PROTO_SendChassis(uint8_t mode, uint8_t moving,
                       float y_pct, float x_pct, float rx_pct,
                       uint8_t pct)
{
    char     pay[PROTO_TX_BUF_SIZE];
    uint16_t n = 0u;

    n = ApStr(pay, n, "CHS,");
    pay[n++] = (mode != 0u) ? 'A' : 'M';
    pay[n++] = ',';
    pay[n++] = (moving != 0u) ? '1' : '0';
    pay[n++] = ',';
    n = ApFloat1(pay, n, y_pct);
    pay[n++] = ',';
    n = ApFloat1(pay, n, x_pct);
    pay[n++] = ',';
    n = ApFloat1(pay, n, rx_pct);
    pay[n++] = ',';
    n = ApU32(pay, n, (uint32_t)pct);

    PROTO_SendPayload(pay, n);
}

/* ==========================================================================
 * 四、指令与参数解析
 * ========================================================================== */

/**
 * @brief  从正文中取出指令名并匹配指令表
 * @note   实现：截取第一个 ',' 之前的子串（最长 PROTO_CMD_NAME_MAX-1），
 *         与 s_cmd_names 做不区分大小写比较。
 * @param  payload 正文
 * @param  cmd     输出：指令编号
 * @retval 1 识别成功；0 未识别
 */
uint8_t PROTO_ParseCmd(const char *payload, ProtoCmd_t *cmd)
{
    char name[PROTO_CMD_NAME_MAX];
    uint16_t i = 0u;
    uint8_t  k;

    if ((payload == NULL) || (cmd == NULL))
    {
        return 0u;
    }

    while ((payload[i] != '\0') && (payload[i] != ',') && (i < (PROTO_CMD_NAME_MAX - 1u)))
    {
        name[i] = payload[i];
        i++;
    }
    name[i] = '\0';

    if (i == 0u)
    {
        *cmd = PROTO_CMD_NONE;
        return 0u;
    }

    for (k = 0u; k < (uint8_t)CMD_NAME_COUNT; k++)
    {
        if (PROTO_StrCaseEq(name, s_cmd_names[k]) != 0u)
        {
            *cmd = (ProtoCmd_t)k;
            return 1u;
        }
    }

    *cmd = PROTO_CMD_NONE;
    return 0u;
}

/**
 * @brief  由指令编号得到协议中使用的指令名字符串
 * @param  cmd 指令编号
 * @retval 指令名；未知返回 "UNK"
 */
const char *PROTO_CmdName(ProtoCmd_t cmd)
{
    if ((uint16_t)cmd >= (uint16_t)CMD_NAME_COUNT)
    {
        return "UNK";
    }
    return s_cmd_names[(uint8_t)cmd];
}

/**
 * @brief  取正文中的第 n 个字段
 * @note   实现：按 ',' 计数跳过前 index 个字段，然后拷贝到 out 直到遇到
 *         ',' 或字符串结束。out_size 不足时返回 0，不做截断拷贝，
 *         避免拿到半截参数导致误动作。
 * @param  payload  正文
 * @param  index    0 = 指令名
 * @param  out      输出缓冲区
 * @param  out_size 缓冲区大小
 * @retval 1 成功；0 不存在或缓冲区不足
 */
uint8_t PROTO_GetArg(const char *payload, uint8_t index,
                     char *out, uint16_t out_size)
{
    uint16_t i = 0u;
    uint8_t  cur = 0u;
    uint16_t n = 0u;

    if ((payload == NULL) || (out == NULL) || (out_size == 0u))
    {
        return 0u;
    }

    while (cur < index)
    {
        if (payload[i] == '\0')
        {
            return 0u;
        }
        if (payload[i] == ',')
        {
            cur++;
        }
        i++;
    }
    if (payload[i] == '\0')
    {
        return 0u;   /* 该字段不存在（例如 "SA," 尾部空参数） */
    }

    while ((payload[i] != '\0') && (payload[i] != ','))
    {
        if (n >= (uint16_t)(out_size - 1u))
        {
            return 0u;
        }
        out[n++] = payload[i++];
    }
    out[n] = '\0';

    return (n > 0u) ? 1u : 0u;
}

/**
 * @brief  统计正文中的字段个数（含指令名）
 * @note   实现：数 ',' 的个数 + 1。
 * @param  payload 正文
 * @retval 字段个数
 */
uint8_t PROTO_ArgCount(const char *payload)
{
    uint16_t i;
    uint8_t  cnt = 1u;

    if (payload == NULL)
    {
        return 0u;
    }
    for (i = 0u; payload[i] != '\0'; i++)
    {
        if (payload[i] == ',')
        {
            cnt++;
        }
    }
    return cnt;
}

/**
 * @brief  字符串转整数
 * @note   支持可选 '+'/'-'；遇到第一个非数字字符即停止。
 * @param  s 字符串
 * @retval 解析结果
 */
int32_t PROTO_ArgToInt(const char *s)
{
    int32_t  val = 0;
    uint8_t  neg = 0u;
    uint16_t i = 0u;

    if (s == NULL)
    {
        return 0;
    }
    if ((s[0] == '-') || (s[0] == '+'))
    {
        neg = (s[0] == '-') ? 1u : 0u;
        i = 1u;
    }
    while ((s[i] >= '0') && (s[i] <= '9'))
    {
        val = val * 10 + (int32_t)(s[i] - '0');
        i++;
        if (val > 1000000000L)
        {
            break;   /* 防止溢出 */
        }
    }
    return neg ? -val : val;
}

/**
 * @brief  字符串转浮点数
 * @note   支持可选 '+'/'-' 以及小数点后任意位（本项目实际只用到 1 位）。
 *         不使用 atof，避免引入较大的库函数。
 * @param  s 字符串
 * @retval 解析结果
 */
float PROTO_ArgToFloat(const char *s)
{
    float    val = 0.0f;
    float    neg = 1.0f;
    uint16_t i = 0u;

    if (s == NULL)
    {
        return 0.0f;
    }
    if (s[0] == '-')
    {
        neg = -1.0f;
        i = 1u;
    }
    else if (s[0] == '+')
    {
        i = 1u;
    }

    while ((s[i] >= '0') && (s[i] <= '9'))
    {
        val = val * 10.0f + (float)(s[i] - '0');
        i++;
    }

    if (s[i] == '.')
    {
        float scale = 0.1f;
        i++;
        while ((s[i] >= '0') && (s[i] <= '9'))
        {
            val += (float)(s[i] - '0') * scale;
            scale *= 0.1f;
            i++;
        }
    }

    return neg * val;
}

/**
 * @brief  忽略大小写的字符串比较
 * @note   实现：逐字符比较，遇到 '\0' 同时结束则相等。
 * @param  a 字符串 A
 * @param  b 字符串 B
 * @retval 1 相等；0 不相等
 */
uint8_t PROTO_StrCaseEq(const char *a, const char *b)
{
    uint16_t i = 0u;

    if ((a == NULL) || (b == NULL))
    {
        return 0u;
    }
    while ((a[i] != '\0') || (b[i] != '\0'))
    {
        char ca = a[i];
        char cb = b[i];

        if ((ca >= 'a') && (ca <= 'z')) { ca = (char)(ca - 'a' + 'A'); }
        if ((cb >= 'a') && (cb <= 'z')) { cb = (char)(cb - 'a' + 'A'); }

        if (ca != cb)
        {
            return 0u;
        }
        i++;
    }
    return 1u;
}
