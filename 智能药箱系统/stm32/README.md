# 智能药箱 —— STM32 下位机固件

> 芯片：**STM32F103C8T6（Blue Pill）** · 库：**STM32F1xx HAL** · 语言：**C99**
> 职责：**只做一件事** —— 通过串口接收角度指令，驱动 4 路舵机完成动作。
> 不做视觉、不做逆解、不做决策（这些都在 PC 端 Python 程序里）。

---

## 1. 定时器资源说明（为什么不用换成 F407）

STM32F103C8T6 拥有 TIM1（高级，4 通道）、TIM2、TIM3、TIM4（各 4 通道），资源完全够用：

| 用途 | 定时器 | 占用引脚 | 说明 |
|---|---|---|---|
| 4 路舵机 PWM（50Hz） | **TIM2 CH1~CH4** | PA0 / PA1 / PA2 / PA3 | 4 通道共用同一计数器，天然同相位 |
| 1ms 插值时基 | **TIM3**（仅内部计数） | 无 | 只开更新中断，不占引脚 |
| HAL 时基 | SysTick | 无 | `HAL_GetTick()` / `HAL_Delay()` |

因此**无需更换为 STM32F407**，F103C8T6 即可满足全部需求。

---

## 2. 引脚分配表

### 2.1 舵机（PWM 输出）

| 舵机编号 | 关节 | TIM2 通道 | 引脚 | GPIO 模式 | 备注 |
|---|---|---|---|---|---|
| servo0 | 底座 yaw 旋转 | TIM2_CH1 | **PA0** | 复用推挽 / 低速 | 无重映射 |
| servo1 | 大臂俯仰（肩） | TIM2_CH2 | **PA1** | 复用推挽 / 低速 | 无重映射 |
| servo2 | 小臂俯仰（肘） | TIM2_CH3 | **PA2** | 复用推挽 / 低速 | 无重映射 |
| servo3 | 夹爪开合 | TIM2_CH4 | **PA3** | 复用推挽 / 低速 | 无重映射 |

### 2.2 串口（与 PC 通信）

| 信号 | 引脚 | GPIO 模式 | 备注 |
|---|---|---|---|
| USART1_TX | **PA9** | 复用推挽 / 高速 | 接 USB-TTL 的 RXD |
| USART1_RX | **PA10** | 上拉输入 | 接 USB-TTL 的 TXD |

### 2.3 其它

| 功能 | 引脚 | 说明 |
|---|---|---|
| 板载 LED | **PC13** | 低电平点亮；0.5s 闪烁 = 正常运行；0.1s 快闪 = 初始化失败 |

> **共地很重要**：USB-TTL 的 GND 必须与 Blue Pill 的 GND 相连，否则串口数据全是乱码。
> 舵机建议**独立 5V 供电**，且该 5V 的 GND 也要与单片机共地；切勿用开发板的 3.3V/5V 直接带 4 个舵机。

---

## 3. CubeMX 配置步骤

下面按"新建工程 → 生成代码 → 放入本固件"的顺序说明。
若你已经有 CubeMX 工程，直接跳到第 4 节。

### 3.1 新建工程

1. `File → New Project → MCU Selector`，搜索 `STM32F103C8`，选择 `STM32F103C8T6`，Start Project。

### 3.2 SYS

| 项 | 值 |
|---|---|
| Debug | **Serial Wire**（务必选，否则下载一次后 SWD 可能被禁用） |

### 3.3 RCC

| 项 | 值 |
|---|---|
| High Speed Clock (HSE) | **Crystal/Ceramic Resonator**（Blue Pill 板载 8MHz 晶振） |
| Low Speed Clock (LSE) | Disable |

### 3.4 时钟树（Clock Configuration）

```
HSE 8MHz  ->  PLL x9  ->  SYSCLK 72MHz
HCLK  = 72MHz
APB1  = 36MHz   （注意：APB1 分频 != 1，所以挂 APB1 的 TIM2/TIM3 时钟 = 36 x 2 = 72MHz）
APB2  = 72MHz
```

### 3.5 USART1

| 项 | 值 |
|---|---|
| Mode | Asynchronous |
| Baud Rate | **115200** |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| 引脚 | PA9 = USART1_TX，PA10 = USART1_RX |

**DMA Settings**（在 USART1 的 DMA Settings 页添加 RX）：

| 项 | 值 |
|---|---|
| DMA Request | **USART1_RX** |
| Channel / Stream | **DMA1 Channel 5** |
| Direction | Peripheral to Memory |
| Mode | **Circular** ← 关键，必须循环模式 |
| Data Width | Byte / Byte |
| Increment Address | Peripheral: 不勾选；Memory: **勾选** |

**NVIC Settings**（在 USART1 的 NVIC Settings 页）：

- 勾选 **USART1 global interrupt**
- DMA1 channel5 global interrupt：在 `System Core → DMA → NVIC Settings` 里勾选

### 3.6 TIM2（4 路舵机 PWM）

在 `Timers → TIM2`：

| 项 | 值 |
|---|---|
| Clock Source | Internal Clock |
| Channel 1 | **PWM Generation CH1**（→ PA0） |
| Channel 2 | **PWM Generation CH2**（→ PA1） |
| Channel 3 | **PWM Generation CH3**（→ PA2） |
| Channel 4 | **PWM Generation CH4**（→ PA3） |
| Prescaler (PSC) | **71** |
| Counter Period (ARR) | **19999** |
| Counter Mode | Up |
| PWM Mode | PWM mode 1 |
| Pulse | 1500（初值，软件会覆盖） |
| CH Polarity | High |
| auto-reload preload | Disable |

> 72MHz / (71+1) = **1MHz**，即 1 个计数 = **1 微秒**；
> 1MHz / (19999+1) = **50Hz**，周期 20ms。
> 所以 CCR 里填多少就是多少微秒脉宽，500~2500 对应 0~180 度。

### 3.7 TIM3（1ms 时基）

| 项 | 值 |
|---|---|
| Clock Source | Internal Clock |
| Prescaler (PSC) | **71** |
| Counter Period (ARR) | **999** |
| NVIC | 勾选 **TIM3 global interrupt** |

1MHz / 1000 = **1kHz**，即每 1ms 进一次更新中断 → 舵机插值节拍。

### 3.8 GPIO

| 引脚 | 模式 |
|---|---|
| PC13 | GPIO_Output（板载 LED，标签可命名 LED） |

（PA0~PA3、PA9、PA10 由 TIM2 / USART1 自动配置，无需手动设置。）

### 3.9 IWDG（可选，推荐）

| 项 | 值 |
|---|---|
| Activated | ✔（或在 `main.h` 里把 `USE_IWDG` 设为 0 关闭） |
| IWDG counter clock prescaler | **64** |
| IWDG down-counter reload value | **1250** |

40kHz / 64 = 625Hz，1250 / 625 = **2 秒**超时。
主循环每圈喂狗一次；一旦程序跑飞，2 秒后自动复位，舵机会重新走上电安全流程。
**调试阶段建议把 `main.h` 里的 `USE_IWDG` 改成 0**，否则单步调试时会被复位。

### 3.10 Project Manager

| 项 | 值 |
|---|---|
| Toolchain / IDE | STM32CubeIDE（或 MDK-ARM V5，均可） |
| Code Generator | **不要勾选** "Generate peripheral initialization as a pair of '.c/.h' files per peripheral"（勾选后 MSP 函数会生成到 `stm32f1xx_hal_msp.c`，与本 `main.c` 中的定义重复） |

点击 **GENERATE CODE**。

---

## 4. 把本固件放入 CubeMX 工程

生成工程后，把本目录下的文件复制进去：

```
把  stm32/Core/Inc/main.h            ->  工程/Core/Inc/main.h        （覆盖）
    stm32/Core/Inc/protocol.h       ->  工程/Core/Inc/protocol.h    （新增）
    stm32/Core/Inc/servo.h          ->  工程/Core/Inc/servo.h       （新增）
    stm32/Core/Src/protocol.c       ->  工程/Core/Src/protocol.c    （新增）
    stm32/Core/Src/servo.c          ->  工程/Core/Src/servo.c       （新增）
    stm32/Core/Src/main.c           ->  工程/Core/Src/main.c        （覆盖）
    stm32/Core/Src/stm32f1xx_it.c   ->  工程/Core/Src/stm32f1xx_it.c（覆盖）
```

然后**必须做以下 3 件事**（否则编译不过或运行异常）：

1. **不要勾选"外设初始化生成到独立 .c 文件"**时，`MX_GPIO_Init / MX_DMA_Init / MX_USART1_UART_Init / MX_TIM2_Init / MX_TIM3_Init / MX_IWDG_Init`
   已由本 `main.c` 提供。若你的工程里 `stm32f1xx_hal_msp.c` 也有 `HAL_TIM_PWM_MspInit` 等函数，
   会与 `main.c` 中的定义**重复**，请删除 `stm32f1xx_hal_msp.c` 里的对应函数（或把 `main.c` 里的删掉）。
2. **删除 CubeMX 生成的 `HAL_TIM_PWM_MspInit / HAL_TIM_Base_MspInit / HAL_UART_MspInit / HAL_MspInit`**
   中与本工程重复的副本（保留一份即可）。
3. **确认 `stm32f1xx_it.c` 中 `SysTick_Handler` 只有一份**（本文件已提供）。

> 若在 CubeMX 里再次 GENERATE CODE，会重新生成 `main.c` / `stm32f1xx_it.c`，
> 记得**再次覆盖**这两个文件。

### 4.1 必须保证：程序不能占用最后两页 Flash

配置区在 `0x0800F800`（页 62），动作表在 `0x0800FC00`（页 63）。
**编译后请确认程序大小 < 63KB**（正常本固件约 15~25KB，没有问题）。
若你的 C8T6 实际是 128KB（很多板子其实是 CB），这两个地址依然可用，无需修改。

---

## 5. 编译与烧录

### 5.1 编译

- **STM32CubeIDE**：`Project → Build All`。
- 编译选项建议：Optimization `-O2`（`-Os` 亦可）；**不要用 `-Ofast`**。

### 5.2 烧录

| 方式 | 说明 |
|---|---|
| **ST-Link V2（推荐）** | SWDIO–PA13、SWCLK–PA14、GND、3.3V 四根线；CubeIDE 里 Run → Debug 即可 |
| **串口 ISP** | 需要把 BOOT0 跳到 1，用 FlyMcu / STM32CubeProgrammer 通过 USART1（PA9/PA10）下载，下完把 BOOT0 跳回 0 |

ST-Link 接线：

```
ST-Link    Blue Pill
3.3V   ->  3.3V
GND    ->  GND
SWDIO  ->  PA13 (DIO)
SWCLK  ->  PA14 (DCLK)
```

### 5.3 上电自检现象

1. 上电后 LED 以 0.5s 周期闪烁；
2. 舵机**不动**（因为上电假定角 = HOME，归位位移为 0），或按你配置的 `SERVO*_BOOT_DEG` 平滑转到 HOME；
3. 归位完成后串口发出一帧 `$READY,VER1.0*09`；
4. 此后每 1 秒发出一帧 `$HB,1*..`、`$HB,2*..` …

> 若 LED 以 0.1s 快速闪烁，说明初始化失败（进入 `Error_Handler`），
> 常见原因是 HSE 配置错误或 CubeMX 生成的 MSP 函数与本文件重复/缺失。

---

## 6. 通信协议速查表

### 6.1 帧格式

```
$<CMD>[,<arg1>,<arg2>,...]*<CS>\r\n
```

- `$` 帧头，`*` 后为校验，`\r\n` 帧尾
- **CS = 从 `$` 之后到 `*` 之前所有字符的按字节 XOR，2 位大写十六进制**
- 例：`$SA,90,75,60,40,800*0F\r\n`（`SA,90,75,60,40,800` 的 XOR = `0x0F`）
- 校验失败 → 整帧丢弃并回复 `$ERR,01,CS*..`，**绝不"容错执行"**

### 6.2 PC → STM32

| 指令 | 格式 | 含义 | 说明 |
|---|---|---|---|
| SA | `$SA,a1,a2,a3,a4,tt*CS` | 四路**同步插值**运动 | `tt` = 总时长 ms；四路同时启动、同时到达 |
| SS | `$SS,id,ang,tt*CS` | 单舵机运动（调试） | `id` = 0..3 |
| GC | `$GC,open*CS` / `$GC,close*CS` | 夹爪开 / 合 | 内部映射为 servo3 的 70° / 15°（可在 `main.h` 改） |
| RUN | `$RUN,slot_id*CS` | 执行 Flash 预存动作序列 | `slot_id` = 1..4 |
| HOM | `$HOM*CS` | 回 HOME 位 | 1500ms 平滑归位 |
| QRY | `$QRY,POSE*CS` | 查询当前四路角度 | 回复 `$ACK,QRY` + `$POSE,...` |
| EST | `$EST*CS` | 急停 | 立即冻结目标角 = 当前角，停止插值 |
| CFG | `$CFG,id,min,max*CS` | 配置某舵机限位并写入 Flash | 例：`$CFG,1,15,165*CS` |

### 6.3 STM32 → PC

| 指令 | 格式 | 时机 |
|---|---|---|
| READY | `$READY,VER1.0*CS` | 上电初始化 + 归位完成后发一帧，**PC 收到它之后才能发业务指令** |
| ACK | `$ACK,SA*CS` | 指令校验通过、已开始执行（不代表执行完毕） |
| DONE | `$DONE,SA*CS` | 运动到达目标位置（动作序列只在整条跑完后发一次 `$DONE,RUN`） |
| POSE | `$POSE,a1,a2,a3,a4*CS` | 响应 QRY，角度保留 1 位小数 |
| ERR | `$ERR,code,msg*CS` | 错误 |
| HB | `$HB,cnt*CS` | 1Hz 心跳，`cnt` 递增（0~65535 回绕） |

### 6.4 错误码

| code | 含义 | 触发场景 | msg 示例 |
|---|---|---|---|
| 01 | 校验失败 | XOR 不匹配，或校验位不是合法 HEX | `CS` |
| 02 | 未知指令 / 参数格式错误 | 无法识别的指令、缺参数、帧超长、槽位非法 | `CMD` `ARG` `FMT` `ID` `SLOT` |
| 03 | 角度越限 | 超过该舵机软限位或硬件绝对限位；`$CFG` 的 min/max 非法 | `LIM` |
| 04 | 执行超时 | 运动开始后超过 `总时长 + 2000ms` 仍未结束（1ms 中断停摆兜底） | `TMO` |
| 05 | 舵机异常/堵转 | 连续 3 次超时 | `STALL` |

### 6.5 典型交互时序

```
STM32  ---- $READY,VER1.0*09 ---->   PC
PC     ---- $SA,90,75,60,40,800*0F -> STM32
STM32  ---- $ACK,SA*.. ------------>  PC      （已开始执行）
STM32  ---- $DONE,SA*.. ----------->  PC      （800ms 后到达）
PC     ---- $QRY,POSE*.. ---------->  STM32
STM32  ---- $ACK,QRY*.. ----------->  PC
STM32  ---- $POSE,90.0,75.0,60.0,40.0*.. -> PC
STM32  ---- $HB,1*.. -------------->  PC      （每 1 秒）
```

> PC 端建议：发完 `SA/SS/GC/RUN/HOM` 后等待 `$DONE`，超时 3~5 秒未收到则重发或报警。
> 心跳可以用于判断下位机是否掉线（超过 3 秒没收到 `$HB` 即认为异常）。

---

## 7. 关键实现说明

### 7.1 串口接收：DMA 循环缓冲 + IDLE 中断 + 软件 FIFO

- `HAL_UART_Receive_DMA()` 以 **Circular** 模式搬数据到 256 字节的 `g_rx_dma_buf`；
- 收到一帧后总线空闲，**IDLE 标志**置位 → 进入 `USART1_IRQHandler`；
- **清除 IDLE 标志的正确姿势（F1 的大坑）**：不能用写 0 清除，必须
  **先读 `SR`，再读 `DR`**（见 `main.c` 的 `Uart_ClearIdleFlag()`）。
  清除失败会导致中断反复触发、主循环被拖死；
- 中断中用 `write_pos = RX_DMA_BUF_SIZE - CNDTR` 计算 DMA 写指针，
  并**正确处理回绕**（`cur < last` 时分两段取），把新增字节搬进 512 字节的软件 FIFO；
- 解析放在主循环里做（单生产者-单消费者，无需关中断），中断占用时间极短。

### 7.2 四路同步插值

不是"每个舵机按各自速度跑"，而是**共享同一个归一化时间参数**：

```c
t = elapsed / total;                       // 四个通道共用
cur[i] = start[i] + (target[i] - start[i]) * t;
```

`start[i]` 在运动启动那一刻被统一记录为"当前角"，因此四路**同时启动、同时到达**。
推进在 TIM3 的 1ms 中断中完成，主循环完全不阻塞。

### 7.3 限位保护（最后一道防线）

- PC 的角度只作为"请求"，STM32 侧 **先全部校验、再统一动作**；
- 任一通道越限 → 整条指令拒绝，回复 `$ERR,03,LIM`，
  **不会出现"两路动了、两路没动"的半截状态**；
- 软限位存在 Flash 中，可用 `$CFG` 修改；硬件绝对限位 `0 ~ 满量程` 不可逾越；
- 判定时带 0.1° 容差，避免浮点/取整误差误判。

### 7.4 急停

`$EST` 到达后，主循环在**同一周期内**调用 `SERVO_StopNow()`：

- 清零插值与序列状态；
- 每个通道 `target = now`（冻结在当前角）；
- 保持当前 CCR 不变 → 舵机停在原地并维持力矩（不会掉臂砸东西）；
- 若希望急停后脱力，把 `main.h` 的 `ESTOP_RELEASE_PWM` 改为 1。

### 7.5 上电安全

1. 装载 Flash 配置（无效则用默认值）；
2. **先写 CCR，再启动 PWM 输出** —— 这样定时器一使能输出的就是正确脉宽，
   绝不会先冒出一个随机宽脉冲让舵机乱抖；
3. 再启动一次"从 `SERVO*_BOOT_DEG` 到 HOME"的平滑同步运动；
4. 归位完成后才发 `$READY`。

> 舵机没有角度反馈，上电时无法知道真实位置。默认 `BOOT = HOME`，
> 因此归位位移为 0（最安全）。若想演示时看到明显归位动作，
> 把 `main.h` 里的 `SERVO0_BOOT_DEG` 等改成 `0`。

### 7.6 Flash 持久化

| 区域 | 地址 | 页 | 内容 | 大小 |
|---|---|---|---|---|
| 配置区 | `0x0800F800` | 62 | 魔数 + 版本 + 校验和 + 限位/HOME/夹爪/微调/方向 | 76 B |
| 动作表 | `0x0800FC00` | 63 | 魔数 + 版本 + 校验和 + 4 槽 × 8 点位 | 668 B |

- 三重校验：**魔数 → 版本号 → 16 位累加和**，任一不符即加载默认值；
- F103 只能**半字（2 字节）编程**且必须**整页擦除**，
  写流程为：关中断 → 解锁 → 擦 1KB 页 → 逐半字写入 → 上锁 → 开中断；
- 只有 `$CFG` 这类低频操作才会触发擦写，Flash 寿命无虞。

### 7.7 看门狗

独立看门狗 IWDG（40kHz 内部 RC，与主时钟独立），2 秒超时，主循环每圈 `HAL_IWDG_Refresh()`。
调试时把 `main.h` 的 `USE_IWDG` 设为 0 即可关闭。

---

## 8. 常用参数调整（都在 `Core/Inc/main.h`）

| 想改什么 | 改哪个宏 |
|---|---|
| 舵机是 270 度的（2500us = 270°） | `SERVO_ANGLE_SPAN_DEG` 改为 `270.0f` |
| 脉宽范围不是 500~2500us | `SERVO_PULSE_MIN_US` / `SERVO_PULSE_MAX_US` |
| HOME 位角度 | `SERVO0_HOME_DEG` ~ `SERVO3_HOME_DEG` |
| 夹爪开/合角度 | `GRIPPER_OPEN_DEG` / `GRIPPER_CLOSE_DEG` |
| 默认软限位 | `SERVO0_LIMIT_MIN_DEG` ~ `SERVO3_LIMIT_MAX_DEG` |
| 某关节装反了 | `SERVO*_INVERT` 改为 `1` |
| 某关节差了几度 | `SERVO*_TRIM_US`（单位 us，正值加大脉宽） |
| 上电是否做归位动作 | `SERVO0_BOOT_DEG` ~ `SERVO3_BOOT_DEG` |
| 归位/心跳/急停行为 | `BOOT_HOME_MOVE_MS` / `HB_PERIOD_MS` / `ESTOP_RELEASE_PWM` |
| 看门狗开关与超时 | `USE_IWDG` / `IWDG_TIMEOUT_MS` |
| 动作序列槽数与每槽点数 | `RUN_SLOT_COUNT` / `RUN_POINTS_PER_SLOT`（改大要确认放得进 1KB 页） |
| 板子没有 8MHz 晶振 | `SystemClock_Config()` 改用 HSI，并同步改 `SERVO_TIM_CLK_HZ` |

换定时器/引脚时，需要同步修改：
`Core/Src/servo.c` 的 `kTimChannel[]`、`Core/Src/main.c` 的 `MX_TIM2_Init()` 与
`HAL_TIM_PWM_MspInit()`，以及 `main.h` 中的 `SERVO_PWM_PSC` / `SERVO_PWM_ARR`。

---

## 9. 目录结构

```
stm32/
├── Core/
│   ├── Inc/
│   │   ├── main.h       引脚与外设宏定义、全局配置（所有可调参数集中于此）
│   │   ├── protocol.h   协议帧构建/解析、校验、指令枚举、错误码
│   │   └── servo.h      舵机 PWM 驱动、角度设置、插值、限位、Flash 存储结构
│   └── Src/
│       ├── main.c       外设初始化 + 主循环 + 中断回调 + 指令处理
│       ├── protocol.c   协议解析状态机与帧构建/发送
│       ├── servo.c      PWM 驱动、同步插值、动作序列、Flash 持久化
│       └── stm32f1xx_it.c  中断服务函数
└── README.md            本文件
```

---

## 10. 常见问题排查

| 现象 | 可能原因 |
|---|---|
| 串口收到乱码 | 波特率不一致 / 没共地 / HSE 实际频率与 `SystemClock_Config` 不符 |
| 收不到 `$READY` | 上电归位没结束（检查 `BOOT_HOME_MOVE_MS`）；或 TX 线没接 |
| 一直报 `$ERR,01` | PC 端校验算法与本文不一致（确认是"从 `$` 后到 `*` 前"的 XOR，且是大写 2 位 HEX） |
| `$ERR,03` | 角度超限位。用 `$CFG` 放宽限位，或改 `main.h` 默认值 |
| 舵机完全不动 | PA0~PA3 没配成复用推挽（检查是否误先调了 `HAL_TIM_Base_Init` 导致 MSP 被跳过）；或舵机没独立供电 |
| 舵机抖动/发热 | 脉宽超出舵机行程；调整限位或 `SERVO_PULSE_MIN/MAX_US` |
| 上电瞬间舵机乱抖 | 检查是否先写了 CCR 再启动 PWM（本固件 `SERVO_Init` 已按正确顺序实现） |
| 调试时总复位 | `main.h` 的 `USE_IWDG` 设为 0 |
| `$CFG` 后断电重启失效 | Flash 写入失败；检查程序是否 < 63KB、Flash 地址是否正确 |
