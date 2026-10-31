# 智能药箱毕业设计 —— 完整工程与代码说明

> 无 ROS / FunASR 语音 + GLM-4-Flash 大模型语义理解 + 本地药品知识库 + USB 摄像头 OpenCV 视觉确认 + STM32 驱动 4 舵机机械臂 + TTS 语音反馈

---

## 0. 一句话结论（方案是否可行）

**方案可行，且已在纯软件仿真下完整跑通。** 关键验证结论：

| 验证项 | 结果 |
|---|---|
| 4 舵机臂运动学可达性（弧形布局） | **8/8 槽位全部可达**，逆解回代误差 < 0.3mm |
| 矩形布局可达性 | 仅 7/9（最远列接近角 -12.6°，太平夹不住）→ **必须用弧形/朝向基座布局** |
| OpenCV 二维码识别（多后端融合） | 噪声/变暗/旋转/透视 **8/8 稳**；模糊 k≥5、缩小 <0.5 才掉 → **加环形补光+留静区** |
| 串口协议（帧构建/校验/半帧/粘包/脏数据） | 自洽通过 |
| 端到端主控流程（正常/高风险确认/抓错放回/空槽/未知） | **45/45 测试全部通过** |

> 注意：STM32 固件无法在沙盒内真编译（无 ARM GCC 工具链），但已通过完整静态审查（头文件 include 链、`main.h` 中 extern 句柄声明、所有 `SERVO_/PROTO_` 函数定义、中断表与 NVIC 使能一一对应）。实际编译需在 Windows + STM32CubeIDE 中完成（见第 5、11 节）。

---

## 1. 系统架构与数据流

```
用户语音 ──┐
            ▼
   ┌───────────────────────────── PC（Python / PyCharm）────────────────────────────┐
   │  ASR(FunASR) → GLM-4-Flash(语义映射) → 知识库(定位药品+槽位)                      │
   │        │                                                                         │
   │        ▼                                                                         │
   │  OpenCV(USB摄像头,固定前上方) ── 二维码/色块/帧差 ── 确认"身份+槽位有无"          │
   │        │                                                                         │
   │        ▼                                                                         │
   │  查预设点位表(PoseTable) → 生成 4 舵机角度序列 → 串口下发                         │
   └──────────────────────────────┬──────────────────────────────────────────────┘
                                    │  USB 串口 115200/8N1  ASCII 帧 `$CMD,args*XOR\r\n`
                                    ▼
                          STM32F103C8T6（只做一件事：收角度→驱动 4 路舵机）
                          TIM2 4路PWM(50Hz) + DMA+IDLE 收串口 + TIM3 1ms 插值
```

- **STM32 不做视觉、不做逆解、不做决策**，只可靠执行角度序列并反馈 ACK/DONE/错误/心跳。
- **过检位（inspect pose）安全闭环**：抓取后先送固定扫码位二次确认，一致才交付，不一致放回并 TTS 报警。这是"抓错了怎么办"的标准答案。

---

## 2. 目录结构

```
smart_medbox/
├── README.md                      ← 本文件
├── config/                        ← 运行时配置（可改）
│   ├── app.yaml                   主配置文件（串口/相机/视觉/重试/槽位）
│   ├── medicines.json             药品清单（名称/类别/槽位/风险/库存）
│   └── poses.json                 预设点位表（gen_poses.py 生成）
├── pc/                           ← PC 端 Python 主控（包）
│   ├── main.py                    主控入口：装配所有部件 + 主循环
│   ├── comm/                      串口通信层
│   │   ├── serial_protocol.py     帧格式/校验/流解析（纯逻辑，可单测）
│   │   └── arm_controller.py      机械臂控制器（发指令+等ACK/DONE+Mock）
│   ├── vision/                    视觉层
│   │   ├── qr_detector.py         二维码多后端融合识别 + 生成标签图
│   │   ├── camera.py              摄像头封装（真实/模拟）
│   │   └── slot_monitor.py        槽位"有无药品"监测
│   ├── audio/                     语音层
│   │   ├── asr.py                 FunASR 语音识别
│   │   └── tts.py                 TTS 语音合成（pyttsx3/edge/mock）
│   ├── llm/                       大模型层
│   │   └── glm_client.py          智谱 GLM-4-Flash 语义映射
│   └── core/                      核心逻辑
│       ├── kinematics.py          运动学 FK/IK
│       ├── pose_table.py          预设点位表读写
│       ├── medicine_kb.py         药品知识库
│       ├── state_machine.py       取药状态机（核心流程）
│       └── logger.py              分级日志（落盘）
├── stm32/                        ← 下位机固件（C/HAL）
│   └── Core/
│       ├── Inc/{main,protocol,servo}.h
│       ├── Src/{main,protocol,servo,stm32f1xx_it}.c
│       └── README.md              CubeMX 配置与编译烧录步骤
├── tools/                        ← 离线工具（标定/生成/验证）
│   ├── calibrate_slots.py         鼠标框选槽位 ROI → slots.yaml
│   ├── color_tuner.py             HSV 阈值调试 → 输出色块配置
│   ├── make_qr_labels.py          批量生成药品二维码标签图
│   ├── gen_poses.py               由药箱布局生成 poses.json 并校验可达
│   ├── vision_sim.py              视觉链路仿真（合成场景+鲁棒性测试）
│   └── kinematics_check.py        运动学可行性验证（工作空间/可达性图）
├── docs/                         ← 生成的图（工作空间、视觉仿真结果）
└── tests/
    └── test_pipeline.py          端到端流程测试（45 项，全仿真）
```

---

## 3. 硬件与引脚分配

### PC 端
- 主控电脑（Win10/11，Python 3.11，PyCharm）
- USB 摄像头：1080P、手动定焦、视场角 ≤60°、固定安装于药箱前上方，**绝不装在机械臂上**
- 环形补光灯常亮
- USB 串口（CH340/CP2102）→ STM32

### STM32（F103C8T6 Blue Pill）引脚
| 信号 | 引脚 | 说明 |
|---|---|---|
| USART1_TX | PA9 | 复用推挽 → PC |
| USART1_RX | PA10 | 上拉输入 ← PC（DMA1_Channel5 循环） |
| TIM2_CH1 | PA0 | 舵机0 底座 yaw |
| TIM2_CH2 | PA1 | 舵机1 大臂肩 |
| TIM2_CH3 | PA2 | 舵机2 小臂肘 |
| TIM2_CH4 | PA3 | 舵机3 夹爪 |
| LED | PC13 | 心跳指示（低电平亮） |
| TIM3 | 内部 | 1ms 插值基准（不占引脚） |

舵机电源：4 路独立 5–6V / ≥3A，**只与逻辑侧共 GND，VCC 不共**。

---

## 4. 通信协议速查（PC ↔ STM32）

帧格式：`$<BODY>*<CS>\r\n`
- `$` 帧头，`*` 校验分隔，`\r\n` 帧尾
- `CS` = BODY 每个字符 ASCII 码**逐字节 XOR**，2 位大写十六进制
- 校验失败整帧丢弃并回 `$ERR,01`；**绝不"容错执行"**

**PC → STM32**
| 指令 | 格式 | 含义 |
|---|---|---|
| SA | `$SA,a1,a2,a3,a4,tt*CS` | 四舵机**同步插值**到目标角，tt=总时长ms（主控指令） |
| SS | `$SS,id,ang,tt*CS` | 单舵机运动（调试） |
| GC | `$GC,open\|close*CS` | 夹爪开/合 |
| RUN | `$RUN,slot_id*CS` | 执行 Flash 预存动作序列 |
| HOM | `$HOM*CS` | 回零位 |
| QRY | `$QRY,POSE*CS` | 查询当前四路角度 |
| EST | `$EST*CS` | 急停（冻结当前位置） |
| CFG | `$CFG,id,min,max*CS` | 配置舵机限位并写 Flash |

**STM32 → PC**
| 指令 | 格式 | 含义 |
|---|---|---|
| READY | `$READY,VER1.0*CS` | 上电初始化完成（PC 收到后才发业务指令） |
| ACK | `$ACK,<cmd>*CS` | 已收到、校验通过、已开始执行 |
| DONE | `$DONE,<cmd>*CS` | 运动到达目标 |
| POSE | `$POSE,a1,a2,a3,a4*CS` | 当前角度上报 |
| ERR | `$ERR,code,msg*CS` | 错误：`01`校验 `02`未知指令 `03`越限 `04`超时 `05`堵转 |
| HB | `$HB,cnt*CS` | 1Hz 心跳，cnt 递增 |

**铁律**：每条指令等 ACK、运动等 DONE、限位校验放 STM32 侧最后一道防线。

---

## 5. 环境依赖与快速开始

### 5.1 Python 依赖
```bash
pip install numpy opencv-python pyserial pyyaml requests pillow qrcode
# 语音/大模型（缺了也会自动降级为 mock，不影响跑通流程）：
pip install funasr sounddevice pyttsx3
# 若要用微信二维码解码后端（可选，识别率更高）：
pip install opencv-contrib-python
```

### 5.2 不接任何硬件先跑通全流程（mock 模式）
```bash
cd smart_medbox
python -m pc.main --mock        # 语音用键盘输入代替，视觉/舵机全仿真
```
按提示输入"我感冒了"，观察 TTS 文本打印、舵机角度序列、过检位确认全流程。

### 5.3 跑端到端测试
```bash
python tests/test_pipeline.py     # 期望输出：测试结果: 45/45 通过
```

### 5.4 生成/校验点位表
```bash
python tools/gen_poses.py --check   # 由 app.yaml 的槽位布局生成 poses.json 并校验可达
```

### 5.5 STM32 编译烧录（Windows）
见 `stm32/README.md`：用 STM32CubeMX 建 F103C8T6 工程，把 `stm32/Core` 下的 .h/.c 替换进 `Core/Inc` 与 `Core/Src`，配置 USART1(115200)、TIM2(4ch PWM)、TIM3(1ms)、DMA1_Channel5(USART1_RX 循环)、PC13 LED，编译烧录。

---

## 6. PC 端代码逐文件 / 逐函数说明

### 6.1 `pc/comm/serial_protocol.py` —— 协议纯逻辑层
- **`xor_checksum(body)`**：计算 BODY 逐字节 XOR 校验值（0–255）。
- **`build_frame(body)`**：把 BODY 打包成完整帧（`$BODY*CS\r\n`），返回 bytes 可直接 `ser.write`。
- **`verify_line(line)`**：校验并解析一行文本；通过返回 BODY，失败/格式错返回 None。
- **`split_cmd(body)`**：拆帧体为 `(指令名, [参数...])`，指令名转大写。
- **`StreamParser`**：串口字节流解析器，处理半帧/粘包/脏数据。
  - **`feed(data)`**：喂入新字节，返回本次凑齐的所有合法帧体列表。
  - **`reset()`**：清空缓冲区（换端口/重连时调用）。
- **`describe_error(code)`**：把错误码翻译成人话（日志/TTS 用）。

### 6.2 `pc/comm/arm_controller.py` —— 机械臂控制器
- **`ArmController`**：串口控制器。
  - **`list_ports()`**：列出可用串口。
  - **`connect()`**：开串口并启动接收线程；显式关 DTR/RTS（防 CH340 复位 STM32）。
  - **`close()`**：关串口/线程。
  - **`wait_ready()`**：等 `$READY` 帧（上电后必须先等）。
  - **`move_all(a1..a4, tt)`**：四舵机同步运动，等 ACK+DONE。
  - **`move_servo(id, ang, tt)`**：单舵机运动。
  - **`grip(open_it, tt)`**：夹爪开/合。
  - **`home()`**：回零位。
  - **`estop()`**：急停（不等 DONE，越快越好）。
  - **`run_slot(slot_id)`**：执行 Flash 预存序列（备用）。
  - **`query_pose()`**：查询当前角度，失败返回 None。
  - **`is_alive()`**：据心跳判断 STM32 是否在。
- **`MockArmController(ArmController)`**：不碰串口的离线版，额外记录 `history`（测试断言用）。

### 6.3 `pc/vision/qr_detector.py` —— 二维码识别
- **`QRResult`**：识别结果数据类（code/cx/cy/width/polygon/backend/votes）。
- **`QRDetector`**：多后端融合解码器（wechat→aruco→builtin 投票，抗模糊/小尺寸）。
  - **`detect(frame, roi)`**：在帧（或 ROI）内解码，返回按可信度排序的结果列表。
  - **`detect_best(frame, roi)`**：返回最可信的单条结果（多后端一致优先）。
  - **`robust_scan(frame, roi, n)`**：连采 n 帧投票，提升稳定性。
- **`make_qr_image(text, size, quiet_zone_ratio, ecc_level)`**：生成二维码灰度图（标签打印用）。

### 6.4 `pc/vision/camera.py` —— 摄像头封装
- **`Camera`**：真实摄像头（OpenCV）。
  - **`open()`**：开流；锁自动对焦/曝光/白平衡，`BUFFERSIZE=1`（避免读到旧帧）。
  - **`read()`**：返回 `(ok, frame)`。
  - **`release()`**：释放。
- **`MockCamera`**：用图片/图片列表模拟，便于无摄像头自测。

### 6.5 `pc/vision/slot_monitor.py` —— 槽位有无监测
- **`SlotROI`**：单个槽位 ROI 矩形封装（`rect()` 返回 x,y,w,h）。
- **`SlotMonitor`**：判每个槽位是否有药。
  - **`set_background(frame)`**：记录空箱背景帧。
  - **`check(frame)`**：返回 `{slot_id: bool}` 有无表（帧差/色块/圆点可见性策略）。
  - **`describe()`**：把状态表转人话。
- **`build_from_config(path)`**：从 slots.yaml 加载 ROI 构造监视器。

### 6.6 `pc/audio/asr.py` —— 语音识别
- **`_print_funasr_help()`**：打印 FunASR 安装/模型下载提示。
- **`parse_funasr_result(obj)`**：解析 FunASR 返回的 list[dict] 取文本。
- **`_is_mostly_chinese(s)`**：判断文本是否以中文为主（过滤噪声）。
- **`ASREngine`**：
  - **`recognize_once(timeout, phrase_timeout)`**：麦克风录音+端点检测+识别，超时返回 None；mock 模式用 `input()` 代替。
  - **`recognize_file(path)`**：识别 wav 文件。
  - **`close()` / `is_mock()`**：释放资源 / 是否 mock。

### 6.7 `pc/audio/tts.py` —— 语音合成
- **`TTSEngine`**：
  - **`speak(text, wait)`**：朗读文本，返回是否成功（失败不影响主控）。
  - **`speak_async(text)`**：开线程非阻塞朗读。
  - **`close()` / `is_mock()`**：释放 / 是否 mock。

### 6.8 `pc/llm/glm_client.py` —— 大模型语义映射
- **`GLMClient`**：
  - **`map_symptom(text)`**：核心入口。把用户原话映射为 `{ok, intent, category, reason, source}`。只让大模型输出**药品类别**（白名单校验），绝不直接输出槽位/药名；失败/限流/幻觉都降级为"没听懂"或本地规则。**mock 模式**用关键词规则离线映射。

### 6.9 `pc/core/kinematics.py` —— 运动学
- **`ArmGeometry`**：几何参数封装，`b()` 返回有效二连杆长度 B。
- **`fk(x, y, z, g)`**：由 (s0,s1,s2) 角算末端 (x,y,z)。
- **`ik(x, y, z, home, g)`**：逆解，返回各舵机角 + 是否在安全区 + 接近角；不可达返回 None。

### 6.10 `pc/core/pose_table.py` —— 点位表
- **`PoseTable`**：预设点位读写。
  - **`load/save`**：JSON 读写。
  - **`has/get/set_slot/slot_ids`**：查询/设置槽位角度。
- **`generate_poses(slots, geom)`**：由槽位坐标批量求逆解生成点位表。

### 6.11 `pc/core/medicine_kb.py` —— 药品知识库
- **`Medicine`**：单药品数据类（code/name/category/slot/high_risk/stock/note）。
- **`MedicineKB`**：
  - **`load/save/add/all/categories`**：读取/保存/新增/遍历/类别列表。
  - **`by_code/by_slot/by_category`**：按码/槽位/类别查询。
  - **`pick_one(category)`**：类别内选一种（库存优先）。
  - **`consume(code, n)`**：扣库存。
  - **`is_valid_category(c)`**：白名单校验（防大模型幻觉越界）。

### 6.12 `pc/core/logger.py` —— 日志
- **`Logger`**：分级日志，`log/info/step/warn/error` + `close()`（落盘，便于回溯与论文测试数据）。

### 6.13 `pc/core/state_machine.py` —— 取药状态机（核心）
- **`MedicineBoxSystem`**：
  - **`listen()`**：获取一次用户语音/输入。
  - **`confirm_with_user(prompt)`**：高风险药二次确认（TTS 问→ASR 答"是/否"）。
  - **`check_slot_has_medicine(slot)`**：视觉判空槽，提前告知。
  - **`inspect_grabbed(expected)`**：过检位扫码，返回实际药品码（安全闭环核心）。
  - **`pick_from_slot / return_to_slot / deliver`**：取/放/递送动作序列。
  - **`fault(msg)`**：异常进入 FAULT 并 TTS 报警。
  - **`run_once()`**：跑一轮完整取药流程，返回状态（success/cancel/mismatch/empty_slot/unknown/fault）。
  - **`emergency_stop()`**：触发急停。

### 6.14 `pc/main.py` —— 主控入口
- **`load_config`**：加载 app.yaml。
- **`build_asr/build_tts/build_camera`**：按配置构造语音/视觉部件（含 mock 降级）。
- **`_KeyboardASR/_PrintTTS`**：无外设时的键盘/打印替身。
- **`build_system`**：装配所有部件成 `MedicineBoxSystem`。
- **`_default_slot_positions`**：默认药箱布局（弧形，朝向基座）。
- **`main`**：解析命令行（`--mock`、`--port` 等）+ 主循环。

---

## 7. STM32 固件逐文件 / 逐函数说明

### 7.1 `Core/Inc/main.h`
全局配置中心：固件版本、串口/缓冲参数、舵机数量与分工、PWM 定时器（TIM2 4ch@PA0–3，TIM3 1ms）、脉宽/角度范围、HOME/夹爪/限位默认值、运动参数、动作槽数、Flash 页地址（页62配置/页63动作表）、LED、看门狗开关，以及 **所有全局句柄 `extern` 声明**（huart1/hdma_usart1_rx/htim2/htim3/hiwdg）与跨文件函数声明。

### 7.2 `Core/Src/main.c`
- **`main()`**：初始化顺序（HAL→GPIO/DMA/USART→TIM2/TIM3→启动 DMA+IDLE→TIM3 1ms→SERVO_Init→主循环）；主循环逐字节喂协议状态机、处理运动完成事件、超时/堵转检测、1Hz 心跳、喂狗。
- **`Uart_ClearIdleFlag()`**：F1 清 IDLE 标志（先读 SR 再读 DR 的固定序列）。
- **`USER_UART_IdleIrqHandler()`**：IDLE 中断入口（清标志+搬运）。
- **`USER_UART_IdleHandler()`**：把 DMA 环形缓冲新增字节搬进软件 FIFO，正确处理**指针回绕**。
- **`Fifo_Push/Fifo_Pop`**：无锁单生产者单消费者字节 FIFO（索引 `& MASK` 回绕）。
- **`Protocol_Handle()`**：指令分发（SA/SS/GC/RUN/HOM/QRY/EST/CFG）。
- **`HandleCmd_SA/SS/GC/RUN/HOM/QRY/EST/CFG`**：各指令处理（先全部检查再动作，限位越限整条拒绝）。
- **`HAL_TIM_PeriodElapsedCallback()`**：TIM3→`SERVO_Update1ms()`。
- **`HAL_UART_ErrorCallback()`**：串口错误最小化恢复（对齐 DMA 读指针、复位协议、重使能 IDLE）。
- **`HAL_MspInit()`**：NVIC 分组 + AFIO 时钟。
- **`SystemClock_Config()`**：HSE 8M→PLL×9→72MHz。
- **`MX_GPIO_Init/MX_DMA_Init/MX_USART1_UART_Init`**：外设初始化。
- **`HAL_UART_MspInit()`**：USART1 的 GPIO+DMA+NVIC（PA9/PA10、DMA1_Channel5 循环、优先级）。
- **`MX_TIM2_Init()`**：TIM2 4 路 50Hz PWM（PSC=71→1MHz，ARR=19999→20ms）；先写 CCR 再使能输出防乱抖。
- **`HAL_TIM_PWM_MspInit()`**：TIM2 的 PA0–PA3 复用推挽。
- **`MX_TIM3_Init/HAL_TIM_Base_MspInit()`**：TIM3 1ms 基准。
- **`MX_IWDG_Init()`**：独立看门狗（~2s 超时）。
- **`Error_Handler()`**：初始化失败停 PWM+LED 闪烁。

### 7.3 `Core/Src/protocol.c` —— 协议实现
- **`HexNibbleToChar/HexCharToVal`**：十六进制字符互转。
- **`ApStr/ApU32/ApFloat1`**：手工拼字符串/整数/1位小数浮点（避免引入 printf 库，省 Flash）。
- **`PROTO_Init()`**：清解析状态机。
- **`PROTO_FeedByte(b, out)`**：字节流状态机；返回 CONT/OK/CS_ERR/FMT_ERR；自动重新同步帧边界、吞掉超长帧。
- **`PROTO_XorCheck()`**：BODY 的 XOR 校验（与 PC 端一致）。
- **`PROTO_BuildFrame()`**：构建完整帧（`$...*XX\r\n`）。
- **`PROTO_SendPayload()`**：拼帧后用 `HAL_UART_Transmit` 阻塞发送。
- **`PROTO_SendReady/Ack/Done/Pose/Err/Hb`**：各上报帧。
- **`PROTO_ParseCmd()`**：从 BODY 取指令名匹配指令表。
- **`PROTO_CmdName()`**：指令编号→名字。
- **`PROTO_GetArg()`**：取第 n 个字段（缓冲区不足返回 0，不截断）。
- **`PROTO_ArgCount()`**：字段计数。
- **`PROTO_ArgToInt/ToFloat()`**：字符串转整数/浮点（自写，省库）。
- **`PROTO_StrCaseEq()`**：忽略大小写比较。

### 7.4 `Core/Src/servo.c` —— 舵机驱动
- **`Servo_FillDefaultCfg/FillDefaultActions`**：用 main.h 宏填充默认配置/动作表并重算校验。
- **`Servo_CalcChecksum()`**：存储结构体 16 位累加和（跳过前 8 字节魔数/版本/校验）。
- **`Servo_FlashRead/FlashWrite`**：Flash 读 / 先整页擦除再半字编程（关中断防时序抖动）。
- **`Servo_ClampMoveMs()`**：运动时长钳位到合法区间。
- **`Servo_MotionStart()`**：启动一次同步插值（先填参数**最后置 running=1**，防中断读到半新半旧）。
- **`Servo_MotionFinish()`**：强制结束当前运动（急停/超时恢复用）。
- **`Servo_EmitDone()`**：置位 DONE 事件（只保留最近一次）。
- **`Servo_SeqStartPoint()`**：启动动作序列下一个点位。
- **`SERVO_Init()`**：舵机初始化（载入持久化→写 CCR→启 PWM→平滑归位）。
- **`SERVO_Update1ms()`**：1ms 周期任务（推进插值/序列/完成上报）。
- **`SERVO_CheckTimeout()`**：超时与连续堵转检测，冻结+报 `$ERR,04/05`。
- **`SERVO_AngleToPulse()`**：角度→脉宽（含 invert、trim、钳位）。
- **`SERVO_ApplyOutput()`**：当前角刷新到 CCR。
- **`SERVO_CheckLimit()`**：限位检查（软限位+硬件绝对限位+容差）。
- **`SERVO_SetLimits()`**：改限位并写 Flash。
- **`SERVO_GetLimits()`**：读限位。
- **`SERVO_StartSyncMove()`**：四路同步运动（任一越限整条不执行）。
- **`SERVO_MoveSingle()`**：单舵机运动（复用同步框架，其余路保持）。
- **`SERVO_SetGripper()`**：夹爪开合（映射 servo3 两预设角）。
- **`SERVO_GoHome()`**：回 HOME。
- **`SERVO_StartSequence()`**：执行 Flash 预存动作序列（装载前全点位限位校验）。
- **`SERVO_StopNow()`**：急停（冻结目标=当前，保持 PWM 力矩）。
- **`SERVO_IsBusy()`**：是否运动中。
- **`SERVO_TakeDoneEvent()`**：取走 DONE 事件（先读后清）。
- **`SERVO_GetPose()`**：读当前四路角度。
- **`SERVO_GetChannel()`**：读通道运行时状态。
- **`SERVO_SaveCfgToFlash/SaveActionsToFlash`**：写配置/动作表到 Flash。
- **`SERVO_LoadCfgFromFlash/LoadActionsFromFlash`**：三重校验（魔数/版本/和）载入，无效则用默认。
- **`SERVO_ResetToDefault()`**：恢复出厂并写 Flash。
- **`SERVO_GetSlotPointCount()`**：查动作槽有效点数。

### 7.5 `Core/Src/stm32f1xx_it.c` —— 中断
- **`NMI/HardFault/MemManage/BusFault/UsageFault/SVC/DebugMon/PendSV`**：内核异常（保留/死循环保留现场）。
- **`SysTick_Handler()`**：`HAL_IncTick()`（HAL 时基）。
- **`DMA1_Channel5_IRQHandler()`**：`HAL_DMA_IRQHandler`（RX 循环搬运）。
- **`USART1_IRQHandler()`**：判 IDLE→清标志搬运→`HAL_UART_IRQHandler` 处理其它事件。
- **`TIM3_IRQHandler()`**：`HAL_TIM_IRQHandler`→回调 `SERVO_Update1ms()`。

---

## 8. 工具脚本逐函数说明

### 8.1 `tools/calibrate_slots.py` —— 槽位 ROI 标定
- **`find_cjk_font()`**：查找系统中文字体路径。
- **`draw_text_block()`**：在图像上画中文说明块（cv2 中文乱码规避）。
- **`SlotCalibrator`**：鼠标拖拽框选槽位，`on_mouse/confirm_pending/undo/draw` 管理框选与撤销。
- **`save_slots_yaml()`**：保存 slots.yaml。
- **`auto_grid()`**：按行列自动生成网格 ROI（偷懒用）。
- **`_display_reachable()/gui_available()/run_interactive()/parse_args()/main`**：辅助与主流程（支持 `--image` 静态图模式，无摄像头也能用）。

### 8.2 `tools/color_tuner.py` —— HSV 调试
- **`parse_roi/crop_roi/build_mask/overlay_mask`**：ROI 解析/裁剪/生成掩膜/叠加。
- **`_fit_width/_to_view/_hstack/_annotate`**：图像缩放/拼接/标注（适配显示）。
- **`TrackbarUI`**：滑动条 UI（`get/reset`）。
- **`_sanity_warn/format_result/copy_to_clipboard/analyze_region`**：阈值合理性警告/格式化输出/复制到剪贴板/区域分析。

### 8.3 `tools/make_qr_labels.py` —— 标签生成
- **`Medicine.from_dict()`**：药品字典→对象。
- **`find_cjk_font/get_font/_default_font_warning`**：中文字体处理。
- **`_qr_to_bgr/make_label/compose_a4`**：二维码转 BGR / 单标签合成（码+药名+风险红框） / A4 排版。
- **`load_medicines/parse_args/main`**：加载清单、参数、主流程。

### 8.4 `tools/gen_poses.py` —— 点位表生成
- **`slot_positions()`**：由布局算 8 槽位坐标（弧形，朝向基座）。
- **`check_reachable()`**：校验各槽位可达 + 接近角是否适合夹持。
- **`main`**：生成 `config/poses.json` 并打印校验结果（`--check` 仅校验不写）。

### 8.5 `tools/vision_sim.py` —— 视觉仿真
- **`build_inspect_scene()/build_box_scene()`**：合成"过检位/药箱"场景图。
- **`slot_rois()/degrade()`**：槽位 ROI / 退化（模糊/噪声/变暗/旋转/透视/缩小）。
- **`main()`**：跑完整鲁棒性测试并出图。

### 8.6 `tools/kinematics_check.py` —— 运动学验证
- **`fk/ik`**：与 `pc/core/kinematics.py` 同算法（独立实现交叉验证）。
- **`check_solution()`**：逆解回代误差检查。
- **`build_slots()`**：生成布局（grid/arc）。
- **`run()/_plot()`**：可达性判定 + 工作空间可视化图。

---

## 9. 测试与仿真结果

```bash
python tests/test_pipeline.py
# 测试结果: 45/45 通过
```
覆盖：协议（构建/校验/半帧/粘包/脏数据）、运动学（FK/IK 往返、越界、安全区）、知识库、语义映射（感冒/发烧/闲聊）、二维码（多后端融合）、主控全流程（正常取药、高风险确认/取消、抓错放回、空槽、未听懂、动作序列完整性）。

视觉仿真结论：`tools/vision_sim.py` 显示二维码在**噪声/变暗/旋转/透视下 8/8 稳定**，模糊（高斯核 ≥5）与过小（缩放 <0.5）才会下降 → 工程上必须：环形补光常亮、相机手动定焦、码留 ≥4 模块静区、纠错等级最高、多帧投票、过检位离相机近且正对。

运动学结论：弧形布局 8/8 可达，矩形 3×3 仅 7/9 → 药箱布局采用**弧形并让每个槽位朝向基座**（见 `gen_poses.py` 的 `slot_positions`）。

---

## 10. 关键设计点与避坑清单

1. **过检位必须加**：抓取后二次扫码确认，是唯一的安全闭环。
2. **限位放 STM32 侧**：PC 算错也不会打坏舵机/机构。
3. **串口开 `dtr=False, rts=False`**：防 CH340 打开即复位 STM32。
4. **摄像头 `BUFFERSIZE=1` + 关自动参数**：否则读到 200ms 前的旧帧，过检位误判。
5. **舵机独立电源只共 GND**，加 470–1000µF + 0.1µF + PWM 串 220Ω，防动即复位。
6. **多后端融合解码**：单一后端在模糊下全失败，融合（wechat/aruco/builtin 投票）鲁棒得多。
7. **大模型只输出类别 + 白名单校验**：幻觉最多"没听懂"，不会拿错药。
8. **运动到位后延时 300–400ms 再拍**：避开运动模糊。
9. **上电先等 `$READY`**：STM32 没初始化完不发业务指令。
10. **协议绝不"容错执行"**：校验失败整帧丢弃回 ERR。
11. **F1 清 IDLE 必须"先读 SR 再读 DR"**；DMA 环形缓冲需正确计算 CNDTR 与回绕。
12. **Flash 写前整页擦除、关中断**：F103 只能半字编程。
13. **主控三进程/模块分离**：语音、视觉、决策+串口解耦，任一块可独立 mock/降级。
14. **全流程日志落盘**：问题回溯 + 论文测试数据来源。

---

## 11. 部署与答辩要点

- **实施顺序**：先 `gen_poses.py` 验证可达 → 舵机+串口协议跑通 → 预设点位标定（`calibrate_slots.py` 框选 ROI）→ OpenCV 视觉（`vision_sim.py` 验证）→ 语音+大模型+TTS → 联调+过检位 → 稳定性打磨+论文。
- **原则**：先把"机械臂能可靠取放"做扎实（占 60% 风险），视觉/语音可后插、可降级。
- **答辩高频追问**：①"抓错了怎么办？"→过检位二次确认+放回报警。②"没有 ROS 怎么通信？"→自定义 ASCII 帧+ACK/DONE+限位双保险。③"为什么不用 OpenMV？"→PC 本就必须常在线，OpenMV 边缘算力价值归零，反而多故障点。
- **STM32 实测前**：务必在 CubeMX 中确认 TIM2 DMA 未与 USART1 DMA 冲突、Flash 末两页未被程序占用（程序 < 63KB）。
