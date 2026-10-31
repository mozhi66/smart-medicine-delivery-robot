# 智能药箱 · 四轮麦轮融合版

一套"语音 → 大模型语义理解 → 视觉确认 → 机械臂取药 → 麦轮底盘送药"的智能药箱系统。
上位机跑 Python，下位机是 STM32F103 裸机固件（Keil SPL 工程），两端通过 ASCII 帧串口协议通信。

> 无 ROS 依赖。语音用 FunASR，语义映射用 GLM-4-Flash（OpenAI 兼容协议），
> 视觉用 OpenCV，全部在普通 PC 上跑通。

---

## 功能

- **语音取药**：说"我感冒了"，ASR 转写 → 大模型映射到药品 → 知识库定位槽位 → 机械臂取药 → TTS 播报
- **视觉二次确认**：抓取后先送到固定扫码位复核二维码，一致才交付，不一致自动放回并语音报警
- **两种底盘模式**：`auto` 由 PC 发 `$MV` 定时横移 + 列码校正；`manual_phone` 由人用手机经 HC-05 控车
- **失联自停**：连续运动受 deadman 保护（AUTO 800ms / MANUAL 3000ms 无指令即停）
- **高风险药二次确认**：`high_risk=true` 的药品取用前必须语音确认

## 系统架构

```
用户语音
   │
   ▼
┌──────────────────── PC（Python 3.10）────────────────────┐
│  ASR(FunASR) → LLM(GLM-4-Flash) → 知识库(药品/槽位)        │
│         │                                                │
│         ▼                                                │
│  OpenCV(USB 摄像头) ── 二维码 / 色块 / 帧差 ── 身份+有无    │
│         │                                                │
│         ▼                                                │
│  点位表 PoseTable → 4 舵机角度序列 → 串口下发              │
└───────────────────────┬──────────────────────────────────┘
                        │ USB 串口 115200/8N1
                        │ ASCII 帧 $CMD,args*XOR\r\n
                        ▼
        STM32F103C8T6（只做执行：收角度→驱动舵机；收速度→驱动麦轮）
        TIM2 四路 PWM(50Hz) + DMA+IDLE 收串口 + TIM3 1ms 插值
```

STM32 不做视觉、不做逆解、不做决策，只可靠执行并回 `$ACK` / `$DONE` / `$CHS` / `$HB`。

## 目录结构

```
.
├── 0stm/                          STM32 固件（Keil MDK / 标准外设库）
│   ├── app/
│   │   ├── main.h                 舵机与系统参数
│   │   ├── servo.c/.h             4 路舵机 PWM + 1ms 插值
│   │   ├── chassis.c/.h           麦轮运动学 + 定时运动 + 失联自停
│   │   └── protocol.c/.h          帧解析 / 校验 / 指令分发
│   ├── user/main.c                主循环 + HC-05 单字符手动通道
│   ├── my_lib/                    OLED / I2C / SPI / USART / 按键驱动
│   ├── std_periph_driver/         STM32F10x 标准外设库
│   ├── startup/                   启动文件
│   ├── template.uvprojx/.uvoptx   Keil 工程
│   └── tools_firmware_static_check.py   固件静态检查脚本
│
├── 智能药箱系统/                   上位机（Python）
│   ├── pc/
│   │   ├── main.py                主控入口：装配 + 主循环
│   │   ├── audio/                 ASR(FunASR) / TTS
│   │   ├── comm/                  串口协议 / 机械臂与底盘控制器
│   │   ├── core/                  状态机 / 运动学 / 点位表 / 药品知识库 / 日志
│   │   ├── llm/                   大模型客户端与多供应商预设
│   │   └── vision/                摄像头 / 二维码 / 扫码窗 / GUI
│   ├── config/                    app.yaml、medicines.json、poses.json
│   ├── docs/                      环境搭建、联调、审查报告
│   ├── tests/                     端到端流水线测试
│   └── tools/                     标定、检查、二维码标签生成等工具
│
├── 融合说明.md                     两版工程合并的取舍、协议权威定义、运动模型
├── 从零上手与硬件接线指南.md        接线与上电顺序（见下方"已知缺口"）
└── 项目说明_四轮麦轮融合版.docx      项目说明文档
```

## 硬件清单

| 部件 | 说明 |
|---|---|
| 主控 | STM32F103C8T6 最小系统板 |
| 机械臂 | 4 路舵机（s0 底座 / s1 / s2 / s3 夹爪），**独立 5–6V ≥5A 供电，只与主控共 GND** |
| 底盘 | 四轮麦克纳姆轮 + TB6612 双路电机驱动（**STBY 必须拉高**） |
| 串口 | CH340 USB 转串口，115200 8N1 |
| 蓝牙 | HC-05（手机手动控车通道） |
| 视觉 | USB 摄像头，固定于药箱前上方 |

## 快速开始

### 上位机

```bash
cd 智能药箱系统
python -m pip install -r requirements.txt

# 看可用串口
python pc/main.py --list-ports

# 纯软件跑通（不接任何硬件）
python pc/main.py --mock --once --text "我感冒了"

# 真机联调
python pc/main.py --port COM3 --mode manual_phone \
    --real-mic --real-llm --real-tts --real-cam --show
```

`config/app.yaml` 里改串口号与相机索引。首次跑 FunASR 会从 ModelScope 下载模型（数百 MB）。

### 固件

1. Keil MDK 打开 `0stm/template.uvprojx`
2. `Options for Target → Debug` 选 ST-Link Debugger，`Utilities → Settings → Flash Download` 勾 **Reset and Run**
3. 编译下载，板子复位后 LED(PC13) 1Hz 闪烁、串口输出 `$READY` + `$HB`

## 通信协议

帧格式：`$CMD,args*XOR\r\n`（XOR 为 `$` 与 `*` 之间所有字节异或）

| 指令 | 格式 | 语义 |
|---|---|---|
| `SA` | `$SA,id,angle,dur` | 单舵机运动 |
| `SS` | `$SS,id,angle,dur` | 舵机立即到位 |
| `GC` | `$GC,angle` | 夹爪闭合到指定角度 |
| `RUN` | `$RUN,n` | 执行预设动作序列 |
| `HOM` | `$HOM` | 归位 |
| `QRY` / `EST` / `CFG` | — | 查询状态 / 急停 / 写限位 |
| `MD` | `$MD,AUTO\|MAN\|Q` | 工作模式（Q 只查询） |
| `MV` | `$MV,y,x,rx[,tt]` | 底盘运动，y/x/rx∈[-1,1]，tt 为毫秒（省略或 0 = 连续） |
| `MS` | `$MS` | 立即停车 |
| `MC` | `$MC,pct` | 速度上限 0~100（0 = 锁死） |
| `MQ` | `$MQ` | 查询 → `$CHS,mode,moving,y,x,rx,pct` |

麦轮逆解（固定）：`FL=y+x+rx  FR=y-x-rx  RL=y-x+rx  RR=y+x-rx`

底盘运动帧**只回 `$ACK` + `$CHS`，不回 `$DONE`**。

手机 HC-05 单字符：`F B L R Q E Z C G H S + -`

## 配置与密钥

`config/app.yaml` 中的 `api_key` 字段**留空**，密钥通过环境变量注入：

```bash
set ZHIPU_API_KEY=你的key          # Windows
export ZHIPU_API_KEY=你的key       # Linux / macOS
```

支持的供应商预设（见 `pc/llm/providers.py`）：智谱 GLM、DeepSeek、通义千问、Kimi、SiliconFlow、OpenAI。
切换用 `python tools/set_provider.py deepseek`。

> ⚠ 不要把填了真实 key 的 `app.yaml` 提交到仓库。仓库的 `.gitignore` 已屏蔽 `.env`、`*.local.yaml`。

## 已知缺口

如实记录，避免误导：

- **`从零上手与硬件接线指南.md` 内容不完整** —— 仅保留了"Keil 烧录与调试"一节，接线总表部分缺失。
- **`智能药箱系统/docs/` 下部分文档为摘录或重建版本**，章节不完整：
  `PyCharm使用说明.md`（碎片摘录）、`phase1_环境与实施指导.md`、`模型与API配置指南.md`、
  `运行测试清单.md`、`代码审查报告_20260922.md`（仅前 150 行）。
- **`tools/camera_check.py` 与 `tools/camera_preview.py` 为片段留存**，不含完整入口，
  无法独立运行；`pc/main.py` 中"先跑 `tools/camera_check.py --list`"的提示目前无法照做，
  请直接用 `python pc/main.py --list-ports` 查串口。
- `融合说明.md` 第 5 节目录树中提到的 `硬件连接总表.md`、`换板子检查清单.md`、`0底盘/` 目录，在当前工程中**不存在**。
- **字体资源未随仓库分发** —— `0stm/my_lib/font/ttfttc/` 下的华文仿宋、等线、黑体等为商业字体，且有版权限制，故未收录；OLED 显示使用 `my_lib/oled_font.h` 内置字库，不影响编译。
- **固件未在 CI 中编译验证** —— 需 Windows + Keil MDK 环境；仓库内仅提供静态检查脚本 `tools_firmware_static_check.py`。
- 机械臂位姿表 `config/poses.json` 的数值来自真机微调，换硬件需重新标定（`tools/pose_demo.py`、`tools/calibrate_slots.py`）。

## 文档索引

| 文档 | 内容 |
|---|---|
| `融合说明.md` | 两版工程合并取舍、协议权威定义、运动模型、两种模式差异 |
| `从零上手与硬件接线指南.md` | Keil 烧录、串口连通性排查、舵机标定流程（部分） |
| `智能药箱系统/README.md` | 上位机完整说明：架构、模块、测试结论 |
| `智能药箱系统/docs/` | 环境搭建、模型与 API 配置、联调记录、代码审查报告 |
| `项目说明_四轮麦轮融合版.docx` | 项目说明文档 |

## 许可

未指定开源许可证。如需公开复用，请先与作者联系。
