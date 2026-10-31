# Phase-1 环境与实施指导

> 目标：把一台「刚装完系统的 Windows 电脑」带到**能跑语音全链路彩排**的状态。
> 全程约 40~90 分钟（主要耗时在下载 torch 与 FunASR 模型）。

---

## 1. 解释器选择（本项目最著名的坑）

> 一句话：**Anaconda 只能跑「假语音」的仿真模式；只要涉及真麦克风、真 ASR，
> 就必须用 Python 3.10。**

| 解释器 | 能干什么 | 不能干什么 |
|---|---|---|
| `python` | 全链路（含 FunASR/torch/真麦克风） | —— |
| Anaconda 基础环境 | mock 仿真、PC 逻辑调试 | funasr/torch 装不上，真 ASR 不可用 |

PyCharm 里给项目配 `Python 3.10` 解释器：
`Settings → Project → Python Interpreter → Add → System Interpreter → 选上面的路径`。

## 2. 安装（一条命令）

```bash
# 在 智能药箱系统/ 目录下，用 Python 3.10 跑：
python tools\install_phase1.py
```

装什么：numpy / pyyaml / opencv-python / pyserial / requests / sounddevice /
pyttsx3 / edge-tts / funasr / torch / matplotlib / python-docx。

- 国内网络慢就加镜像：
  `pip install -i https://pypi.tuna.tsinghua.edu.cn/simple <包>`
- torch 已装过或想手动控制版本：`--no-torch`。

## 3. 下模型（FunASR 三件套）

```bash
python tools\dl_models.py
```

| 模型 | 用途 | 体积 |
|---|---|---|
| paraformer-zh | 语音识别主干 | ~1GB |
| fsmn-vad | 语音活动检测（切静音） | ~2MB |
| ct-punc | 标点恢复 | ~1GB |

缓存位置：`~/.cache/modelscope/hub`（改 `MODELSCOPE_CACHE` 可挪）。

## 4. 验收（三连）

```bash
python tools\check_env.py        # 没有阻塞项
python tools\phase1_smoke.py     # Phase-1 冒烟通过
python tools\tts_speaker_test.py # 三句都出声
```

## 5. 配置 LLM（不配也能跑）

```bash
python tools\set_provider.py            # 看当前+预设
python tools\set_provider.py deepseek   # 示例：切 DeepSeek
```

- Key 三选一：app.yaml 内联 / 环境变量 / 不填（自动本地规则）；
- 详细说明见 `docs/模型与API配置指南.md`。

## 6. 药品词库（jieba）

```bash
python tools\patch_jieba.py --test
```

生成 `userdict.txt`（药品名/类别/症状口语）。在代码里
`jieba.load_userdict('userdict.txt')` 即可生效。

## 7. 常见故障速查

| 现象 | 原因 | 处理 |
|---|---|---|
| `No module named 'cv2'` | 选错了解释器（Anaconda） | 换 Python 3.10 |
| ASR 首次跑卡住几分钟 | 在现下模型 | 先跑 `dl_models.py` |
| `HTTP 405` | base_url 少了 `/api/paas/v4` | 用 `set_provider.py` 重置 |
| 麦克风没声音 | 设备被占用（腾讯会议等） | 关掉占用程序，重跑 check_env |
| TTS 不出声 | 没装 pyttsx3 / 无声卡 | `pip install pyttsx3`；`--mock` 验证代码通路 |
| 串口打不开 | CH340 驱动没装 / 口被占 | 装驱动，`check_env.py` 看串口列表 |

## 8. 完成标准

- [ ] `check_env.py` 结论：没有阻塞项
- [ ] `phase1_smoke.py` 结论：Phase-1 冒烟通过
- [ ] `tts_speaker_test.py` 能听到三句中文
- [ ] （可选）`set_provider.py` 已切到想用的供应商且 Key 生效

到此 Phase-1 结束，可以进入硬件联调（见 `docs/运行测试清单.md` 第 4 段）。
