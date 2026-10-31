# 模型与 API 配置指南

> 智能药箱 · 四轮麦轮融合版 —— LLM 多供应商配置
> 相关代码：`pc/llm/providers.py`（预设）、`pc/llm/glm_client.py`（客户端）、
> `config/app.yaml`（配置）、`tools/set_provider.py`（一键切换）

---

## 1. 这张表在管什么

上位机的「症状 → 药品类别」语义映射走云端大模型（OpenAI 兼容接口）。
**换供应商只改两处**：`config/app.yaml` 的 `llm.provider` + `llm.model` 两行
（或直接用 `python tools/set_provider.py <供应商名>` 一条命令）。

安全边界（写在 glm_client.py 的最重要设计）：

1. 大模型**只输出类别**，绝不输出槽位号或具体药名——防幻觉拿错药；
2. 返回类别必须通过本地白名单校验，否则判 unknown 请用户重说；
3. 网络失败/超时/限流都有重试与兜底，最多是「没听懂」，不会拖垮系统；
4. **没配 Key 也能跑**——自动回退本地关键词规则（mock），离线可彩排。

## 2. 可用供应商预设

| 供应商 | provider 名 | 默认模型 | Key 环境变量 | 说明 |
|---|---|---|---|---|
| 智谱 GLM | `zhipu` | glm-4-flash | `ZHIPU_API_KEY` | 免费快，分类任务够用（默认） |
| DeepSeek | `deepseek` | deepseek-chat | `DEEPSEEK_API_KEY` | 便宜量大 |
| 通义千问 | `dashscope` | qwen-turbo | `DASHSCOPE_API_KEY` | 阿里云 |
| Kimi | `moonshot` | moonshot-v1-8k | `MOONSHOT_API_KEY` | 月之暗面 |
| 硅基流动 | `siliconflow` | Qwen/Qwen2.5-7B-Instruct | `SILICONFLOW_API_KEY` | 聚合平台 |
| OpenAI | `openai` | gpt-4o-mini | `OPENAI_API_KEY` | 需境外网络 |
| 本地 Ollama | `ollama` | qwen2.5:7b | （不需要） | 先 `ollama pull qwen2.5:7b` |
| 自建端点 | `custom` | （自己填） | （自己填） | vLLM / one-api 等 |

预设的完整定义在 `pc/llm/providers.py` 的 `PRESETS` 字典——
**加新供应商只改这里**，客户端与 app.yaml 自动跟上。

## 3. app.yaml 的 llm 段（当前配置）

```yaml
llm:
  provider: zhipu                # 供应商预设名
  base_url: https://open.bigmodel.cn/api/paas/v4   # 接口根地址（不带 /chat/completions）
  api_key: 55a7...               # 内联 Key；留空则读下面环境变量
  api_key_env: ZHIPU_API_KEY     # api_key 为空时才读这个环境变量
  model: glm-4-flash             # 模型名
  timeout: 45                    # 单次请求超时（秒）
  retries: 2                     # 失败重试次数
```

**Key 的三种填法**（按优先级）：

1. `app.yaml` 的 `api_key` 内联（答辩演示最省事，别把文件传到公网）；
2. 环境变量 `api_key_env` 指定的变量（推荐日常开发）；
3. 都不填 → 自动走本地 mock 规则，程序仍可完整运行。

## 4. 一键切换

```bash
# 看当前配置 + 预设列表
python tools/set_provider.py

# 切到 DeepSeek（自动改 provider/base_url/model/api_key_env 四行）
python tools/set_provider.py deepseek

# 切换同时微调模型
python tools/set_provider.py dashscope --model qwen-plus

# 只写 Key，不动供应商
python tools/set_provider.py --set-key sk-xxxxxxxx
```

切换后**密钥不动**（不属于预设，避免误覆盖）。

## 5. 接口地址的坑（重要）

- `base_url` 是**根地址**，不要带 `/chat/completions`——
  客户端会自己拼（见 providers.py 的 `resolve()`）。
- 智谱的正确根地址是 `https://open.bigmodel.cn/api/paas/v4`；
  **少了 `/api/paas/v4` 会打到官网页面接口 → HTTP 405**。
- 自建端点（vLLM/one-api）填到 `http://<ip>:<port>/v1` 即可。

## 6. 验收

```bash
# 1) 看当前生效配置（不联网）
python tools/set_provider.py

# 2) 端到端验证：本地规则 + 云端各跑一次
python tools/phase1_smoke.py --offline     # 强制本地规则
python tools/phase1_smoke.py               # 有 Key 就走云端

# 3) 期望看到
#    [ OK ] 语义映射: 「感冒」-> 感冒药（source=mock 或 llm）
```

没 Key 时第 2 步会提示「本次只验证本地规则」并自动降级——
这是设计行为，不是故障。
