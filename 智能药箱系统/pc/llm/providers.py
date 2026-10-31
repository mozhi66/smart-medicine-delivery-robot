# -*- coding: utf-8 -*-
"""
LLM 供应商预设。
换模型/换供应商只改这里 + app.yaml 的 provider + model 两行。
"""
from typing import Dict, Optional

PRESETS = {
    "zhipu": {
        "base_url": "https://open.bigmodel.cn/api/paas/v4",
        "model": "glm-4-flash",
        "api_key_env": "ZHIPU_API_KEY",
        "doc": "智谱 GLM（OpenAI 兼容接口）",
    },
    "deepseek": {
        "base_url": "https://api.deepseek.com/v1",
        "model": "deepseek-chat",
        "api_key_env": "DEEPSEEK_API_KEY",
        "doc": "深度求索 DeepSeek",
    },
    "dashscope": {
        "base_url": "https://dashscope.aliyuncs.com/compatible-mode/v1",
        "model": "qwen-turbo",
        "api_key_env": "DASHSCOPE_API_KEY",
        "doc": "通义千问（阿里云）",
    },
    "moonshot": {
        "base_url": "https://api.moonshot.cn/v1",
        "model": "moonshot-v1-8k",
        "api_key_env": "MOONSHOT_API_KEY",
        "doc": "Kimi（月之暗面）",
    },
    "siliconflow": {
        "base_url": "https://api.siliconflow.cn/v1",
        "model": "Qwen/Qwen2.5-7B-Instruct",
        "api_key_env": "SILICONFLOW_API_KEY",
        "doc": "硅基流动",
    },
    "openai": {
        "base_url": "https://api.openai.com/v1",
        "model": "gpt-4o-mini",
        "api_key_env": "OPENAI_API_KEY",
        "doc": "OpenAI",
    },
    "ollama": {
        "base_url": "http://localhost:11434/v1",
        "model": "qwen2.5:7b",
        "api_key_env": "",
        "doc": "本地 Ollama",
    },
    "custom": {
        "base_url": "",
        "model": "",
        "api_key_env": "",
        "doc": "自建 OpenAI 兼容端点",
    },
}

DEFAULT_PROVIDER = "zhipu"


def resolve(provider=None, base_url=None, model=None, api_key=None,
            api_key_env=None):
    """
    返回合并后的配置 dict（09-29 版行为，供 GLMClient 的 provider 层消费）。

    键位约定：
      provider   : 生效的供应商名
      base_url   : 接口根地址（OpenAI 兼容，不带 /chat/completions）
      endpoint   : base_url + '/chat/completions'（直接可 POST 的全地址）
      model      : 生效的模型名
      api_key    : 显式密钥（没传则空串，由调用方按 key_env 兜底）
      key_env    : 应该从哪个环境变量读密钥
      doc/label  : 人类可读描述
    """
    name = provider or DEFAULT_PROVIDER
    p = PRESETS.get(name, PRESETS[DEFAULT_PROVIDER])
    root = (base_url or p["base_url"] or "").rstrip("/")
    key_env = api_key_env or p.get("api_key_env") or ""
    return {
        "provider": name,
        "base_url": root,
        "endpoint": (root + "/chat/completions") if root else "",
        "model": model or p["model"],
        "api_key": api_key or "",
        "key_env": key_env,
        "api_key_env": key_env,
        "doc": p.get("doc") or p.get("label") or "",
        "label": p.get("label") or p.get("doc") or "",
        "note": p.get("note") or "",
    }
