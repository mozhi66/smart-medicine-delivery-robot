#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 云端大模型语义映射 (glm_client.py)
================================================================================
职责：把用户的自然语言（"我感冒了""头有点疼"）映射成本地知识库里的【药品类别】。

安全设计（这是本模块最重要的部分）
----------------------------------
1. **只让大模型输出"类别"，绝不让它输出槽位号或具体药名。**
   大模型会幻觉。它如果直接说"3号槽位"，出错就是拿错药；
   它只说"感冒药"，再由本地知识库查具体药品，出错最多是没听懂。

2. **强制 JSON 输出 + 白名单校验。**
   大模型返回的类别必须出现在本地类别列表里，否则判为 unknown，
   请用户再说一遍。绝不允许把未知类别拿去查库。

3. **网络失败不能拖垮系统。**
   超时、限流、返回垃圾数据，都有重试和兜底，最多是"没听懂"。

4. **mock 模式**：不联网也能跑通全流程（关键词规则 + 模糊匹配），
   用于离线仿真和答辩前预演。

模型：智谱 GLM-4-Flash（免费、响应快，适合这种简单的分类任务）
接口：https://open.bigmodel.cn/api/paas/v4/chat/completions
================================================================================
"""

from __future__ import annotations

import difflib
import json
import os
import re
import time
from typing import Dict, List, Optional, Sequence

from llm.providers import resolve, DEFAULT_PROVIDER

try:
    import requests
    _HAS_REQUESTS = True
except Exception:                      # pragma: no cover
    requests = None
    _HAS_REQUESTS = False


API_URL = "https://open.bigmodel.cn/api/paas/v4/chat/completions"

SYSTEM_PROMPT = """你是一个智能药箱的语义理解模块。
用户会用口语描述自己的不适或需求，你的任务是把他说的映射到【给定的药品类别列表】中的一个。

严格要求：
1. 只能从候选类别列表中选择，绝对不能自己编造新类别。
2. 如果用户的描述确实无法对应到任何候选类别（比如闲聊、问天气），就返回 intent="chitchat"。
3. 如果用户提到了药品需求但候选里没有合适的，返回 intent="unknown"。
4. 必须只输出一个 JSON 对象，不要输出任何解释、不要 markdown 代码块。

输出格式（严格）：
{"intent":"medicine","category":"<候选类别之一>","reason":"<一句话说明，10字以内>"}
或
{"intent":"chitchat","category":null,"reason":"<一句话说明>"}
或
{"intent":"unknown","category":null,"reason":"<一句话说明>"}

intent 只能是 medicine / chitchat / unknown 三者之一。"""


class GLMClient:
    """
    智谱 GLM 语义映射客户端。

    用法
    ----
        cli = GLMClient(api_key="xxx", categories=["感冒药", "退烧药"])
        r = cli.map_symptom("我感冒了，鼻子不通气")
        # r = {'intent':'medicine', 'category':'感冒药', 'reason':'...', 'ok':True}
    """

    def __init__(self, api_key: Optional[str] = None,
                 model: Optional[str] = None,
                 timeout: float = 12.0, retries: int = 2,
                 categories: Optional[Sequence[str]] = None,
                 mock: bool = False, verbose: bool = True,
                 # ↓↓↓ 新增（全部可选，不传就是旧行为）↓↓↓
                 provider: Optional[str] = None,
                 base_url: Optional[str] = None,
                 api_key_env: Optional[str] = None,
                 prompt_template: Optional[str] = None):
        """
        参数
        ----
        api_key       : 密钥。None 时按顺序从 api_key_env → 供应商默认环境变量读。
        model         : 模型名。None 时用供应商预设的默认模型。
        timeout       : 单次请求超时秒数
        retries       : 失败重试次数
        categories    : 本地药品类别白名单（必填，用于约束与校验输出）
        mock          : True 时不联网，用本地关键词规则（离线仿真用）
        verbose       : 是否打印日志
        provider      : 供应商预设名（zhipu/deepseek/dashscope/…），见 providers.py
        base_url      : 接口根地址；给了就覆盖 provider 预设
        api_key_env   : 从哪个环境变量读密钥；给了就覆盖 provider 预设
        prompt_template : 自定义 system prompt；None 用内置 SYSTEM_PROMPT
        """
        # ---- 解析供应商（显式入参 > 预设 > 默认智谱）----
        cfg = resolve(provider=provider or DEFAULT_PROVIDER,
                      base_url=base_url, model=model,
                      api_key=api_key, api_key_env=api_key_env)

        # 密钥兜底：显式 key > 指定环境变量 > 预设环境变量 > 老的 ZHIPU_API_KEY
        key = cfg["api_key"] or os.environ.get(cfg["key_env"], "")
        if not key:
            key = os.environ.get("ZHIPU_API_KEY", "")

        self.provider = cfg["provider"]
        self.base_url = cfg["base_url"]
        self.endpoint = cfg["endpoint"] or API_URL
        self.api_key_env = cfg["key_env"]
        self.api_key = key
        self.model = cfg["model"] or "glm-4-flash"
        self.prompt_template = prompt_template or SYSTEM_PROMPT

        self.timeout = float(timeout)
        self.retries = int(retries)
        self.categories: List[str] = list(categories or [])
        self.mock = bool(mock) or (not _HAS_REQUESTS)
        self.verbose = bool(verbose)
        self.last_raw: str = ""


    # ==================================================================
    # 主入口
    # ==================================================================
    def map_symptom(self, text: str) -> Dict:
        """
        把用户说的话映射成药品类别。

        参数
        ----
        text : 用户说的话，如 "我感冒了"

        返回
        ----
        dict:
          ok       : bool  是否成功得到可用结果
          intent   : 'medicine' | 'chitchat' | 'unknown'
          category : 药品类别字符串或 None（一定是白名单里的合法值）
          reason   : 大模型给出的简短理由
          source   : 'glm' | 'mock' | 'fallback'，便于日志排查
        """
        text = (text or "").strip()
        if not text:
            return self._result(False, "unknown", None, "空输入", "fallback")

        if self.mock:
            return self._mock_map(text)

        prompt = self._build_user_prompt(text)
        for attempt in range(self.retries + 1):
            raw = self._call_api(prompt)
            if raw is None:
                if self.verbose:
                    print("[GLM] 第 %d 次调用失败，重试..." % (attempt + 1))
                time.sleep(0.6 * (attempt + 1))
                continue
            parsed = self._parse_json(raw)
            if parsed is None:
                continue
            cat = parsed.get("category")
            intent = str(parsed.get("intent", "unknown")).strip().lower()

            # ---- 白名单校验：这是安全的核心 ----
            if intent == "medicine":
                if cat in self.categories:
                    return self._result(True, "medicine", cat,
                                        parsed.get("reason", ""), "glm")
                # 大模型可能返回了近似但不完全一致的类别，做一次模糊匹配
                near = self._fuzzy_category(str(cat))
                if near:
                    return self._result(True, "medicine", near,
                                        "近似匹配:%s" % cat, "glm")
                return self._result(False, "unknown", None,
                                    "类别 '%s' 不在药品库中" % cat, "glm")
            if intent == "chitchat":
                return self._result(True, "chitchat", None,
                                    parsed.get("reason", ""), "glm")
            return self._result(False, "unknown", None,
                                parsed.get("reason", ""), "glm")

        return self._result(False, "unknown", None, "调用大模型失败", "fallback")

    # ==================================================================
    # 内部：构造 prompt / 调 API / 解析
    # ==================================================================
    def _build_user_prompt(self, text: str) -> str:
        """构造用户侧 prompt：候选类别 + 用户原话"""
        cands = "、".join(self.categories) if self.categories else "（无）"
        return ("候选药品类别列表：%s\n\n"
                "用户说的话：%s\n\n"
                "请输出 JSON：" % (cands, text))

    def _call_api(self, user_prompt: str) -> Optional[str]:
        """
        调用智谱 HTTP 接口，返回模型输出的文本；失败返回 None。
        """
        if not _HAS_REQUESTS:
            return None
        if not self.api_key:
            if self.verbose:
                print("[GLM] 未设置 API Key（环境变量 ZHIPU_API_KEY），已降级为本地规则")
            self.mock = True
            return None
        headers = {
            "Authorization": "Bearer %s" % self.api_key,
            "Content-Type": "application/json",
        }
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": user_prompt},
            ],
            "temperature": 0.2,       # 分类任务要稳定，不要创造力
            "top_p": 0.7,
            "max_tokens": 256,
        }
        try:
            resp = requests.post(self.endpoint or API_URL, headers=headers,
                                 json=payload, timeout=self.timeout)
        except Exception as e:
            if self.verbose:
                print("[GLM] 网络异常: %s" % e)
            return None

        if resp.status_code != 200:
            if self.verbose:
                print("[GLM] HTTP %d: %s" % (resp.status_code, resp.text[:200]))
            return None
        try:
            data = resp.json()
            content = data["choices"][0]["message"]["content"]
        except Exception as e:
            if self.verbose:
                print("[GLM] 解析响应失败: %s" % e)
            return None

        self.last_raw = content
        return content

    @staticmethod
    def _parse_json(raw: str) -> Optional[dict]:
        """
        从模型输出里抠出 JSON 对象。
        模型时不时会在 JSON 外面包 ```json 或说两句废话，这里做兼容。
        """
        if not raw:
            return None
        s = raw.strip()
        # 去掉 markdown 代码块
        s = re.sub(r"^```(?:json)?", "", s).strip()
        s = re.sub(r"```$", "", s).strip()
        # 直接解析
        try:
            return json.loads(s)
        except Exception:
            pass
        # 抠第一个 {...}
        m = re.search(r"\{[^{}]*\}", s, flags=re.S)
        if m:
            try:
                return json.loads(m.group(0))
            except Exception:
                pass
        return None

    def _fuzzy_category(self, cat: str, cutoff: float = 0.6) -> Optional[str]:
        """用字符串相似度把近似类别纠正到白名单内的合法值"""
        if not self.categories or not cat:
            return None
        m = difflib.get_close_matches(cat, self.categories, n=1, cutoff=cutoff)
        return m[0] if m else None

    # ==================================================================
    # 内部：mock（离线规则映射）
    # ==================================================================
    _MOCK_RULES = {
        "感冒药": ["感冒", "着凉", "流鼻涕", "鼻塞", "打喷嚏", "鼻子不通", "伤风"],
        "退烧药": ["发烧", "发热", "体温高", "烧到", "退烧"],
        "止咳药": ["咳嗽", "咳", "嗓子痒", "干咳"],
        "止痛药": ["头疼", "头痛", "牙疼", "牙痛", "疼", "痛"],
        "肠胃药": ["肚子疼", "胃疼", "拉肚子", "腹泻", "消化不良", "胃不舒服", "恶心"],
        "消炎药": ["发炎", "消炎", "嗓子疼", "扁桃体"],
        "抗过敏药": ["过敏", "起疹", "皮肤痒", "打喷嚏过敏"],
        "外用消毒": ["擦伤", "外伤", "消毒", "破皮", "碘伏"],
    }

    def _mock_map(self, text: str) -> Dict:
        """
        离线规则映射：按关键词打分，命中最多关键词的类别胜出。
        用于没有网络/没有 API Key 时跑通全流程。
        """
        best_cat, best_hit = None, 0
        for cat, kws in self._MOCK_RULES.items():
            if cat not in self.categories:
                continue
            hit = sum(1 for k in kws if k in text)
            if hit > best_hit:
                best_cat, best_hit = cat, hit
        if best_cat:
            return self._result(True, "medicine", best_cat,
                                "本地规则命中%d个关键词" % best_hit, "mock")
        return self._result(False, "unknown", None, "本地规则未匹配", "mock")

    # ==================================================================
    @staticmethod
    def _result(ok: bool, intent: str, category: Optional[str],
                reason: str, source: str) -> Dict:
        """统一结果结构"""
        return {"ok": bool(ok), "intent": intent, "category": category,
                "reason": reason, "source": source}
