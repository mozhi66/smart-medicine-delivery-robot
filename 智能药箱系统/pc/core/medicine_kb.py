#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 本地药品知识库 (medicine_kb.py)
================================================================================
职责：管理"药品 <-> 槽位"的对应关系，是整个系统的权威数据源。

设计原则（很重要）
------------------
云端大模型只负责把"我感冒了"理解成"感冒药"这一层【语义映射】，
具体吃哪一种药、在哪个槽位、是不是高风险，一律以本地知识库为准。
原因：大模型会幻觉。让它直接输出槽位号，出错就是拿错药；
让它只输出"类别"，再由本地库查具体药品，出错最多是"没听懂"，
可以请用户再说一遍 —— 这是安全性上本质的差别。

medicines.json 结构
-------------------
{
  "medicines": [
    {"code": "MED001", "name": "感冒灵颗粒", "category": "感冒药",
     "slot": 1, "high_risk": false, "stock": 12, "note": "饭后服用，一次1袋"}
  ]
}
================================================================================
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class Medicine:
    """一种药品"""
    code: str                       # 二维码内容，如 MED001，全局唯一
    name: str                       # 药品名，如 "感冒灵颗粒"
    category: str                   # 类别，如 "感冒药"（大模型只输出到这一层）
    slot: int                       # 所在槽位号（从 1 开始）
    high_risk: bool = False         # 是否高风险药（需要二次确认）
    stock: int = 0                  # 剩余数量
    note: str = ""                  # 服用提示，会由 TTS 播报

    def __repr__(self) -> str:
        return "Medicine(%s %s 槽位%d%s)" % (
            self.code, self.name, self.slot, " [高风险]" if self.high_risk else "")


class MedicineKB:
    """
    药品知识库。

    用法
    ----
        kb = MedicineKB.load("config/medicines.json")
        kb.categories()                  # ['感冒药', '退烧药', ...] 交给大模型做候选
        m = kb.pick_one("感冒药")         # 挑一个库存>0 的
        kb.by_code("MED003")             # 扫码后反查药品
        kb.by_slot(3)                    # 看某个槽位放的是什么
    """

    def __init__(self, medicines: Optional[List[Medicine]] = None):
        self.medicines: List[Medicine] = list(medicines or [])
        self._by_code: Dict[str, Medicine] = {}
        self._by_slot: Dict[int, Medicine] = {}
        self._reindex()

    # ------------------------------------------------------------------
    @staticmethod
    def load(path: str) -> "MedicineKB":
        """
        从 JSON 文件加载。文件不存在时返回空库（不抛异常）。
        """
        if not os.path.isfile(path):
            return MedicineKB([])
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        items = []
        for d in data.get("medicines", []) or []:
            try:
                items.append(Medicine(
                    code=str(d["code"]),
                    name=str(d.get("name", d["code"])),
                    category=str(d.get("category", "")),
                    slot=int(d.get("slot", 0)),
                    high_risk=bool(d.get("high_risk", False)),
                    stock=int(d.get("stock", 0)),
                    note=str(d.get("note", "")),
                ))
            except (KeyError, ValueError) as e:
                print("[警告] 药品数据格式错误，已跳过: %s (%s)" % (d, e))
        return MedicineKB(items)

    def save(self, path: str) -> None:
        """保存回 JSON（库存变化后要落盘）"""
        os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
        data = {"medicines": [
            {"code": m.code, "name": m.name, "category": m.category,
             "slot": m.slot, "high_risk": m.high_risk,
             "stock": m.stock, "note": m.note}
            for m in self.medicines
        ]}
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    # ------------------------------------------------------------------
    def _reindex(self) -> None:
        """重建 code / slot 两个索引"""
        self._by_code = {m.code: m for m in self.medicines}
        self._by_slot = {m.slot: m for m in self.medicines}

    def add(self, m: Medicine) -> None:
        """新增一种药品"""
        self.medicines.append(m)
        self._reindex()

    def all(self) -> List[Medicine]:
        """所有药品"""
        return list(self.medicines)

    def categories(self) -> List[str]:
        """
        所有药品类别（去重、保持顺序）。
        这个列表要传给大模型做候选集，把输出约束在已知范围内。
        """
        out, seen = [], set()
        for m in self.medicines:
            if m.category and m.category not in seen:
                seen.add(m.category)
                out.append(m.category)
        return out

    # ------------------------------------------------------------------
    def by_code(self, code: str) -> Optional[Medicine]:
        """按二维码内容反查药品（扫码确认身份时用）"""
        return self._by_code.get(str(code).strip())

    def by_slot(self, slot: int) -> Optional[Medicine]:
        """按槽位号查药品（判断"这个位置应该有什么药"）"""
        return self._by_slot.get(int(slot))

    def by_category(self, category: str) -> List[Medicine]:
        """按类别查所有药品（可能有多种，如感冒药有颗粒和胶囊）"""
        return [m for m in self.medicines if m.category == category]

    # ------------------------------------------------------------------
    def pick_one(self, category: str) -> Optional[Medicine]:
        """
        从某类别里挑一个可用的药品。

        挑选策略：优先库存 > 0 的；同类多个时取槽位号最小的（行为可预测）。
        全部缺货时返回 None。
        """
        cands = self.by_category(category)
        if not cands:
            return None
        in_stock = [m for m in cands if m.stock > 0]
        pool = in_stock or cands
        return sorted(pool, key=lambda m: m.slot)[0]

    def find_by_text(self, text: str) -> Optional[Medicine]:
        """
        药名直取：在一句话里找完整药名，命中则直接锁定该药品。

        规则
        ----
        - 句子（去空格/标点）包含完整药名即命中，如
          "我要拿感冒灵颗粒" -> MED001；"肚子疼" -> None（走症状映射）。
        - 多个药名同时命中时取名字最长的（防止短名抢先）。
        - 不看库存：命中缺货药品时由调用方播报"没有库存"。

        为什么不依赖大模型
        ------------------
        ASR 热词里已有全部药名（识别可靠），本地字符串匹配零延迟、
        离线可用，也不占用"症状 -> 类别"分类通道的容错空间。
        """
        if not text:
            return None
        t = str(text)
        for ch in (" ", "，", "。", "！", "？", "、", "\t"):
            t = t.replace(ch, "")
        best: Optional[Medicine] = None
        best_len = 0
        for m in self.medicines:
            name = "".join(str(m.name or "").split())
            # 名字至少 2 字，避免单字误命中；最长优先
            if len(name) >= 2 and name in t and len(name) > best_len:
                best = m
                best_len = len(name)
        return best

    def consume(self, code: str, n: int = 1) -> bool:
        """
        取药成功后扣减库存。

        返回
        ----
        扣减是否成功（库存不足或药品不存在返回 False）
        """
        m = self.by_code(code)
        if m is None or m.stock < n:
            return False
        m.stock -= n
        return True

    def is_valid_category(self, category: str) -> bool:
        """类别是否在已知范围内（大模型输出的白名单校验）"""
        return category in self.categories()
