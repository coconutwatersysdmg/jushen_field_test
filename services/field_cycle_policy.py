# -*- coding: utf-8 -*-
"""现场离线装货循环步骤定义。"""
from __future__ import annotations


FIELD_FIRST_STEPS = (
    ("DEVICE_CHECK", "1. 外接设备连接检查（PLC / 雷达 / 相机）"),
    ("FIELD_SENSE", "2. 相机插孔识别 ∥ 雷达底板四角粗定位"),
    ("FIELD_CORNER_SHELL", "3. 相机精定位并按 P1/P2 → P3/P4 划格"),
    ("FIELD_WAIT_MANUAL_PLACE", "4. 等待人工放置 B1（确认后拍照检测）"),
    ("FIELD_PLACE_VERIFY", "4. 读取 B1 放货照片并判区"),
)

FIELD_REPEAT_STEPS = (
    ("FIELD_RETURN_ORIGIN", "1. 返回原点（仅 XY，Z/R 现场保持）"),
    ("FIELD_SENSE", "2. 离线识别插孔并展示坐标（不插取）"),
    ("FIELD_PRE_PLACE_MONITOR", "3. 读取上一 B 区离线照片监测"),
    ("FIELD_WAIT_MANUAL_PLACE", "4. 等待人工放置当前 B 区（确认后拍照检测）"),
    ("FIELD_PLACE_VERIFY", "4. 读取当前 B 区放货照片并判区"),
)


def field_steps_for_round(round_index: int):
    """返回现场当前轮次的步骤。"""
    return cycle_steps_for_round(round_index)


def cycle_steps_for_round(round_index: int):
    return FIELD_FIRST_STEPS if int(round_index) == 0 else FIELD_REPEAT_STEPS
