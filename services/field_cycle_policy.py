# -*- coding: utf-8 -*-
"""现场离线联调的装货循环步骤定义。"""
from __future__ import annotations


FIELD_FIRST_STEPS = (
    ("DEVICE_CHECK", "1. 外接设备连接检查（PLC / 雷达 / 相机）"),
    ("LAB_SENSE", "2. 相机插孔识别 ∥ 雷达底板四角粗定位"),
    ("LAB_CORNER_SHELL", "3. 相机精定位并按 P1/P2 → P3/P4 划格"),
    ("LAB_WAIT_MANUAL_PLACE", "4. 等待人工放置 B1（确认后拍照检测）"),
    ("LAB_PLACE_VERIFY", "5. 到 B1 中心实拍并判区"),
)

FIELD_REPEAT_STEPS = (
    ("LAB_RETURN_ORIGIN", "1. 返回原点（仅 XY，Z/R 现场保持）"),
    ("LAB_SENSE", "2. 实拍识别插孔并下发坐标（不插取）"),
    ("LAB_PRE_PLACE_MONITOR", "3. 回上一 B 区中心拍照监测"),
    ("LAB_WAIT_MANUAL_PLACE", "4. 等待人工放置当前 B 区（确认后拍照检测）"),
    ("LAB_PLACE_VERIFY", "5. 到当前 B 区中心实拍并判区"),
)


def field_steps_for_round(round_index: int):
    return FIELD_FIRST_STEPS if int(round_index) == 0 else FIELD_REPEAT_STEPS
