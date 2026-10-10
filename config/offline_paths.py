# -*- coding: utf-8 -*-
"""现场离线数据路径（经交换机读工控机落盘文件，不实时连相机/雷达）。

用法：
1. 笔记本 RJ45 接现场交换机，IP 设同网段（1 线建议 10.11.206.171 / 掩码 255.255.0.0）
2. 能打开工控机共享后，把下面路径改成真实路径（可用 \\\\IP\\共享\\... 或映射盘如 Z:\\...）
3. 本软件只读文件，不调用海康/梅卡/Livox SDK

默认指向与本项目同级的 field_offline_data\\（从 jushenzhineng_v3\\examples 拷来的联调样例）。
现场联调时改成工控机上的真实文件即可。
"""
from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any, Dict

PROJECT_ROOT = Path(__file__).resolve().parents[1]
# 与 jushen_field_test 同级：E:\Homework\AA定位\field_offline_data\
_DATA = PROJECT_ROOT.parent / "field_offline_data"

# 规划表备忘（账号/共享名待甲方确认）
# TODO【甲方】梅卡工控机 1 线 IP=10.11.206.190；共享账号密码
# TODO【甲方】雷达电脑 1 线 IP=10.11.206.191；共享账号密码

OFFLINE_PATHS: Dict[str, Any] = {
    # ---------- 雷达（现场 PointNet++）【必需】----------
    # 填目录则取其中最新 .pcd；也可直接填文件
    "radar_pcd": str(_DATA / "radar" / "board.pcd"),
    # 现场例: r"\\10.11.206.191\share\lidar\latest.pcd"

    # ---------- 梅卡托盘相机（离线文件）【可选】----------
    "mech_rgb": str(_DATA / "mech" / "rgb.jpg"),
    "mech_depth": str(_DATA / "mech" / "depth.png"),
    "mech_cloud": "",

    # ---------- 海康角点相机（离线文件，角点 YOLO）【必需】----------
    # 方式 A：一张总图（留空时用下面按点分图）
    "hik_corner_image": "",
    # 方式 B：按角点分文件（有则优先）
    "hik_corner_images_by_point": {
        "P1": str(_DATA / "hik" / "P1.jpg"),
        "P2": str(_DATA / "hik" / "P2.jpg"),
        "P3": str(_DATA / "hik" / "P3.jpg"),
        "P4": str(_DATA / "hik" / "P4.jpg"),
    },
}


def get_offline_paths() -> Dict[str, Any]:
    return deepcopy(OFFLINE_PATHS)
