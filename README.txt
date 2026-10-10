巨神智能 · 现场离线联调（独立项目）
========================================
目录：本仓库根目录 jushen_field_test

一、首次准备（只需一次）
  双击 安装环境.bat
  → 在本目录创建 .venv 并安装 requirements.txt
  不依赖旁边任何项目的 runtime。

二、日常启动
  双击 启动软件.bat
  （只用本目录 .venv 或 runtime，不会去找其它项目）

三、离线数据（与本项目同级）
  ..\field_offline_data\
    radar\board.pcd
    hik\P1.jpg .. P4.jpg
    mech\rgb.jpg / depth.png
  默认已写在 config\offline_paths.py；现场改成工控机路径即可。
  冒烟：.venv\Scripts\python.exe tools\smoke_offline_flow.py

四、数据与算法
  不实时连相机/雷达；读离线 PCD / 图片。
  算法：现场 PointNet + 角点 YOLO。
