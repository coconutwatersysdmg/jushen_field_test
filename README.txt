巨神智能 · 现场离线联调（独立项目）
========================================
目录：本仓库根目录 jushen_field_test

当前现场雷达测试
----------------
运行 main.py 或双击「启动软件.bat」，控制台会持续输出 [RADAR_SYNC] 监视日志。
雷达工控机 wave 文件夹的文件同步到本项目「存itl」文件夹，每 10 秒扫描一次。
同步不限制文件名或类型，不按启动时间过滤；本地缺少或大小/修改时间有变化的文件会拷贝。
PCD 拷贝完成后才以正式文件名出现；拷贝失败会输出原因，下次扫描重试。
点击「选择 PCD」，默认打开「存itl」；选好后点击「运行雷达算法」。
取消选择就停在原地，后台同步继续；当前不运行相机、插孔、放货流程。
所有监视输出同时保存在 logs/startup.log；关闭程序会停止本次同步进程。
监听启动失败、共享不可达或监听进程退出，只输出日志，不弹窗、不阻止本地 PCD 选择和算法运行。
同步脚本只尝试访问当前网络上的共享，不切换网卡、不修改 IP 或路由。
恢复完整现场流程：将 config/feature_switches.py 的 RADAR_ONLY_TEST 改为 False。

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
