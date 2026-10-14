# ITL 自动转换

运行 `main.py` 后，两项后台任务各自独立运行：PowerShell 将工控机文件同步到 `存itl`；
Python 只监视本地 `存itl`，将写入完成的 `.itl` 转换为 `PCD/<同名>.pcd`。
任一任务失败都只输出控制台日志，不阻止手动选择本地 PCD。

本地目录每秒检查一次，大小和修改时间稳定两秒后才尝试转换；Windows 下持有只允许读共享的
文件句柄，仍被写入占用的 ITL 会留到下一次重试。先生成临时文件，再原子替换正式 PCD，
选择框不会看到半成品。转换记录保存到 `workdir/itl_conversion_state.json`。
重启后保留记录的文件不会重复转换；源文件变化、输出丢失或标定参数变化后会重新转换。

## 支持的 ITL 格式

实现来自用户提供的 `to_pcd.zip` 中 `BZKJDEMO_pcd.exe` 的文件读取规则和标定数据，
不是通用的任意 ITL 文件阅读器，也不是将文件扩展名改成 PCD：

- 每条记录 2980 字节，CoLa-B `sSN LMDscandata`，`DIST1` 通道含 1441 个距离值。
- 扫描起点为 0°，步长 0.125°；距离单位毫米，比例 1，偏移 0。
- 厂商在记录最后两字节保存有符号转台角度，单位 0.01°，这两字节不作为协议校验和。
- 使用厂商两张补偿表、INI 中 `PARA_M / PARA_L / PARA_G` 和原程序相同的整数截断顺序。
- 默认标定与压缩包的 `IntellyLaserApp.ini` 一致：2°、-0.8°、5830 mm。
- 输出标准 little-endian binary PCD，保留原点顺序和零距离回波，可直接交给项目预览和算法读取。

补偿表保存在 `config/itl_sensor_compensation.json`。修改标定请编辑 `config/itl_conversion.ini`
并重启程序。遇到不同格式、损坏数据或不支持的扫描参数会明确报错并保留原文件，不发布无效 PCD。

基础 CoLa 格式可参考 [SICK 官方 LMDscandata 文档](https://sickag.github.io/sick_lidar_developer_guide/data-streaming/lmdscandata-format/)。
ITL 中的转台角度、补偿表和截断规则以本次提供的厂商程序及配对样本为准。

## 校验

压缩包内 `LaserData-socket1-2025-12-03-12-27-50.itl` 共 3601 条记录、5,189,041 个点。
自动转换结果与压缩包同名手工 PCD 的全部 XYZ 坐标逐点完全一致（最大误差 0 mm）。
实际样本、原 EXE 以及压缩包均无需放入代码仓库或运行环境。
