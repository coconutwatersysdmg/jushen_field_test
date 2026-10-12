现场离线联调算法包（已迁入本项目，禁止引用项目外路径）

目录
----
cam_yolo_lab/   现场离线角点/插孔 YOLO + camera_extrinsic.json + weights/best.pt
fork_hole_lab/  现场离线托盘插孔传统 OpenCV 算法
dynamic_box_monitor/ 现场离线放货区域动态监测算法

运行形态
----------
软件固定为「现场离线联调」：从 config/offline_paths.py 读取现场文件，
设备适配器固定使用 Mock，不连接实时 PLC、雷达或相机。

说明
----
- 现场离线相机路径需要采集元数据中的 plc_pose（拍摄时已自动写入）
