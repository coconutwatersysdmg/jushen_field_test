现场离线联调算法包

目录
----
camera_transform.py  RGB-D 相机坐标到 WORLD 的标定变换
fork_hole/            托盘插孔传统 OpenCV 算法
dynamic_box_monitor/  放货区域动态监测算法

现场角点 YOLO
------------
角点识别服务位于 services/corner_recognition_service.py，
权重固定为 models/corner_service.pt。该模型属于现场流程，已保留。

运行形态
----------
软件固定为「现场离线联调」：从 config/offline_paths.py 读取现场文件，
设备适配器固定使用 Mock，不连接实时 PLC、雷达或相机。

说明
----
- 相机坐标变换需要采集元数据中的 plc_pose。
