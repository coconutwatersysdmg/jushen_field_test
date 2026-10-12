现场离线放货区域动态监测算法。

该模块使用 LAB 颜色分割和轮廓几何提取纸箱四角，不使用 YOLO。
系统集成时只使用它输出的图像 UV 角点；区域合格判定由
services/field_dynamic_box_monitor_service.py 使用 RGB-D 对齐深度、
采集位姿和现场规划区域 WORLD 四角完成判定。
