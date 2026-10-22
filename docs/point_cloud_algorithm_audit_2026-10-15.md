> 历史日志校正：此前标名 12:51 的照片坐标实际匹配 11:32 扫描。涉及该照片旧配对的约 4.2 m 差值及边界异常判断已撤回；应以历史日志核对后的坐标对照为准。

# 点云角点算法梳理与精度审查（2026-10-15）

本轮目标：以 P1–P8 的世界坐标为主要评价对象，定位重复运行波动与现场参考偏差，确定精度优先的优化顺序。本轮没有修改生产算法、配置、模型权重或现场设备状态。

结论：当前同时存在可重复性问题和明显的边界/板面估计问题。单独增加 RANSAC 次数、缩小体素或固定一个随机种子，都不足以覆盖已发现的问题。先检查输入点云与现场参考的几何一致性，再将“语义识别”和“几何精测”拆开，并保留原始点供精测使用。

## 1. 样本、坐标与实验边界

- PCD 目录有 5 份文件；本轮对 11-32-55 做分阶段隔离实验，对 10-18-53、12-51-14 各做 3 次完整基线运行。
- 两张现场截图由用户提供，文件名与 PCD 对应。用户确认：截图是原现场系统自动计算值，本身也有误差；两边使用同一雷达到世界坐标标定。因此下文称“现场参考差”，不将其当成独立实测真值的绝对误差。
- 现场截图坐标为人工转录，原图仍是核查依据；表中单位均为 mm。
- 环境为本项目 .venv：Python 3、NumPy 2.4.6、PyTorch 2.14.1+cpu、Open3D 0.20.0，没有使用 GPU。诊断进程设置 PyTorch 4 线程、OMP_NUM_THREADS=4；这些是实验设置，没有写回应用。
- 3 次或 5 次重复是根因定位实验，不能替代工程验收，也不是毫米级精度承诺。

厂家标定关系为先在雷达原点绕 Z 旋转 −0.7°，再执行轴交换和符号变换：`Xw = 13939 + Yr`、`Yw = 7715 − Xr`、`Zw = Zr`。两步合成等效于绕 Z 旋转 −90.7°，然后平移 (13939, 7715, 0)。对 18 个输入组合逐点对照，厂家公式与当前 `radar_to_world()` 最大数值差约 9.1×10⁻¹³ mm。该校验支持当前实现与用户提供的标定规则一致；二维变换不会改变 Z。

用户进一步确认这些 PCD 来自 ITL 转换。本项目 `utils/itl_to_pcd.py` 并非简单复制已有 XYZ：它从距离、扫描角和编码器值重建坐标，在本地实现中将 `PARA_M` 作为倾角、`PARA_L` 作为横滚角、`PARA_G` 作为安装高度使用，并读取两张补偿表。上述字段解释来自当前代码，不能仅凭本地实现独立证明厂家的原始定义。这一环节与后续 WORLD 标定是不同的几何步骤；整个 INI 还包含 ROI、阈值和设备参数，不能把整个文件笼统称为补偿参数。

当前转换使用 M=2.0°、L=−0.8°、G=5830 mm。厂家包还有一份 `IntellyLaserApp1.ini`，其中 M=−1.0°、L=0.4°、G=5850 mm。两份文件同时存在不是错误证据，尚需核实现场实际使用哪套参数。没有据此修改生产参数。

已有项目说明 `docs/itl_conversion.md` 记载转换实现来自厂家程序及其标定数据，并曾与压缩包内 2025-12-03 的同名手工 PCD 做过全量逐点一致对照。这是已有文档记录，本轮没有重新取得该原始配对样本验证；现有单元测试也保存了少量厂家坐标检查点。因此，应核对当前现场参数，而不是因结果不同就直接认定转换公式写错。用户目前不知道现场实际使用的参数。

使用当前参数将 12-51-14 的同名 ITL 重新转换到独立输出目录，得到的 PCD 与现有文件 SHA-256 完全一致（4bbcfd07d0f5ee0f223e1f3932fd96a5f747b91009fbe4a59859e3bbebe41d58）。这确认该样本可由当前转换规则精确复现；它不能证明参数与现场一致或点云几何正确。

## 2. 当前实际处理流程

```text
PCD 原始点
  → ROI 裁剪（每轴跨度外扩 2.5%）
  → 20 mm 体素降采样
  → 最大平面 RANSAC（140 mm 内点阈值），删除其内点
  → DBSCAN（eps=100 mm，min_points=50），只保留最大簇
  → 统计离群点过滤（6 邻居，3 倍标准差）
  → PointNet++ 整云推理，逐点 argmax 得到标签
  → label 2 / label 3 分别独立处理
  → 板面 RANSAC（100 mm 内点阈值）
  → PCA 建立 U/V/N 局部坐标
  → 沿 U/V 切片，每片选一个最外侧点
  → 四条直线独立 RANSAC（30 mm 阈值）与 SVD 重拟合
  → 直线求交得到四角 XY；恢复到同一拟合平面得到 XYZ
  → 雷达转世界坐标 → 尺寸、姿态和装载规划
```

注意：这里“整云”是预处理后的全部点，不是原始 PCD 全部点。当前主流程没有再随机抽成 49152 点；`pointnet_num_point=49152` 主要保留作初始化兼容，调大它不会直接增加本主流程的输入点数。网络内部仍固定抽取 512、128 个中心建立局部特征，因此整云输入也不等于取消了网络内部采样。

11-32-55 第一次预处理的点数为：5,189,041 → ROI 1,329,941 → 体素 690,409 → 去主平面 291,334 → 最大簇 216,966 → 离群点过滤 214,214。最终角点直接由最后这批点及其语义标签计算，没有回到原始 ROI 中精测。

## 3. 已经通过实验确认的问题

### 3.1 同一输入下，网络分类随连续调用变化

对固定的 214,214 个预处理点，复用同一模型连续推理 3 次：第二、三次相对第一次分别有 3,035、3,318 个点的类别变化。结合后续几何步骤，P3、P4 的最大两两位移分别约 167.25、167.03 mm。

代码中的最远点采样使用 `torch.randint` 选择起点。推理封装只在模型初始化时设置 Torch 种子，后续每次前向计算继续消耗随机状态。`.eval()` 已经调用，不能将本问题简单归因于 Dropout 没有关闭。

控制实验：固定同一批点，每次推理前重新设置 Torch 种子，连续两次的分类标签完全相同（变化数为 0）。这直接验证了该环境下随机状态对分类结果的影响，但尚未验证跨设备、跨版本的逐位一致性。

### 3.2 预处理已经会产生不同的点集合

固定同一个体素点云，仅重复主平面 RANSAC 4 次，内点数分别为 394821、395476、392989、393310，内点索引哈希全部不同。

完整预处理重复 3 次，最终点数为 214214、214325、214248。把点按 XYZ 排序后，坐标哈希仍不同，因此差异不仅是点的排列顺序。

代码没有控制 Open3D 随机状态。补设种子也要用实验验收：本轮固定点与标签，逐次重设 Open3D 种子后，几何角点仍出现最高约 0.35 mm 的两两位移。另一个进程仅设置 OMP_NUM_THREADS=1 后仍有差异，不能将该设置称为已经验证的“单线程确定性方案”。Open3D 0.20.0 的平面分割源码使用 TBB 并行计算并带自适应停止；OMP 环境变量不足以证明它被限制为单线程。

### 3.3 固定分类后，几何波动明显减小，但未完全消失

固定输入点与第一次分类结果，仅重复几何后处理 5 次：label 2 的四角在输出精度内相同；label 3 的最大角点位移约 1.37 mm。与固定点、变化分类时的约 167 mm 相比，这个样本主要的大幅波动与分类变化及其边界放大效应有关。

这不代表其他文件的几何步骤都只有 1.37 mm 波动，也不能将其解释为对真实坐标的准确度。

## 4. 与现场参考的逐点比较

下面“运行波动”为 3 次完整运行中同一点的最大两两三维距离；“现场参考差”为每次结果与截图对应点的三维距离范围。点号按当前接口直接对应，没有重新排列或做事后配准。

### LaserData-socket1-2026-10-15-10-18-53.pcd

| 点 | 最大运行波动 | 现场参考差范围 | 首次结果减参考值 ΔX / ΔY / ΔZ |
|---|---:|---:|---|
| P1 | 0.6 | 171.5–171.7 | +99.1 / -66.4 / -123.2 |
| P2 | 6.9 | 173.3–178.2 | +129.5 / -30.5 / -115.2 |
| P3 | 5.9 | 493.4–498.7 | +446.1 / -193.4 / -111.0 |
| P4 | 5.4 | 546.0–550.4 | +461.4 / -230.4 / -179.2 |
| P5 | 12.4 | 230.0–231.1 | -14.0 / -212.9 / -85.7 |
| P6 | 98.9 | 275.1–306.3 | -39.0 / -247.8 / -113.0 |
| P7 | 1.2 | 256.3–257.4 | -73.7 / -160.6 / +185.8 |
| P8 | 2.1 | 272.8–273.5 | -148.0 / -158.6 / +166.5 |

### LaserData-socket1-2026-10-15-12-51-14.pcd

| 点 | 最大运行波动 | 现场参考差范围 | 首次结果减参考值 ΔX / ΔY / ΔZ |
|---|---:|---:|---|
| P1 | 2.5 | 297.2–298.7 | -179.4 / -147.0 / -188.3 |
| P2 | 2.4 | 249.6–251.0 | -102.8 / -122.4 / -192.2 |
| P3 | 1.2 | 378.8–379.8 | -170.8 / -274.5 / -197.4 |
| P4 | 0.6 | 426.9–427.5 | -168.9 / -306.7 / -245.1 |
| P5 | 87.9 | 372.0–406.3 | -165.1 / -332.8 / -18.8 |
| P6 | 115.6 | 453.0–507.8 | -274.7 / -422.8 / -33.1 |
| P7 | 9.5 | 4220.8–4221.6 | -4197.7 / -241.3 / +370.0 |
| P8 | 2.4 | 4290.2–4291.5 | -4269.5 / -217.8 / +361.1 |

12-51-14 的首次基线中，P7 算到 (4481.262, 4984.657, 1445.039)，现场参考是 (8679, 5226, 1075)；P8 的差异也超过 4 m。两点重复运行波动仅约 9.5、2.4 mm，因此即使消除随机性，这种边界范围差异仍然存在。

另一次该文件的诊断推理中，label 3 的板面内点包含 35,250 个 WORLD X 小于 8674 mm 的点；最小 X 约 4373 mm。也就是说，超出参考尾端约 4 m 的区域已经被纳入分割后的板面支持点，不只是最后直线交点的微小数值偏移。仍需在点云中确认这些点的物理归属：车板、附近物体或输入点云自身几何差异，不能仅凭截图确定所有这些点都是错分点。

10-18-53 还存在两块板接缝不一致：现场参考 P3/P5、P4/P6 的 XY 分别相同，而当前两块板独立拟合所得的对应边在 XY 上有数百毫米差异。下一步应检查分割是否截断高板边缘，以及两板共用接缝的物理约束是否适用。

![12-51-14 顶视及侧视比较](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_audit_20261015/field_12_51_comparison.png)

图中红线是当前基线首次输出，绿虚线是现场系统参考；点颜色来自同一 PCD 的另一次诊断推理。因此它用于检查范围和板面形态，不能将图中每一个彩色点解释为红线那次运行的精确内点。

侧视图还显示：当前 label 3 的点本身形成从约 1040 mm 升至 1440 mm 的斜面，而现场参考低板四角约为 1065–1077 mm。拟合结果确实跟随了当前点集的形态，不能把这部分差异全部解释成求交数值误差。本轮也查询了完整原始点云到参考角点的最近三维距离，12-51-14 为约 150–331 mm。角点本就可能没有直接回波，所以最近点距离不是角点测量误差，也不能单独证明 PCD 转换有错；这些证据说明应检查扫描、PCD 导出/ITL 转换、采集补偿和参考样本的一致性。

## 5. 精度风险与建议顺序

| 优先级 | 当前机制与证据 | 建议验证的改进 | 验证指标 |
|---|---|---|---|
| P0 | 当前点集的范围、坡度与现场参考存在大差异，拟合在跟随点集形态 | 核对 ITL 转换的 M/L/G 与补偿表是否与现场一致，做同源厂家导出与本地导出对照 | 原始点对参考边界的支持、两种导出点坐标差 |
| P0 | 网络 FPS 随机起点；固定点重复推理产生不同标签 | 将推理随机状态按文件/任务隔离，或设计确定性采样；同时固定输入顺序与候选相同评分时的选择规则 | 同文件、同实例连续运行及重启后的逐点标签、角点重复性 |
| P0 | label 3 支持点超出参考尾端约 4 m | 可视化语义点、板面内点和四条边，检查物理归属；对候选板面增加朝向、高度、连通性和边界支持检查 | P7/P8 是否落在正确物理边界，误识别率 |
| P0 | 语义硬标签直接决定最终边界，标签变化可放大到 167 mm | 语义网络先给粗区域和置信度；用原始 ROI 中的几何证据精测边界，给边界留回搜裕量 | 分类扰动后的角点位移、现场参考差 |
| P1 | 20 mm 体素和统计过滤后的点直接用于最终拟合 | 保留原始点及索引映射，粗采样用于找目标，原始或更细点用于边缘局部精测 | 体素 20/10/5 mm 及原始点局部精测的对照，边界保留率 |
| P1 | 每片只取一个最外点，四条边独立拟合 | 保留边界邻域的多点支持，用密度、连续性和稳健拟合筛选；以证据选择候选边 | 边覆盖长度、残差分布、角点对点删除和噪声的敏感度 |
| P1 | 四个 XYZ 角点被强制恢复到同一全局平面 | 明确角点定义；若要各角附近的实际板面高度，比较局部平面/局部稳健估计与全局平面 | 各角 Z 的偏差、局部支撑点和外推距离 |
| P1 | 默认最大平面即地面、最大点簇即目标，没有检查目标身份 | 根据现场高度、法向、位置和连通结构验证候选；避免无条件删除主平面、删除小簇 | 地面误删率、板边/两作业面保留情况 |
| P1 | 两块板独立求角点，没有接缝与形状一致性校验 | 在实物支持时加入共享接缝 XY、近似平行/垂直等软约束；保留形变空间 | 接缝间隙、边方向差、约束前后对真实点的拟合代价 |
| P2 | 输出有坐标和耗时，但没有完整质量证据 | 保存输入/权重哈希、完整参数、随机状态、板面与边支持点、覆盖率和退化状态 | 能解释每次角点来源，能识别无足够证据的结果 |

20 mm 体素不是“必然有 20 mm 误差”，平面或直线拟合有可能得到更小的统计误差；但体素平均、边界缺点和混合表面会引入偏差，增加迭代无法恢复已删去的原始边缘。

140 mm、100 mm、30 mm 分别是内点筛选阈值，不是输出精度，也不是误差上界。应根据板面粗糙度、测量噪声与目标混杂情况做逐项对照，不能全部机械地改成几毫米。

截图高板四角到最佳拟合平面的最大绝对残差分别约 19.18 mm、11.09 mm。这说明现场参考输出本身并非严格共面；可能来自车板形变、局部测量、算法定义或参考误差，不能据此断言实物一定翘曲。当前全局平面角点模型与这种输出定义存在差异，应先确定要测量的物理量。

刚性标定旋转和平移不会改变长度；单纯调 WORLD 平移或旋转也无法把 13 m 的板面范围变成 9 m。不要为了贴合这两张截图去反向调整已有 WORLD 标定常数。ITL 转换的参数一致性应单独核查；本轮尚未证实转换公式或其参数存在错误。

## 6. 建议的下一版算法结构

```text
原始 PCD → 保留完整 ROI 点与来源索引
                  ├─ 粗处理 + 可重复语义推理 → 候选作业面 / 置信度
                  └─ 回到原始 ROI，带裕量搜索板面和边界
                         → 几何、连通性与高度联合确认目标范围
                         → 四边多点稳健拟合与条件检查
                         → XY 角点 + 与角点定义一致的局部/全局 Z
                         → 世界坐标 + 每角质量证据
```

这里回到原始点不等于给所有原始点套最近邻标签后直接拟合。边界必须保留独立的几何判据，否则只是把粗分割的错误复制到更多点上。网络半径、局部点数、归一化或邻域选择的改变还可能偏离训练分布，应单独验证，必要时重新训练。

先实现一个可复查的基线与目标范围检查，再验证原始点精测，最后评估局部 Z 和结构约束；每阶段只改一类因素。全量跑 5 个文件，特别检查两块板接缝与 P7/P8 尾端，避免只对一个样本调参。

## 7. 验收建议与尚未完成的工作

1. 同文件至少重复 20 次，分别测同一进程、重启进程及改变文件处理顺序；每个点报告 XYZ 极差和三维最大两两距离。
2. 固定文件后，再改变噪声、局部缺点和采样密度，检查鲁棒性，不能把固定种子当鲁棒性证明。
3. 保留当前现场系统作为相对比较基准，另取得具有测量不确定度说明的独立控制点或实测角点，再评估绝对准确度。
4. 先由工程需求确定允许的各角 XY、Z 误差和失败处置；目前用户只明确了“角点坐标最重要”，尚未给出量化容差。

本轮完成的是梳理、现象复现、隔离实验和优化路线。生产算法修复、完整五文件验收、独立真值精度评估尚未实施。

## 8. 代码与证据入口

- [algorithm_modules/point_cloud_segment_module/point_cloud_pipeline.py](C:/Users/18252/Desktop/jushen_field_test/algorithm_modules/point_cloud_segment_module/point_cloud_pipeline.py:204)
- [algorithm_modules/point_cloud_segment_module/point_cloud_pipeline.py](C:/Users/18252/Desktop/jushen_field_test/algorithm_modules/point_cloud_segment_module/point_cloud_pipeline.py:497)
- [algorithm_modules/point_cloud_segment_module/point_cloud_pipeline.py](C:/Users/18252/Desktop/jushen_field_test/algorithm_modules/point_cloud_segment_module/point_cloud_pipeline.py:686)
- [algorithm_modules/point_cloud_segment_module/point_cloud_pipeline.py](C:/Users/18252/Desktop/jushen_field_test/algorithm_modules/point_cloud_segment_module/point_cloud_pipeline.py:831)
- [algorithm_modules/point_cloud_segment_module/models/pointnet2_utils.py](C:/Users/18252/Desktop/jushen_field_test/algorithm_modules/point_cloud_segment_module/models/pointnet2_utils.py:63)
- [algorithm_modules/point_cloud_segment_module/inference.py](C:/Users/18252/Desktop/jushen_field_test/algorithm_modules/point_cloud_segment_module/inference.py:217)
- [services/point_cloud_processing_service.py](C:/Users/18252/Desktop/jushen_field_test/services/point_cloud_processing_service.py:81)

- [outputs/point_cloud_audit_20261015/evidence.json](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_audit_20261015/evidence.json)
- [outputs/point_cloud_audit_20261015/field_references.json](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_audit_20261015/field_references.json)
- [outputs/point_cloud_audit_20261015/field_comparison.json](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_audit_20261015/field_comparison.json)
- [outputs/point_cloud_audit_20261015/corner_comparison.csv](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_audit_20261015/corner_comparison.csv)
- [outputs/point_cloud_audit_20261015/vendor_transform_check.json](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_audit_20261015/vendor_transform_check.json)
- [outputs/point_cloud_audit_20261015/itl_reconversion_check.json](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_audit_20261015/itl_reconversion_check.json)
- [outputs/point_cloud_audit_20261015/network_reseed_control.json](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_audit_20261015/network_reseed_control.json)
- [outputs/point_cloud_audit_20261015/single_thread_control.json](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_audit_20261015/single_thread_control.json)
- [outputs/point_cloud_audit_20261015/raw_reference_support.json](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_audit_20261015/raw_reference_support.json)
- [outputs/point_cloud_audit_20261015/sample_12_51_profiles.json](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_audit_20261015/sample_12_51_profiles.json)
- [outputs/point_cloud_audit_20261015/run_audit.py](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_audit_20261015/run_audit.py)
- [outputs/point_cloud_audit_20261015/compare_field_references.py](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_audit_20261015/compare_field_references.py)

Open3D 并行实现参考：[Open3D v0.20.0 平面分割源码](https://github.com/isl-org/Open3D/blob/v0.20.0/cpp/open3d/geometry/PointCloudSegmentation.cpp)。其中 TBB 并行和自适应停止机制是进一步排查可重复性的依据；具体本地结果以实验 JSON 为准。
