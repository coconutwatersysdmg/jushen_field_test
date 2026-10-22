# point_cloud_segment_module

当前版本：`geometry_revision=head_near_anchors_v3`（2026-10-19）。本次只修改底板边界、区域归属和角点高度；模型权重、全覆盖推理、预处理及世界标定保持原配置。

当前仅支持 `geometry_scheme=2`。已移除的方案一仍只保留历史文件，不恢复入口或自动回退。

临时标签纠正：对整份分割结果统计标签3点数。若 `0 < N3 < 30000`，全部标签3点并入标签2，再进行板型、边界与高度计算；`N3 >= 30000` 时保留标签3，恰好30000点不合并。标签0/1与所有XYZ保持原样，原模型预测数组不修改。并入点同样参与标签2的H拟合。算法JSON和服务结果中的 `label_correction` 记录阈值、是否应用、并入数量及纠正前后点数。此规则针对整份推理后的保留点集，不按PointNet分组分别统计。

车头横线从**模型分割出的单板/高板底面点云**投影后拟合，不使用挡板标签1或支撑物点。左侧（P1/P3/P5/P7）为近雷达侧，独立拟合后作为长度方向基准。

- 右侧长边先自由拟合，固定其与原车头横线的交点P2，再旋转到与左侧长边平行。低板直接使用高板左右长边的延长线。宽度只由底板确定。
- 其他横线先自由拟合，固定其与左侧长边的原交点，再旋转到与车头横线平行；之后与修正后的右侧长边重新求交。整车车尾为单板尾线或低板尾线，低板车尾固定P7，重新求P8。
- 车头横线和左侧长边之间保留实测夹角，不额外强制90°。正交UV仅是投影坐标，`geometry_fit.lines`中的横线允许斜率，不能再假设所有横边为u常量。
- 无支撑物：修正后的原高板尾线为共同分界。有支撑物：修正后的低板前线为共同分界，高板区域延伸到这里。P3/P5、P4/P6分别共用XY；高度各自独立。
- 支撑物仅参与有无判断及区域归属，不加入底板点集、宽度拟合或高度拟合；超过左右底板边线的支撑物点不扩大底板。原XYZ及语义标签不改。

支撑物判据采用连接处可靠法向自身的主方向：在该轴正/反方向20°内统计集中度C，严格大于80%才判无支撑物，等于或低于80%判有支撑物。不再要求主方向贴合车长轴，高低板高差不作为类型判定的门槛。主方向通过观测法向和散布矩阵轴搜索并迭代细化，属于确定性的方向众数估计。至少50个可靠法向、有效比例至少20%的要求保留；证据不足暂按有支撑物返回，并单独记录原因。取点仍使用合并前高低板参考和连接处内部的结构标签候选。单一水平方向也可能集中，这是当前仅按集中度判断的规则。`transition_support.merge_applied`记录区域归属是否已应用；`support_points_added_to_floor=false`。分类器建议的支撑物后沿不作为最终分界依据，分界由低板起点确定。

人工标注后的支撑物试验可通过 `PipelineConfig(support_decision_mode='structural_evidence')` 启用。它检查连接立面横向去倾斜后的纵向残差P90（不超过40mm）、去除高板平面30mm薄带后的辅助法向集中度（40度范围至少90%、至少50个可靠法向），并要求竖直面覆盖底板宽度至少50%。另用完整连接点的法向与附近底板法向比较，记录顶面和竖直板方向的证据；这个方向统计保留与高板同高的点。高板边缘薄带本身不足以证明支撑物，需在尾部参考线后观测到连续且有足够面积的顶面。两个空间分离的平行立面也作为有支撑物证据。几何诊断、支撑物顶面和原点云始终保留。同高支撑物只观测到一个立面时仍可能缺少证据，正式默认继续使用 `normal_concentration`。本轮评估仅使用人工确认的两类高低板；单板不参与支撑物统计，单板处理规则不变。离线结构分类试验及用户确认的“同高支撑物已并入高板，按无支撑物流程处理”策略单独记录在 LIDAR_DATA 结果目录，尚未替换默认软件分类器。

底板高度默认A方案：选区沿长度分五条，各条稳健拟合局部平面，取条带中心Z后等权平均。B/C/D方案和参数保留。

| 板型 | 本板长度选区 | 近侧起宽度选区 |
|---|---:|---:|
| 单板 | 前60% | 95% |
| 高板 | **最终确定的高板区域**的前80% | 95% |
| 低板 | 从最终共同分界起的前40% | 95% |

先完成横线修正及最终区域确定，再生成局部裁剪框。`height_source_radar_corners_xyz_mm`保存最终区域的全精度裁剪基准四边形，`height_estimation.height_crop_basis=final_this_deck_geometry`。有支撑物时，原高板尾线未通过检查不会单独阻止H计算，只要低板起点已能确定最终高板结束线即可。尾部未选入裁剪框的区域不要求有完整扫描点。高度拟合仍只用裁剪框内本板同标签底面点（包括按临时规则并入的点），支撑物标签1不参与；所有五条支撑通过才输出A方案H。

最终四个角点Z均使用本板H，不再调用角点邻域局部Z估计；`corner_height_estimation.status=deck_reference_height`，`corner_height_mean_mm`也是实际角点均值H，`geometry_plane_height_mean_mm`仅为原参考平面诊断值。同板倾角为0，长宽由最终等高角点重算。

最终坐标、长宽、高度、角度及装载长度统一四舍五入为整数，JSON和界面采用相同规则（2.5→3，−2.5→−3）。先用完整精度完成拟合、求交、坐标变换和尺寸计算，最后取整；界面沿用最终尺寸，避免用已取整角点再次计算造成偏差。拟合系数、原始平面角点、选区与条带中心等内部诊断数据及比例、耗时保留精度。

必要参考边/低板几何失败时返回`partial/unsupported`，不自动改判单板。H不足时保留`available_xy_corners_radar_mm`及`global_plane_radar_corners_xyz_mm`作诊断，正式`corners_xyz_mm/radar_corners_xyz_mm`为空；服务返回失败状态，不把参考平面Z作为有效坐标，也不生成装载规划。界面仍按高板/低板原顺序显示已有尺寸和缺失H。

历史版本快照位于`outputs/scheme1_backup_20261016_124835`、`outputs/scheme2_before_nearside_20261016_145409`；本次修改前快照位于`outputs/head_reference_geometry_20261019_154004/before`。旧结果保留。

## 1. 文件夹结构

推荐目录结构如下：

```text
Algorithm_modules/
└─ point_cloud_segment_module/
   ├─ __init__.py
   ├─ inference.py
   ├─ point_cloud_pipeline.py      # 主流程；主要参数集中在文件前部 PipelineConfig
   ├─ README.md
   ├─ LICENSE
   ├─ checkpoints/
   │  └─ best_model.pth            # PointNet++ 权重
   ├─ models/
   │  ├─ __init__.py
   │  ├─ pointnet2_part_seg_msg.py
   │  └─ pointnet2_utils.py
   ├─ data/                         # 本地放原始 PCD；数据较大可不随代码分发
   │  ├─ 1-1.pcd
   │  ├─ 1-2.pcd
   │  └─ ...
   └─ output/
      └─ json/                      # 程序运行后自动生成
```

> `data/` 中的 PCD 原始数据体积较大，本压缩包不包含实际 PCD。使用时把测试/现场 PCD 放入本地 `data/` 文件夹即可。

## 2. 当前正式处理流程

原始PCD沿用现有ROI、体素、去地面、聚类和离群点预处理；PointNet++每组固定49152点，直接覆盖全部入口点，不做KNN标签恢复。

几何阶段使用分割后的底板点云：底板平面内点的水平XY投影 → 独立拟合车头横线/近侧长边/原远侧长边 → 固定原右前交点、远侧长边转到与近侧平行 → 各横边固定原近侧交点、转到与车头横线平行 → 求交得到XY。高低板分界按支撑物分类采用原高板尾边或低板前边；两板共用左右侧线及最终分界线。

然后按本板原始同标签点估计H，四个角点Z全部设为该板H，再转WORLD并重算尺寸、姿态和装载规划。WORLD仍为绕Z旋转-90.7°、XY平移13939/7715 mm，Z不变。高度失败时只保留诊断XY，不输出平面或局部拟合Z作为正式角点，不生成装载规划。

## 3. 运行环境

当前项目已在 Windows Conda `project` 环境中使用。核心依赖：

```text
Python
numpy
torch
open3d
```

另外，模型代码依赖本模块 `models/` 下的 PointNet++ 实现。

建议直接复用项目现有 `project` Conda 环境，不要随意升级 PyTorch/Open3D 后再判断模块问题。

## 4. 命令行运行

在项目根目录执行（使用便携 runtime）：

```bat
runtime\python.exe -m algorithm_modules.point_cloud_segment_module.point_cloud_pipeline ^
"algorithm_modules\point_cloud_segment_module\data" ^
--checkpoint "algorithm_modules\point_cloud_segment_module\checkpoints\best_model.pth" ^
--device cuda ^
--num-point 49152 ^
-o "algorithm_modules\point_cloud_segment_module\output"
```

也可以处理单个 PCD：

```bat
runtime\python.exe -m algorithm_modules.point_cloud_segment_module.point_cloud_pipeline ^
"algorithm_modules\point_cloud_segment_module\data\1-1.pcd" ^
--checkpoint "algorithm_modules\point_cloud_segment_module\checkpoints\best_model.pth" ^
--device cuda ^
--num-point 49152 ^
-o "algorithm_modules\point_cloud_segment_module\output"
```

## 5. 参数怎么改

所有现场常用参数都集中在 `point_cloud_pipeline.py` 最前面的：

```python
@dataclass(frozen=True)
class PipelineConfig:
```

主要参数分组如下。

### ROI

```python
roi_min = (300.0, -11000.0, -200.0)
roi_max = (9000.0, 14000.0, 3500.0)
roi_redundancy = 0.025
```

ROI 和分割、边缘拟合均使用雷达坐标（mm），保留每轴跨度 2.5% 的裁剪裕量。提取角点后由 `radar_to_world()` 转到世界坐标；Z 不变，点的编号和顺序不变。尺寸和姿态使用转换后的角点计算。

### 预处理

```python
voxel_size = 10.0
plane_threshold = 140.0
plane_ransac_n = 3
plane_iterations = 500

dbscan_eps = 100.0
dbscan_min_points = 50

stat_neighbors = 6
stat_std_ratio = 3.0
```

### 底板平面与边缘

```python
edge_plane_distance = 100.0
edge_plane_ransac_n = 5
edge_plane_iterations = 800
edge_min_inliers = 3000

slice_width_u = 50.0
slice_width_v = 30.0
slice_min_points = 20

line_ransac_distance = 30.0
line_ransac_iterations = 300
line_min_inliers = 25
```

### 车辆方向

```python
truck_head_dir = (-1.0, 1.0, 0.0)
```

局部坐标定义：

- **U**：车长方向，`Umin` 为车头端，`Umax` 为车尾端。
- **V**：车宽方向。
- **N**：底板拟合平面法向。

### 第一作业面规划

```python
pallet_length_mm = 1200.0
pad_trigger_ratio = 0.50
```

当前规则：

- 只有同时成功识别 `label_2` 和 `label_3` 两块底板时才启用。
- 只对 `label_2`（第一作业面）计算。
- 托盘沿车长 U 方向占用 1200 mm。
- 最后剩余空间占一个托盘长度 **≥ 50%** 时，当前第一版逻辑直接判定使用垫板。
- 补偿距离：

```text
remaining_workface_compensation_mm
= pallet_length_mm - remaining_length_mm
```

例如剩余 800 mm，则补偿距离为 400 mm。

## 6. 角点与姿态定义

若检测到一块底板：

```text
P1 = Umin,Vmin
P2 = Umin,Vmax
P3 = Umax,Vmin
P4 = Umax,Vmax
```

若检测到两块底板：

```text
label_2 : P1 ~ P4
label_3 : P5 ~ P8
```

JSON 的 `corners_xyz_mm` 为世界坐标，标记 `coordinate_frame="world"`；一块板输出 P1–P4，两块板输出 P1–P8。`radar_corners_xyz_mm` 保留转换前的雷达坐标供核对，界面和下游不再对世界角点重复转换。

姿态参数：

- `tilt_angle_deg`：车头 → 车尾沿长度方向相对于水平面的纵向倾斜角，单位 **°**。
- `offset_angle_deg`：俯视情况下，车长 U 正方向相对于理论 `world -Y` 的水平偏移角，单位 **°**。

两个字段都已经是角度，可以直接读取显示，无需二次换算。

## 7. JSON 输出示例

```json
{
  "label_2": {
    "corner_ids": ["P1", "P2", "P3", "P4"],
    "corners_xyz_mm": [[...], [...], [...], [...]],
    "length_mm": 4380.54,
    "width_mm": 2985.288,
    "height_mean_mm": 1414.566,
    "tilt_angle_deg": 0.393143,
    "offset_angle_deg": 2.283609,
    "loading_plan": {
      "pallet_length_mm": 1200.0,
      "normal_pallet_count": 3,
      "remaining_length_mm": 780.54,
      "remaining_ratio": 0.65045,
      "pad_trigger_ratio": 0.5,
      "pad_required": true,
      "remaining_workface_compensation_mm": 419.46,
      "final_pallet_count": 4
    }
  },
  "label_3": {
    "corner_ids": ["P5", "P6", "P7", "P8"],
    "corners_xyz_mm": [[...], [...], [...], [...]],
    "length_mm": 13000.439,
    "width_mm": 2972.443,
    "height_mean_mm": 1209.339,
    "tilt_angle_deg": 1.637939,
    "offset_angle_deg": 0.788217
  }
}
```

最终文件位置：

```text
output/json/<原PCD文件名>.json
```

## 8. 集成到上位机时

当前版本为了独立测试，会把结果写入 JSON。

真正集成到系统后，不需要依赖 `output` 文件夹。`PointCloudPipeline.process_file()` 内部已经拿到了 Python 字典 `result`，可将写 JSON 的步骤替换为直接 `return result`，交给 UI / PLC / 任务规划模块读取。

JSON 主要用于当前离线测试、核对和留档。

## 9. 注意事项

1. `best_model.pth` 必须和当前 PointNet++ 网络结构、标签定义一致。
2. 当前每次固定49152点，原始点数增加主要增加覆盖组数。若CUDA报OOM，不输出不完整标签；应检查该固定组大小与设备显存，不会用KNN补标签。
3. `label_2` / `label_3` 的语义定义由当前训练数据保持一致，不要随意调换。
4. `data/` 只存测试/现场原始 PCD，不属于代码依赖，可以不随代码仓库提交。
5. 修改现场参数时，优先只改 `PipelineConfig`，不要直接在算法函数内部改硬编码。

## 10. 重复性控制（2026-10-15）

当前 PCD 后处理增加了以下控制，不修改 ITL 转换、世界标定、模型权重或几何距离阈值：

- 主平面去除与板面拟合使用局部 NumPy 随机生成器和固定候选数量；候选按支持点数、均方残差依次评分，完全同分时保留较早候选。采样和评估顺序固定，避免平面候选并行竞争导致结果变化。
- 候选与最终平面使用正交最小二乘拟合，重合点和共线点不会生成有效平面。平面内点索引恢复到原输入行号。
- 网络输入先按坐标建立固定顺序，再按固定种子打乱。这样既消除原点序变化的影响，也避免网络邻域按行号取点时偏向坐标排序的一端。推理标签恢复到调用方原始点序。
- 每次网络推理使用独立的 Torch 随机状态上下文，结束时恢复调用方状态；多次投票在本次上下文内按固定随机序列运行。

这些改动用于控制相同输入、相同参数和相同运行环境下的重复性。它们不能证明分类正确、所选边界正确或角点达到某个绝对精度；现场算法坐标没有参与候选评分或参数选择。GPU 和跨依赖版本的一致性需单独验证。

离线验证（使用项目 `.venv`，不连接设备）：

```powershell
.venv/Scripts/python.exe -m unittest tests.test_point_cloud_repeatability -v
.venv/Scripts/python.exe tools/check_point_cloud_repeatability.py --input PCD --output outputs/repeatability_check
```

验证脚本默认对目录内文件交替顺序运行 3 轮，对 `11-32-55` 样本（存在时）补足到 20 次，并用新进程复核该样本。输出保存中间点集/标签哈希、未取整雷达角点位移、支持点与拟合残差；这是重复性和点云内部证据，不是与外部真值的精度验收。
