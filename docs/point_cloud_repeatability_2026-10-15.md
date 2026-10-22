# 点云算法第一轮修复：可重复性（2026-10-15）

本报告为 20 mm 体素阶段的验证记录。随后用户确认将生产默认体素改为 10 mm，已执行并重新运行 12 份点云生成坐标对照；当前批次每文件一次，不等于已对全部新增点云重复验收。下列原始验证次数及结果仍保留其当时参数含义。

本轮只处理 PCD 之后的算法可重复性。ITL 转换和厂家安装参数继续搁置。用户明确要求：不能为了接近现场算法的数值而调参。实现和评分没有读取现场截图坐标，也没有添加坐标补偿。

本轮处理了三类已复现问题：网络内部随机采样状态在连续调用间变化；网络邻域对点顺序敏感；平面 RANSAC 的随机采样、并行停止及同分候选竞争导致结果变化。

## 实际修改

- 网络输入建立固定顺序后用固定种子打乱，保留邻域采样的空间覆盖，推理后将标签恢复到原始行号。
- 每次网络推理隔离并重设 Torch 随机状态，结束后恢复调用方状态；同一次多票推理保留固定的随机序列。
- 主平面与板面 RANSAC 使用局部随机生成器和固定候选预算，按支持点数、均方残差依次评分，完全同分时保留较早候选。候选与最终内点平面使用正交最小二乘拟合，拒绝共线/重合采样。
- 保留原有 ROI、20 mm 体素、140/100/30 mm 几何距离阈值、网络权重、ITL 转换配置和 WORLD 标定。

曾尝试固定 Open3D 种子并关闭提前停止，但在两层完全同分的合成平面上仍复现并行竞争。该方案未通过验收，最终改为确定性的候选评估；其早期输出仅为诊断记录，不作为最终结果。

## 验证结果

新增回归测试先复现了随机状态、同输入、点序和内点映射问题；补充的两层同分平面测试也先失败再通过。最终全项目 unittest 为 135 项通过，包含 6 项新增回归测试。独立代码复核未发现剩余实质问题。

实际文件验证采用 CPU，PyTorch 2.14.1+cpu、Open3D 0.20.0、NumPy 2.4.6、Torch 4 线程。目录文件交替正反顺序运行；一个样本补足 20 次，再由新进程复核。另在软件默认的 24 线程设置下，对 214214 个固定预处理点连续推理两次，分类标签哈希一致；该补充检查不等于跨线程设置逐位一致性保证。

| 文件 | 完整运行次数 | 最大角点两两位移（mm，未取整雷达角点） | 每次均有板面角点 |
|---|---:|---:|---|
| LaserData-socket1-2026-10-15-09-08-29.pcd | 3 | 0.000000000 | 是 |
| LaserData-socket1-2026-10-15-10-18-53.pcd | 3 | 0.000000000 | 是 |
| LaserData-socket1-2026-10-15-11-32-55.pcd | 20 | 0.000000000 | 是 |
| LaserData-socket1-2026-10-15-12-51-14.pcd | 3 | 0.000000000 | 是 |

当前有效范围验收：通过。同时比较了中间点集和标签哈希、阶段点数及完整几何结果；新进程检查为 {'result': True, 'stage_counts': True, 'labels_sha256': True, 'xyz_sha256': True}。

用户在运行期间明确指出并删除了 `LaserData-socket1-2026-10-14-12-19-18.pcd` 问题文件。本轮早期记录仍含该文件的失败提取结果，所以未筛选原始报告的总状态为失败；它没有被改写成通过。`summary.json` 明确记录排除理由，并只对当前目录内剩余 4 份有效输入验收。没有为了让问题文件输出角点而降低阈值或挑选种子。

## 尚未解决的精度风险

可重复性不等于准确性，也不等于面对噪声和缺点的鲁棒性。另在一组固定的 214214 个预处理点上进行探索性扰动：

- `drop_1_percent`：保留点中的分类变化约 1.54%，最大角点位移约 89.83 mm。
- `noise_sigma_2mm`：保留点中的分类变化约 1.18%，最大角点位移约 213.45 mm。

这两个扰动幅度是诊断选择，不代表已测得的雷达噪声，也不是准确度真值。它们说明当前语义硬标签、每片单个极值点、独立四边拟合之间存在明显的放大效应。下一轮应研究：语义只给候选区域，保留边界裕量，用原始点或细点局部精测边界，并记录边覆盖与退化状态。不能靠固定种子或外部坐标偏置解决这一问题。

GPU、不同依赖版本、任意外部线程同时使用全局 Torch RNG 的情形尚未验证；当前证据限定在上述 CPU 环境。参数正确性、目标归属、独立实测角点精度仍需单独验证。

## 耗时与记录

当前有效样本的单次全流程耗时中位数约 21.79 秒，范围 17.71–25.57 秒。测试期间有部分其他检查并行运行，因此这些时间是环境记录，不能作为严格性能对照。

- [algorithm_modules/point_cloud_segment_module/point_cloud_pipeline.py](C:/Users/18252/Desktop/jushen_field_test/algorithm_modules/point_cloud_segment_module/point_cloud_pipeline.py)
- [tests/test_point_cloud_repeatability.py](C:/Users/18252/Desktop/jushen_field_test/tests/test_point_cloud_repeatability.py)
- [tools/check_point_cloud_repeatability.py](C:/Users/18252/Desktop/jushen_field_test/tools/check_point_cloud_repeatability.py)
- [outputs/point_cloud_repeatability_20261015/summary.json](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_repeatability_20261015/summary.json)
- [outputs/point_cloud_repeatability_20261015/verified/repeatability.json](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_repeatability_20261015/verified/repeatability.json)
- [outputs/point_cloud_repeatability_20261015/robustness_probe.json](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_repeatability_20261015/robustness_probe.json)
- [outputs/point_cloud_repeatability_20261015/default_threads_control.json](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_repeatability_20261015/default_threads_control.json)
- [outputs/point_cloud_repeatability_20261015/all_tests.log](C:/Users/18252/Desktop/jushen_field_test/outputs/point_cloud_repeatability_20261015/all_tests.log)
