# ITL 转换器部署与验证

截至 2026-10-14，ITL 复制、转换任务排队及外部命令调用接口可以接入现场流水线。原包 EXE 因缺少依赖无法启动，补齐官方 32 位 PCL/OpenNI2 运行库后已在本机隔离目录启动 GUI；三次真实样例试验均在载入 ITL 时退出 `0xC0000409`，未生成 PCD。默认转换命令保持空列表，应用正常启动不会调用厂商 GUI，日志会明确报告转换器未就绪；不能将压缩包自带 PCD 当成新 ITL 的转换结果。

## 软件中的使用方式

软件现在只保留“现场离线联调”。主窗口的“选择雷达 PCD”按钮默认打开项目的 `pcd/` 文件夹，也可以选择其他本地位置的 `.pcd` 文件。所选路径显示在按钮旁并保存，第二步雷达算法使用该文件。没有选择时，默认从 `pcd/` 找最新 PCD；没有可用数据会报错，不生成替代点云。正在运行的旧窗口需要关闭并重新打开才会加载改动。

ITL 监听仍由 `scripts/radar_sync.ps1` 执行，软件启动时自动启动，退出时停止。配置位于 `config/radar_pipeline.json`：默认源是 `\\10.11.207.112\D$\IN\点云显示1\点云显示(1)\wave`，本地目录是项目 `存itl/`，PCD 输出目录是项目 `pcd/`。仅复制 `.itl`，等待文件大小和修改时间稳定后才发布；复制中途变化会延后，单个无法读取的文件不阻断其他文件。默认只同步监听启动后修改的远程文件，需要补同步历史文件时将 `sync_existing` 改为 `true` 并重启软件。

连接可访问时输出 `CONNECTED`，复制成功输出 `COPIED`，失败输出原因。用 `启动软件_调试模式.bat` 启动可看到控制台输出，普通启动的同样内容写入 `logs/radar_sync.log`。实际工控机共享连通性仍需要现场内网验证。账号可由环境变量 `JUSHEN_RADAR_USER`、`JUSHEN_RADAR_PASSWORD` 指定；未提供时延用原脚本账号，日志不显示密码。

本地 ITL 转换队列、文件稳定等待、重试、结果校验和发布已实现；缺少已验证的转换程序时保留 ITL 并输出明确提示。转换成功记录写入 `logs/radar_conversion_state.json`，重启不重复转换未变化且结果仍存在的文件。更新的同名 ITL 输出另一个 PCD，保留旧结果。请将下文经过真实样例验证的参数列表填写到 `config/radar_pipeline.json` 的 `converter_command` 后重启。

本地验证结果：全部 93 项自动化测试通过；真实 PowerShell 脚本与测试转换进程完成复制、日志、发布和退出清理测试；软件无窗口启动验证正常退出。测试转换程序只验证流水线行为，不能替代厂商 ITL 转换验证。

## 厂商包检查结果

检查来源为 `C:\Users\18252\Desktop\to_pcd.zip`，原压缩包未修改。仅 EXE 和两个 INI 已提取到项目 `third_party/itl_converter/`，未自动安装软件或修改系统设置。

- `BZKJDEMO_pcd.exe`：4,110,848 字节，PE I386 / WINDOWS_GUI，32 位 MFC 应用；SHA-256 为 `d363bf8d91eb43ae0c277a8125ab748eb3f530d42467635157e444a6d43a3a7f`。
- 原包直接启动立即退出，十进制退出码 `3221225781`，即 `0xC0000135`（缺少运行 DLL）。
- 导入的以下 DLL 不在压缩包、旁边的 `radar_pcd_fetcher` 项目及本次检查的本机库目录中：`pcl_common_release.dll`、`pcl_features_release.dll`、`pcl_filters_release.dll`、`pcl_io_release.dll`、`pcl_search_release.dll`、`pcl_segmentation_release.dll`、`pcl_visualization_release.dll`。
- `pcl-1.8.1-pdb-msvc2017-win32.zip` 的 21 个文件只有 `.pdb` 调试符号，没有 `.dll`。嵌套 `to_pcd.zip` 同样只有 EXE、INI、日志和样例文件，没有运行 DLL。
- `C:\Windows\SysWOW64` 已有 `mfc140.dll`、`MSVCP140.dll`、`VCRUNTIME140.dll`；当前具体缺口为 PCL 运行库，恢复后仍可能发现其下级依赖。
- EXE 为 GUI 程序；尚未核实命令行入口和无人值守转换方式。不得假定它接受 `input.itl output.pcd` 参数。

后续已从 [PCL 官方 1.8.1 发布页](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.8.1) 下载 `PCL-1.8.1-AllInOne-msvc2017-win32.exe`（436,050,008 字节，SHA-256 `1180abde0aabd1bd4beb52d732ae2d396800d0402eb21cd2df4c1882abf2bebe`），使用 [7-Zip 官方下载页](https://www.7-zip.org/download.html) 提供的工具仅提取文件，未执行任何安装器。所需 DLL、嵌套 OpenNI2 MSI 中的 32 位原生 `OpenNI2.dll` 和原 EXE/INI 的副本最终放在被 Git 忽略的 `third_party/itl_converter/runtime/`（18 个运行文件共 30,822,823 字节），来源和逐文件哈希保存在相邻 `SOURCE_MANIFEST.json`；系统目录和注册表未改动。下载缓存与最初探测副本位于被忽略的 `workdir/converter_probe/`。

该隔离运行包已成功启动 `Intelly点云显示软件V1.0`。界面观察到“打开文件”（控件 ID 1007）、“打开PCD文件”（1005）、“显示点云”（1010）和“保存点云文件”复选框（1047）；“打开文件”弹出的标准文件对话框使用文件名控件 1148 和打开按钮 1，筛选为所有文件。启动时出现短暂串口错误，主窗口日志显示“串口已被占用”和“关闭网络”；本次未点击连接网络、开始旋转或任何设备控制按钮。恢复 GUI 能运行并不等于离线转换或自动化已验证。

`services/vendor_itl_cli.py` 是依据这些已观察控件实现的**实验性、未启用**离线适配器。它只对自己启动进程所属的窗口发送原生控件消息，不使用全局键盘/鼠标操作，不触发设备按钮；每次复制运行包和 ITL 到独立临时目录，启用保存点云复选框，再执行打开本地文件，必要时显示点云。它检查生成文件并停止自己启动的进程，外层 `convert_itl` 再次校验并原子发布最终 PCD。

实际样例依次试验了原始 ASCII 文件名、延长显示前等待时间，以及将输入放入与原包一致的 `wave/` 子目录。后两次明确在显示按钮触发前即退出，三次都没有生成 PCD。Windows Application Error（事件 1000）记录故障模块 `ucrtbase.dll`、异常 `0xC0000409`、偏移 `0x0002DA71`。目前不能确定是厂商运行包兼容性、读取格式还是程序内部缺陷；未据此猜改几何或设备参数。下一步需要厂商提供匹配该 EXE 的完整运行包并验证其样例读取行为。

原 GUI 探测进程及适配器产生的进程均已停止。首次试验的一份约 30 MB 临时目录 `C:\Users\18252\AppData\Local\Temp\vendor-itl-ly_a0hi5` 因清理操作被自动审批策略拒绝而保留，不会被正常流水线使用。

需厂商提供与该 EXE 匹配的完整 **32 位** 运行包，以及经过验证的命令行调用方式。如果只提供 GUI，应再验证其离线转换操作和可自动化接口，并避免启动实时设备连接。INI 中包含现场地址和扫描/安装参数，应保留厂商确认过的配置，不在调试过程中猜改几何参数。

## 配置外部命令

`services/itl_converter.py` 暴露：

```python
convert_itl(source, destination, command_template, timeout_s=120, cancel_event=None)
validate_pcd(path, cancel_event=None)
```

`command_template` 必须为参数列表，且同时包含 `{input}` 和 `{output}`。命令示意如下，路径和参数必须替换为厂商实际验证过的接口；这不是 `BZKJDEMO_pcd.exe` 的已知语法：

```json
[
  "C:/Vendor/VerifiedConverter.exe",
  "--input", "{input}",
  "--output", "{output}"
]
```

若厂商修复了样例读取，再验证本项目实验性适配器可使用以下参数；当前默认配置不会运行它：

```text
python services/vendor_itl_cli.py --input INPUT.itl --output OUTPUT.pcd --runtime-dir third_party/itl_converter/runtime --timeout 90
```

在 `command_template` 中，应使用 Python、脚本和运行包的绝对路径，例如 `["C:/.../python.exe", "C:/.../services/vendor_itl_cli.py", "--input", "{input}", "--output", "{output}", "--runtime-dir", "C:/.../third_party/itl_converter/runtime"]`。该适配器是针对已观察 GUI 的软件接口，仍需用真实样例验证成功，不能因为界面可访问就开启默认转换。

调用不经过 shell。空列表触发 `ConverterUnavailable`，不会制造 PCD。命令运行目录为可执行文件所在目录，方便使用相邻 DLL/INI；输入为绝对路径，输出为目标目录内唯一临时目录中的绝对 `.pcd` 路径。转换程序必须写入指定输出，并等待转换完成后退出；不能只启动后台子进程后立即返回。模板中需要传递字面大括号时使用 `{{` 和 `}}`。

退出码非零、超时、应用退出取消、缺失输出或 PCD 校验失败都会报错并清理临时文件。超时/取消会终止该转换命令及仍属于它的子进程。仅退出码为零且输出通过校验后，才使用同一文件系统的硬链接原子发布目标 PCD；目标已存在时拒绝覆盖，包括转换期间并发生成的同名目标。因此 PCD 输出目录应在支持硬链接的本机文件系统上，例如 NTFS。

PCD 校验要求含 `x/y/z` 字段、有效尺寸和点数。ASCII 会完整校验列数、数值、有限坐标及实际点数；binary 检查数据字节数；binary_compressed 检查尺寸、LZF 字面块/回溯引用和展开字节数。文件结构校验不能证明安装坐标系、运动补偿或几何精度正确。

## 验证边界

`python -m unittest tests.test_itl_converter -v` 覆盖真实本地测试命令、中文/空格路径、缺失运行程序、退出失败、无输出、截断数据、超时、应用退出取消、临时文件清理、并发同名文件保护及三种 PCD 编码。

包内 `LaserData-socket1-2025-12-03-12-27-50.pcd` 为 76,088,940 字节，声明 5,189,041 点；本次已完整验证其 ASCII 点数据。匹配 ITL 为 10,730,980 字节，包含 SICK `sSN LMDscandata` 原始报文；原始距离报文不足以确认厂商的安装变换和运动补偿。本项目没有重写该未公开转换算法。

取得完整运行包后，先用该配对样例执行真实转换，比较生成 PCD 的点数、坐标范围及几何/坐标系与样例是否一致，再填写流水线转换命令。当前自动化测试使用明确标记的测试程序，验证调用与发布行为，不作为真实 ITL 转换精度的证据。

LZF 块结构校验参照 [liblzf 的解码规则](https://github.com/nemequ/liblzf/blob/master/lzf_d.c)。
