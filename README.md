# ACG720 旋转倒立摆 · UI02

基于 ACG720 / GW5AT-138C 的旋转倒立摆，FPGA 完成传感器采集、控制计算、电机驱动和状态保护，Mac 本地网页上位机负责显示、任务下发、记录与回放。

本仓库整理自已实测的 `WT01E2_UI02_16点高速曲线_20261028`。这是保持功能的代码清理版，不是新的控制算法版本。固件标识继续使用 **UI02 / V8**。

## 项目成员

协作成员（GitHub）：

- [ZongbaoMai](https://github.com/ZongbaoMai)
- [FairyWarehouse](https://github.com/FairyWarehouse)

## 功能

- 自动起摆、捕获与稳摆；WT01 的 PD 主控制路线、1€ 滤波和现有其他控制分支均保留。
- 横臂八方位定位、恒速与正弦任务。
- 2～16 点自绘速度曲线，校验单值时间、速度、加速度及起止速度；可在运行中应用有效修改。
- 同向翻转 360°、受控回落、快速起摆（后两者仍有已知实测限制）。
- 大字姿态显示、横臂动态放大、速度曲线、Reset 显示归零。
- 原始串口记录、历史回放、分组导出；带 BOM、CRLF 的 CSV 可供 Windows 使用。
- 板上 S0 独立急停、通信续期、传感器及运动保护。

## 目录

```text
fpga/
  src/                         28 个有效 RTL 模块
  constraints/                 引脚与时序约束
  acg720_one_euro_WT01E2.gprj   Gowin 工程
  build.tcl                    构建入口
Mac上位机/
  server.py                    本机 HTTP 服务、串口与回放
  controls.py                  网页控制所有权与 C4 协议
  trajectory.py                自绘曲线与 C3 速度流
  flip.py / return_motion.py   特殊动作请求与状态跟踪
  telemetry.py / protocol_vendor.py
  history_export.py            历史数据导出
  web/                         无需前端打包的网页资源
  data/                        4 份真实 WE20 历史回放样例
  tests/                       解码、显示、回放与导出测试
  启动上位机.command / 停止上位机.command
tests/                         RTL 与任务/串口/HTTP 回归
tools/                        构建、验证、SRAM 校验工具
docs/                         架构、协议、已知限制与清理说明
.github/workflows/offline.yml  离线自动检查
```

### 启动上位机（Mac）

安装 Python **3.9+**，双击根目录 `启动Mac上位机.command`；也可执行：

```sh
python3 Mac上位机/launch.py
```

默认地址 `http://127.0.0.1:8777/`，占用时在 8777～8786 中选择空闲端口。服务仅绑定本机；Python 服务与网页不需要 pip 或 npm 运行依赖。换一台电脑时串口名称可能改变，请从界面选择。

连接并记录 → 启用网页控制 → 选择起摆方式并起摆 → 稳摆后选择横臂任务或“更多动作”。接管期间板上 S1～S4 忽略，释放后恢复板上操作；S0 始终有效。Reset 仅改变浏览器显示坐标，不改变 FPGA 控制参考。

未接板时可切换“记录回放 / 导入”，选择内置 WE20 历史记录。它们不是 UI02 实测性能证据。新记录写入 `Mac上位机/recordings/`；新导入文件写入 `data/import_*.bin`，这些本地数据已设置 Git 忽略。导入也会更新 `data/catalog.json`，提交时不要把个人导入目录条目带入。

当前串口实现使用 POSIX `termios` / `fcntl`，本版实机上位机面向 Mac；不是 Windows 串口移植版。Windows 可使用导出的 CSV / JSON / 原始数据制作视频。

### 编译 FPGA

安装支持 GW5AT-138C 的 Gowin EDA。可以直接打开 `fpga/*.gprj`，或使用：

```sh
python3 tools/build_gowin.py --ide /path/to/Gowin_EDA/IDE --prefix local_build
```

Mac 默认 IDE 路径已保留；其他安装位置用 `--ide` 指定。构建产物在 `fpga/impl/pnr/`，审计结果在 `reports/`，均不提交。工具不仅检查返回码，还检查新产物、源文件一致性和实际时序报告。FPGA 时钟为 50 MHz。

本次保留原有已上板 `.fs` 为独立发布附件，不混入源码仓库。它的 SHA256、来源和与本仓库的对应关系见 [固件说明](docs/FIRMWARE.md)。当前下载方式是 SRAM，主电源断电后程序丢失。此整理版没有执行下载或 Flash 固化。

### 离线回归

依赖 Python 3.9+、Node.js 18+ 和 Icarus Verilog，命令在 PATH 可用后执行：

```sh
python3 tools/validate.py
```

测试在临时副本中执行，包括 RTL、合成串口 PTY、HTTP、历史导出和显示数学断言，不访问真实 USB。结果写入 `reports/offline_validation.json`。GitHub Actions 使用同一入口；不需要 FPGA、Gowin 授权或电机。

代码格式：Python 使用 Ruff 0.14.3；网页使用 Prettier 3.6.2，配置见 `.prettierrc.json`。这些工具仅用于开发，不是上位机启动依赖。

## 功能边界

本次没有修改运动控制参数、状态机、保护条件、通信心跳与超时。28 份在用 RTL 和两份约束与实测基线逐字节相同；仅从工程列表移除了未实例化的两份红外模块。上位机删除无用符号、不可达绘图分支和闲置样式，并整理格式。

受控回落原因码 9、快速起摆准入和网页断连导致停止等问题仍然存在，不能把代码清理当作这些问题已修复。参见 [已知限制](docs/KNOWN_ISSUES.md) 和 [清理说明](docs/CLEANUP.md)。

## 来源与许可

项目包含同学提供的工程、WT01 / WHEELTEC 算法移植及多次自研扩展。当前没有为整个工程指定新的开源许可证；公开仓库不自动授予修改或再分发许可。请在选定统一许可证前核对原始来源与授权，见 [来源说明](THIRD_PARTY_NOTICES.md)。
