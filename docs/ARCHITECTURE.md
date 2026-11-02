# 架构与维护边界

## 实时控制链

`ab4_top.v` 是唯一生产顶层。`ads1115_reader` 从 ADS1115 读取电位器，`encoder_quadrature` 采集横臂编码器；`swing_controller` 组织起摆、捕获、稳摆、回落和翻转的状态切换。默认使用 WT01 平衡 PD 路线，1€ 滤波与 LQR 等已有分支仍属于有效依赖。

`arm_task` 产生定点、恒速与正弦参考；`velocity_stream` 把上位机速度曲线转换为受加速度限制的速度流。`flip_maneuver`、`transition_control` 及对应角度/输出保护处理特殊动作。`motor_tb6612` 产生 PWM / 方向 / STBY，顶层对四路输出保留 S0 原始电平门控。

输入与控制节拍 1 kHz；ADC 配置、10 kHz PWM、保护阈值和全部增益均沿用基线。编码器输出轴每圈 1040 计数。代码里某些名称仍带 LQR 或旧版本，不能凭命名删除：实际模块依赖以及历史协议兼容仍会使用它们。

## 上位机链

```text
USB bytes → Decoder / normalize → Engine → HTTP → browser
              ├─ 原始记录                  ├─ 姿态、曲线、统计
              └─ HistoryExport             └─ 本地显示 Reset
browser → Controls / Trajectory / Flip / ReturnMotion → UART → FPGA
```

`server.py` 负责本机服务、线程与共享状态；各动作 Python 模块维护握手与租约。网页修改显示坐标不会回写传感器数据。历史导出读取已保存文件，不抢占串口。`protocol_vendor.py` 和 `telemetry.py` 保留旧版本解析，供已存在的实验数据回放与导出。

C4 用于网页所有权和常规任务，C3 用于速度流、翻转及生命周期动作。当前固件遥测为 V8 / 172 字节 / 50 Hz / 115200 8N1。细节见 `UI01_protocol.md` 与 `UI02_curve_protocol.md`；UI01 文档仅作为继承协议背景，UI02 的参数差异以后者为准。

## 不应混淆的坐标

- FPGA 控制零点、固定停泊位、当前任务目标不是同一个变量。
- Reset 与固定显示偏差仅影响画面；不改变定位任务的物理参考。
- 姿态横臂角度按一圈取模，时间曲线保留连续位移，以避免 0/360° 截断。
- 显示的摆杆 0/180° 使用既有显示映射，不是新做的机械精密标定。

## 测试边界

自动回归包含实际 RTL 仿真、协议序列化、模拟 PTY、HTTP 所有权、旧记录导出与显示数学断言。简化物理模型用于局部回归，不等于带配重实物的稳定性证明。本次代码整理不启动电机、不重做实机实验。
