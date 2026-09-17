# IRB 1090 连续关节轨迹录制与重放

## 功能边界

该功能记录示教器点动时的六轴关节位置，并通过已有 Python TCP 桥接程序以统一低速重放。

- 仅验证一条固定、空载轨迹。
- 必须从 `pyHome` 开始，可以在任意已确认安全的姿态结束。
- 不控制吸盘、相机或任何 I/O。
- 重放的是关节路径，不保证 TCP 沿直线移动。
- Python、RWS和TCP均不是安全系统，急停、三位使能和现场监护始终有效且必不可少。

## 1. 更换控制器模块

1. 再次备份控制器。
2. 停止 RAPID，申请 RobotStudio 写权限。
3. 使用新版 `rapid/PythonBridge.modx` 替换控制器中的 `PythonBridge` 模块。
4. 确认模块名称仍为 `PythonBridge`，入口仍为 `PythonBridgeMain`，且错误列表为空。
5. 检查 `pyHomeCaptured` 和 `pyTestCaptured`。模块替换可能保留持久值，但不能依赖这一行为：
   如果任何值变成 `FALSE`，必须重新执行对应的捕获例程。
6. 确认当前实物仍为 P 工具，且 `toolState=2`。

新版模块保留原有 `PING`、状态、关节读取、HOME和TEST命令，并增加流式轨迹协议。

## 2. RWS账号和证书

录制阶段使用 RobotWare 7 的只读 HTTPS RWS，不申请RAPID写权限。设置当前PowerShell会话的账号：

```powershell
$env:ABB_RWS_USER = "Default User"
$env:ABB_RWS_PASSWORD = "<控制器UAS密码>"
```

密码只存在当前进程环境中，不写入JSON。推荐导出并信任控制器证书：

```powershell
python python\trajectory.py --ca-cert C:\path\controller-ca.pem rws-check
```

如果目前只有控制器自签名证书，可在隔离实验室网络中临时显式使用：

```powershell
python python\trajectory.py --insecure rws-check
```

检查过程持续10秒。机器人必须保持静止，成功标准为：

- 能读取 `ROB_1`、`PythonBridge/pyHome`、`pyHomeCaptured` 和 `Modul1/toolState`；
- `pyHomeCaptured=TRUE`、`toolState=2`；
- 采样率中位数不低于10 Hz；
- 最大采样间隔不超过250 ms。

RWS检查失败时不要开始录制。先处理账号、HTTPS证书、端口或控制器服务问题。

## 3. 录制轨迹

1. 控制器处于 **Manual Reduced Speed**，速度倍率不超过10%。
2. 现场清空机器人工作区域，保证轨迹与障碍物至少留有50 mm试验间隙。
3. 启动 `PythonBridgeMain`，用现有命令返回HOME：

   ```powershell
   python python\abb_client.py home
   ```

4. 目视确认机器人位于 `pyHome`，然后从示教器停止 `PythonBridgeMain`。同一个 `T_ROB1`
   不能在运行桥接运动程序的同时进行示教器点动。
5. 开始录制：

   ```powershell
   python python\trajectory.py --insecure record --file trajectory.json
   ```

6. 第一次按回车后有5秒倒计时。倒计时结束后，使用示教器连续点动机械臂。
7. 走完模拟取料、转运、放料动作后，停在已确认安全的目标终点。
8. 机器人完全停稳后在电脑按回车。不要在机器人仍移动时结束录制。

录制没有固定时间限制，原始样本会持续写入 `trajectory.json.part`。正常结束后生成正式JSON并删除
临时文件。如果RWS断开或程序异常，临时文件可能保留，但不能用于重放。

目标文件已经存在时程序默认拒绝覆盖；确认需要替换时显式添加 `--overwrite`。

## 4. 离线校验

```powershell
python python\trajectory.py validate --file trajectory.json
```

只有显示 `VALID trajectory` 才能继续。校验包括：

- 原始采样率和采样缺口；
- 首点距 `pyHome` 不超过0.2度；终点可以是任意安全姿态；
- `pyHomeCaptured=TRUE`、录制时 `toolState=2`；
- 重放点全部为有限六轴数值；
- 相邻重放点任一轴变化不超过1.0度；
- 重放点SHA-256与文件记录一致。

预处理会删除小于0.02度的连续静止点，以0.05度容差压缩冗余点，再插值保证最大关节步长。
原始样本和时间戳仍会完整保留，但重放不会复现人工点动的速度和停顿。

## 5. 空载低速重放

1. 从示教器重新启动 `PythonBridgeMain`，看到监听55000端口的信息。
2. 保持手动低速、倍率不超过10%，现场人员持续持有三位使能开关。
3. 确认吸盘没有抓取物体，工作区无人且无遮挡。
4. 执行：

   ```powershell
   python python\trajectory.py play --file trajectory.json
   ```

5. 输入完整的 `PLAY` 才会发送轨迹。控制器再次检查当前姿态和首点都在 `pyHome` 的0.2度范围内。重放完成后机器人停在录制终点。
6. 第一次只重放一次并全程观察；成功后再重复两次。

RAPID以 `MoveAbsJ`、`v10`、中间点 `z0`、末点 `fine` 和 `tool_P` 执行。Python每25点等待一次
控制器进度响应，整个动作不会自动重试或循环。

## 6. 错误与恢复

- `ERR NOT_AT_HOME`：当前位置或轨迹首点不是 `pyHome`。机器人不会自动寻找起点；开放终点轨迹完成后必须先由现场人员安全返回Home，才能再次播放。
- `ERR BAD_POINT`：轨迹帧格式或关节值无效。
- `ERR STEP_TOO_LARGE`：相邻两点任一轴超过1.0度。
- `ERR STREAM_LOST`：流式传输超时或断线。
- `ERR WRONG_TOOL`：`toolState` 不是2。
- `ERR NOT_CONFIGURED`：`pyHome` 尚未捕获。
- `ERR RECOVERY_REQUIRED`：上次重放已进入错误状态，必须现场检查并从
  `PythonBridgeMain` 重新启动，不能简单续跑原来的程序指针。

首点被接受后的任何错误都按“机器人状态未知”处理：停止发送命令，检查实际机器人位置和控制器
Event Log，不得自动重连续播。传输中断会使RAPID进入 `ERROR` 并停止，需现场检查后重新把程序
指针设到 `PythonBridgeMain`。

当前 `tool_P` 的备份质量仅为 `0.001 kg`。本阶段只允许既有末端设备的空载低速试验；带物运行或
提高速度前必须由设备负责人重新核定工具质量、重心和物体负载。
