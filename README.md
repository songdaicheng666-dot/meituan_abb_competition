# ABB IRB 1090 Python TCP 控制验证

本项目通过 RobotWare 7 的 Socket Messaging 功能，验证 Python 能否控制
ABB IRB 1090 在两个固定点之间运动，不需要 EGM。运动仍由 RAPID 执行，
Python 只能请求预先示教的 `HOME` 和 `TEST` 两个点。

## 安全边界

本软件不是安全系统。执行任何运动前必须：

- 完整备份控制器；
- 清空机器人工作区域，检查吸盘、相机和线缆；
- 使用 **Manual Reduced Speed（手动低速）**，速度倍率不超过 10%；
- 将 FlexPendant 三位使能开关保持在中间档；
- 确保急停按钮随时可用；
- 运动命令超时后，必须先检查机器人，禁止直接重发 `MOVE`。

Python 不会远程上电、启动 RAPID、复位故障或实现急停。第一次验证必须在
机器人现场有人监护。

## 文件与运行条件

- `rapid/PythonBridge.modx`：RobotWare 7 RAPID TCP 服务端。
- `python/abb_client.py`：Python 3.10+ 标准库客户端。
- `tests/test_abb_client.py`：协议和本地假服务端测试。
- `docs/controller_backup_findings.md`：工具、相机、吸取动作和既有运动的备份提取记录。

默认网络参数：

```text
控制器：    192.168.125.1
Python主机：192.168.125.220
TCP端口：  55000
```

本机已经按用户范围安装 Python 3.12。安装完成前已经打开的终端可能没有刷新
`PATH`，如果 `python --version` 仍打开应用商店或提示不可用，请关闭并重新打开
PowerShell。也可以直接运行：

```powershell
& "$env:LOCALAPPDATA\Programs\Python\Python312\python.exe" --version
```

RobotStudio 自带的脚本目录不是可直接运行本客户端的 Python 解释器。

## 1. 备份控制器

在 RobotStudio 中选择已连接的控制器，点击 **Backup** 创建完整备份。加载或
修改 RAPID 代码前，先停止 RAPID 程序运行。

现有备份目录不会被桥接程序修改，并已从 Git 提交范围中排除。

## 2. 已从控制器备份确认工具数据

已分析 `1090-500097_BACKUP_2026-09-15`，得到以下关系：

- `T_ROB1` 的全局系统模块声明了 `tool_W`、`tool_P`、`tool_D` 和 `tool_B`；
- 备份中的持久变量为 `toolState := 2`；
- 现有 `Get_toolP()` 在完成 P 工具换装后也会写入 `toolState := 2`；
- 码垛、取放和相机准备位 `Redy_Camara_Palletizing` 的现有运动指令均使用
  `tool_P`。

因此第一版桥接程序直接引用控制器中已有的全局 `tool_P`，不再复制一个独立的
`pyTool`。桥接程序同时要求 `toolState = 2`；不满足时 `GET_STATE` 返回
`WRONG_TOOL`，所有 `MOVE` 命令返回 `ERR WRONG_TOOL` 且不会运动。

备份中 `tool_P` 的 TCP 为 `[13.9862, -1.15307, 157.969]` mm，姿态四元数为
`[1, 0, 0, 0]`。其负载质量被配置为 `0.001 kg`。桥接程序保留并复用这套现有
数据，但 `0.001 kg` 对实际吸盘/相机组合而言值得设备负责人后续复核；本次低速
可行性验证不得擅自猜测并修改负载参数。

加载模块前仍需目视确认当前实际安装的是现有程序定义的 P 工具。`toolState`
只是软件记录，不能替代现场确认；如果机械端实际换过工具但状态未同步，不得运动。

## 3. 加载 RAPID 模块

1. RobotStudio 连接 `192.168.125.1`。
2. 停止 RAPID 任务并点击 **Request Write Access**；如果控制器要求认证或
   FlexPendant 确认，由现场操作人员完成。
3. 在 **RAPID > T_ROB1** 下使用 **Load Module**（不同语言版本名称可能稍有
   差异），选择 `rapid/PythonBridge.modx`。
4. 检查 RAPID 错误列表。存在任何语法、变量冲突或工具数据错误时不得继续。

模块入口是 `PythonBridgeMain`，没有新增 `main`，不会替换现有应用入口。

## 4. 示教两个固定点

示教发生在模块加载完成之后、启动 TCP 服务之前。

1. 控制器切换到 **Manual Reduced Speed**，速度倍率不超过 10%。
2. 机器人位于当前已经确认安全的姿态时，在示教器上选择/调用
   `CapturePyHome`，保持使能开关并执行一次。该例程只读取 `CJointT()`，
   不会主动移动机器人。
3. 使用示教器把 TCP 沿现场确认无遮挡的水平方向点动约 50 mm。
4. 选择/调用 `CapturePyTest` 并执行一次。
5. 在 RAPID Data 中确认 `pyHomeCaptured` 和 `pyTestCaptured` 都为 `TRUE`。
6. 手动点动回 HOME 姿态，并保存模块/控制器备份，使持久变量得以保留。

只有当前 `toolState = 2`，并且 `pyHomeCaptured` 和 `pyTestCaptured` 都为 `TRUE`
时，服务端才接受运动命令。工具状态不对时返回 `ERR WRONG_TOOL`；点位尚未捕获时
返回 `ERR NOT_CONFIGURED`。

## 5. 启动服务并测试端口

1. 把程序指针设置到 `PythonBridgeMain`。
2. 从 FlexPendant 启动该例程，应该看到：

   ```text
   PyBridge listening: 55000
   PyBridge waiting for PC
   ```

3. 只有看到以上信息后，才在电脑执行：

   ```powershell
   Test-NetConnection 192.168.125.1 -Port 55000
   ```

`Test-NetConnection` 会建立连接后立即断开，因此示教器显示一次
`client connected` 和 `client disconnected` 属于正常现象。

如果 RAPID 已经显示正在监听，但 TCP 测试仍失败，再在控制器的
**Firewall Manager** 中允许 TCP 入站端口 55000。不要修改
**Connected Services**。如果 `SocketBind` 本身报错，应停止操作并核实控制器
网口地址，不要盲目修改网络设置。

## 6. 执行 Python 验证

`--host` 等全局参数必须写在子命令前面。

```powershell
python python\abb_client.py ping
python python\abb_client.py ping-loop --count 20
python python\abb_client.py state
python python\abb_client.py joints
```

如果新终端中的 `python` 命令仍不可用，把上述命令的 `python` 替换为：

```powershell
& "$env:LOCALAPPDATA\Programs\Python\Python312\python.exe"
```

通信和关节读数验证正确后，现场人员保持使能开关，首先测试 HOME：

```powershell
python python\abb_client.py home
```

在确认提示中输入 `HOME`。机器人此时应该已经位于 HOME，不应出现明显移动。
随后测试约 50 mm 的运动：

```powershell
python python\abb_client.py test
python python\abb_client.py home
```

`--yes` 可以跳过文字确认，第一次真机测试禁止使用。持续连接的交互模式为：

```powershell
python python\abb_client.py interactive
```

正常运动会先收到 `OK ACCEPTED <目标>`，完成后收到 `OK DONE <目标>`。
客户端不会自动重试运动命令。

## 7. 本地自动测试

以下测试只使用本机假服务端，不连接也不会移动机器人：

```powershell
python -B -m unittest discover -s tests -v
```

## 故障排查与回退

- `WRONG_TOOL` / `ERR WRONG_TOOL`：控制器中的 `toolState` 不是 2。停止测试，
  先由现场人员核对实际工具和现有换刀流程；禁止只为绕过检查而手改状态值。
- `NO_HOME` / `NO_TEST`：执行相应的点位捕获例程。
- `ERR NOT_CONFIGURED`：`pyHome` 或 `pyTest` 至少有一个尚未完成捕获。
- 连接超时：先确认 `PythonBridgeMain` 正在运行并显示监听信息，再检查端口和
  防火墙。
- 运动超时：禁止重发，立即检查机器人状态和控制器 Event Log。
- RAPID `ERR MOTION`：停止测试，先排查控制器的运动错误。

需要回退时，停止 RAPID，从 `T_ROB1` 卸载 `PythonBridge`，把程序指针重置到
现有应用的 `main`；必要时恢复最初备份。
