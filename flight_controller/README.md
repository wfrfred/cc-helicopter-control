# 飞控运行时说明

这个目录是直升机飞控的运行时部分。它的核心目标是把 UI 输入和传感器状态转换成旋翼执行命令，同时把模式状态、控制器状态和遥测输出保持在清晰的 ownership 边界内。

## 总览

```text
          UI / rednet input
                 |
                 v
        +------------------+
        | protocol/input   |
        +------------------+
                 |
                 v
        +------------------+        +------------------+
        | tasks/control    | <----> | tasks/sensor     |
        +------------------+        +------------------+
                 |
                 v
        +------------------+
        | state/mode_state |
        +------------------+
                 |
                 v
        +------------------+
        | active mode      |
        +------------------+
                 |
                 v
        +------------------+
        | control/*        |
        +------------------+
                 |
                 v
        +------------------+        +------------------+
        | hardware/mixer   | -----> | protocol/actuator|
        +------------------+        +------------------+
                 |
                 v
        +------------------+
        | telemetry/terms  |
        +------------------+
```

控制链路只保留一条主路径：输入和传感器状态进入模式层，模式层产出控制目标，控制器计算执行命令，mixer 生成桨距消息，最后由 actuator protocol 发出。

## 文件树

```text
flight_controller/
  config.lua              飞控参数、硬件参数、PID 和导航配置
  startup.lua             飞控启动入口
  navigation.lua          航点导航运行时

  tasks/
    control_task.lua      主控制循环
    sensor_task.lua       传感器读取和坐标整理
    input_task.lua        输入接收
    telemetry_task.lua    遥测发送

  protocol/
    input.lua             UI 输入解码
    actuator.lua          执行器输出协议

  state/
    mode_state.lua        模式选择和模式生命周期
    flight_state.lua      飞行就绪状态和传感器 age 报告
    height_lock.lua       高度释放/捕获状态机
    heading_lock.lua      航向释放/捕获状态机

  modes/
    manual.lua            手动姿态和手动轴输入
    position_hold.lua     位置保持
    cruise.lua            巡航速度保持
    navigation.lua        导航模式包装
    axis_locks.lua        manual / position_hold 内部使用的轴锁组件
    common.lua            模式目标的公共结构

  control/
    controller.lua        控制器编排入口
    horizontal.lua        水平位置/速度控制
    vertical.lua          高度/垂直速度控制
    attitude.lua          姿态和角速度控制
    allocation.lua        控制分配和输出限幅

  hardware/
    rotor_phase.lua       旋翼相位读取
    mixer.lua             桨距混控

  telemetry/
    terms.lua             遥测结构整理

  lib/
    mathx.lua             飞控语义相关数学辅助
    pid.lua               PID
    feedforward.lua       前馈
    attitude_math.lua     姿态计算
    attitude_allocator.lua 姿态通道分配
```

## 模块划分

`tasks` 是运行时循环层。它负责调度输入、传感器、控制、遥测这些长期任务，不保存飞控语义状态。

`protocol` 是外部边界层。UI 输入在这里变成飞控内部输入；执行命令在这里变成外设消息。

`state` 保存跨帧状态。`mode_state` 只负责模式选择和生命周期调度；飞行就绪状态和轴锁状态机分别放在更小的状态模块中。

`modes` 是飞行模式层。模式自己拥有自己的目标快照和内部状态。`manual` 和 `position_hold` 使用轴锁；`cruise` 持有进入时冻结的速度、高度和航向；`navigation` 使用导航运行时的目标。

`control` 是控制器层。它只关心当前状态和模式层给出的控制目标，不知道 UI 输入细节，也不决定当前飞行模式。

`hardware` 是硬件相关层。相位读取和桨距混控分开，mixer 只负责桨距公式。

`telemetry` 是观测层。它整理运行时状态给 UI 使用，不反向参与控制决策。

## 模式

`manual` 表示操作者正在直接给姿态、爬升或航向输入。姿态目标会随输入积分，进入 manual 时会从当前姿态初始化，避免上一次 manual 目标残留。高度和航向由 manual 内部的轴锁处理释放后的捕获。

`position_hold` 表示水平位置保持。进入时捕获当前水平位置。高度和航向同样由模式内部的轴锁处理。

`cruise` 表示水平速度保持。它只能从 manual 进入，进入时捕获当前水平速度，同时冻结当前高度和航向。按住 manual 输入进入 cruise 是预期行为，不需要先松手，也不会立刻退回 position_hold。

`navigation` 表示航点导航。它覆盖水平位置、高度和航向目标。导航退出后的目标模式由退出原因决定：横向或航向手动输入进入 manual，只有爬升输入进入 position_hold，导航完成或取消也回到 position_hold。

## 控制器

控制器更接近两条外环加一个姿态内环的结构。水平控制输出姿态需求，垂直控制输出总距，姿态控制再把姿态目标变成三轴力矩命令，最后由 allocation 做通道分配和限幅。

```text
 horizontal target      current position/velocity
        |                         |
        v                         v
   +----------+   pos/vel error   +------------------+
   | position | ----------------> | horizontal ctrl  | ---- roll/pitch target
   | velocity |                   +------------------+
   +----------+                             |
                                            v
 vertical target        current height/speed      heading target
        |                         |                      |
        v                         v                      v
   +----------+   height/speed    +------------------+   +----------------+
   | height   | --------------->  | vertical ctrl    |   | attitude target|
   | speed    |                   +------------------+   +----------------+
   +----------+                            |                     |
                                           v                     v
                                    collective command     target orientation
                                                                 |
                                                                 v
 current orientation/rates --------------------------------> +------------+
                                                            | attitude   |
 external feedforward ------------------------------------> | rate ctrl  |
                                                            +------------+
                                                                  |
                                                                  v
                                                            raw commands
                                                                  |
                                                                  v
                                                            +------------+
                                                            | allocation |
                                                            +------------+
                                                                  |
                                                                  v
                                                           actuator command
```

`horizontal` 负责水平位置和速度误差，输出姿态需求。`vertical` 负责高度和垂直速度，输出 collective。`attitude` 负责姿态误差、角速度目标和角速度 PID。`allocation` 负责姿态通道分配、限幅和最终命令。

每层拥有自己的诊断项，顶层 controller 只做编排和拼接。最终执行命令只以顶层 command 为准，遥测里的控制项用于观察，不作为下一帧控制输入。

## 运行时约束

飞控运行时直接使用 ComputerCraft/CC:Sable 提供的全局能力，例如 `vector`、`matrix`、`quaternion`、`rednet`、`peripheral`、`parallel` 和 `sleep`。本地测试可以安装这些全局能力，但飞控生产代码不导入本地测试桩。

传感器字段齐全后，控制循环保持运行。传感器 age 超过阈值只通过 flight 状态报告 warning 或 fault，不会直接把执行命令归零。输入超时是另一条路径，会按配置使用默认零输入。

模式目标是控制器输入；遥测是观测输出。控制逻辑不读取遥测页面状态，也不通过 UI 调试字段改变控制行为。

## 验证

常规修改后运行：

```sh
sh tools/check_lua.sh
lua tools/smoke_test.lua
lua tools/run_control_fixture.lua
```
