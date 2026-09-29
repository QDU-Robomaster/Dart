# Dart

飞镖系统模块，把飞镖架的 yaw 云台、四个摩擦轮和推杆放在一个模块中控制。

## 工作方式

- 构造时创建线程 `dartThread`（栈深 `param.task_stack_depth`，优先级 `MEDIUM`），
  每轮循环后休眠 2 ms。
- 模式：`GetEvent()` 返回的 `LibXR::Event` 注册了 `Dart::DartMode` 的四个值
  `RELAX`(0)、`YAW_COMMON`(1)、`YAW_SCAN`(2)、`GAME`(3)，激活对应事件 ID 即切换模式；
  `SetMode(mode)` 等价于激活该事件。CMD 的 `CMD_EVENT_START_CTRL` 切到 `RELAX`，
  `CMD_EVENT_LOST_CTRL` 切到 `GAME`（遥控断开后进入比赛自动流程）。
- yaw 云台：yaw 输出轴角度由电机 `abs_angle` 的增量除以固定减速比 19.2032 累加得到，
  经角度环 `pid_yaw_angle` + 速度环 `pid_yaw_speed` 以 `MODE_CURRENT` 输出。状态机：
  - `INITIALIZING`：设定点每周期移动 2π/250，150 个周期后检测到 |扭矩| > 0.075 即认为到达机械
    限位，记为最小角，最大角 = 最小角 + 65 rad，转入 `SCANNING`；进入 `GAME` 模式时重新初始化。
  - `SCANNING`：以 8 rad/s 在最小角与最大角之间往返扫描。
  - `NORMAL_CONTROL`：目标角 = 当前角 + 上位机或遥控给出的 yaw 偏移，已标定时限制在
    [最小角, 最大角] 内。收到新的非零上位机 yaw 时进入该状态；上位机数据超过 100 ms 未更新时
    回到 `SCANNING`（未标定时回到 `INITIALIZING`）。
  - `RELAX` 模式下 yaw 电机 `Relax()`。
- pitch 电机始终下发 0 电流。
- 摩擦轮：四个 `RMMotor` 做转速环（rpm），前两路目标为 `fric2_setpoint_speed`，后两路为
  `fric1_setpoint_speed`，以 `MODE_CURRENT` 输出。停止和起转阶段 PID 输出限幅为 0.1，
  后右摩擦轮转速超过 `fric1_setpoint_speed` 后解除限幅并认为就绪。当前固定为单发模式：
  飞镖发射口状态为“正在打开”或“已打开”时启动摩擦轮，否则停止；该状态在 `GAME` 模式下来自
  裁判系统，在 `YAW_*` 模式下由遥控置位，切换到 `RELAX` / `YAW_*` 模式时复位。
- 推杆：上电后先向负方向寻找限位（250 个周期后 |扭矩| > 0.02），推杆角度按
  `push_motor_gear_ratio` 换算，最小位置 = 限位 + 2 rad，最大位置 = 限位 + 61 rad；角度环
  `push_motor_angle_pid` + 速度环 `push_motor_speed_pid`，`MODE_CURRENT` 输出。摩擦轮就绪、
  发射口已打开且推杆空闲时，发射命令的上升沿使推杆推向最大位置；检测到发射后停在当前位置，
  等待下一次上升沿。摩擦轮停止时推杆回到最小位置。
- 发射检测：摩擦轮就绪且推杆正在推出时，后左摩擦轮 |扭矩| > 0.1 视为一次发射，
  此后 100 ms 内 `launch_flag` 为 true。检测与发布只在 `GAME` 模式下执行。
- 遥控（`chassis_cmd`）：`YAW_COMMON` / `YAW_SCAN` 下 z > 0.7 时置发射口状态为已打开并产生
  发射命令；`YAW_COMMON` 下 x 直接作为 yaw 偏移。

Topic：

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `host_dart_gimbal_cmd` | 创建并订阅 | `Dart::DartGimbalCMD`（`yaw`） | 上位机给出的 yaw 偏移 |
| `param.fire_notify_topic_name`（默认 `fire_notify`） | 订阅 | `HostData::LauncherCMD`（`isfire`） | 发射命令 |
| `param.chassis_cmd_topic_name`（默认 `chassis_cmd`） | 订阅 | `CMD::ChassisCMD` | 遥控输入 |
| `param.launcher_cmd_topic_name`（默认 `launcher_cmd`） | 订阅 | `CMD::LauncherCMD` | 当前代码只订阅，不读取 |
| `param.launcher_ref_topic_name`（默认 `launcher_ref`） | 订阅（仅 `GAME`） | `Referee::LauncherPack` | 飞镖发射口状态 |
| `launch_flag` | 发布（仅 `GAME`） | `bool` | 发射检测标志 |

## 依赖

- `QDU-Robomaster/Motor`：yaw / pitch 电机的抽象接口。
- `QDU-Robomaster/RMMotor`：摩擦轮与推杆电机。
- `QDU-Robomaster/CMD`：CMD 事件与 `ChassisCMD` 类型。
- `QDU-Robomaster/Referee`：`LauncherPack` 类型。
- `QDU-Robomaster/HostData`：`LauncherCMD` 类型（`fire_notify` 通常由 `HostData` 创建）。

无外部软件包，仅使用 LibXR。

## 构造接口

```cpp
Dart(Motor& motor_yaw,
     Motor& motor_pitch,
     RMMotor& motor_fric_front_left,
     RMMotor& motor_fric_front_right,
     RMMotor& motor_fric_back_left,
     RMMotor& motor_fric_back_right,
     RMMotor& push_motor,
     CMD& cmd,
     const Param& param = {...});
```

依赖：

- `motor_yaw`、`motor_pitch`：`Motor`，yaw 与 pitch 电机。
- `motor_fric_front_left` / `front_right` / `back_left` / `back_right`：`RMMotor`，四个摩擦轮电机。
- `push_motor`：`RMMotor`，推杆电机。
- `cmd`：`CMD` 实例。

配置（`Param`，PID 为 `LibXR::PID<float>::Param`，依次为 `k, p, i, d, i_limit, out_limit, cycle`）：

- `task_stack_depth`：线程栈深，默认 4096。
- `pid_yaw_angle`：yaw 角度环，默认 `{1.0, 900.0, 0.0, 0.0, 0.0, 1000.0, false}`。
- `pid_yaw_speed`：yaw 速度环，默认 `{1.0, 0.001, 0.0, 0.0, 0.0, 1.0, false}`。
- `push_motor_gear_ratio`：推杆电机减速比，默认 36.0。
- `fric1_setpoint_speed`：后两路摩擦轮目标转速 (rpm)，默认 4500.0；也是就绪判定阈值。
- `fric2_setpoint_speed`：前两路摩擦轮目标转速 (rpm)，默认 4400.0。
- `fric_speed_pid_0..3`：四路摩擦轮速度环（前左、前右、后左、后右），默认均为
  `{1.0, 0.001, 0.0, 0.0, 0.0, 1.0, false}`。
- `push_motor_speed_pid`：推杆速度环，默认 `{1.0, 0.0008, 0.0, 0.0, 0.0, 1.0, false}`。
- `push_motor_angle_pid`：推杆角度环，默认 `{1.0, 1000.0, 0.0, 0.0, 0.0, 2000.0, false}`。
- `launcher_cmd_topic_name`、`chassis_cmd_topic_name`：订阅的 CMD 发射 / 底盘命令 Topic，默认
  `"launcher_cmd"`、`"chassis_cmd"`，须与 CMD 的同名参数一致。
- `launcher_ref_topic_name`：订阅的裁判系统发射数据 Topic，默认 `"launcher_ref"`。
- `fire_notify_topic_name`：订阅的上位机开火通知 Topic，默认 `"fire_notify"`。

## 使用

```sh
xrobot module add QDU-Robomaster/Dart
xrobot setup
xrobot instance add QDU-Robomaster/Dart
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把依赖填为已列出的电机与 CMD 实例的 id：

```yaml
modules:
  - module: QDU-Robomaster/Dart
    id: dart_0
    args:
      - motor_yaw: motor_yaw
      - motor_pitch: motor_pitch
      - motor_fric_front_left: motor_fric_front_left
      - motor_fric_front_right: motor_fric_front_right
      - motor_fric_back_left: motor_fric_back_left
      - motor_fric_back_right: motor_fric_back_right
      - push_motor: push_motor
      - cmd: cmd
      - param:
          task_stack_depth: '4096'
          pid_yaw_angle:
            - '1.0'
            - '900.0'
            - '0.0'
            - '0.0'
            - '0.0'
            - '1000.0'
            - 'false'
          pid_yaw_speed:
            - '1.0'
            - '0.001'
            - '0.0'
            - '0.0'
            - '0.0'
            - '1.0'
            - 'false'
          push_motor_gear_ratio: '36.0'
          fric1_setpoint_speed: '4500.0'
          fric2_setpoint_speed: '4400.0'
          fric_speed_pid_0:
            - '1.0'
            - '0.001'
            - '0.0'
            - '0.0'
            - '0.0'
            - '1.0'
            - 'false'
          fric_speed_pid_1:
            - '1.0'
            - '0.001'
            - '0.0'
            - '0.0'
            - '0.0'
            - '1.0'
            - 'false'
          fric_speed_pid_2:
            - '1.0'
            - '0.001'
            - '0.0'
            - '0.0'
            - '0.0'
            - '1.0'
            - 'false'
          fric_speed_pid_3:
            - '1.0'
            - '0.001'
            - '0.0'
            - '0.0'
            - '0.0'
            - '1.0'
            - 'false'
          push_motor_speed_pid:
            - '1.0'
            - '0.0008'
            - '0.0'
            - '0.0'
            - '0.0'
            - '1.0'
            - 'false'
          push_motor_angle_pid:
            - '1.0'
            - '1000.0'
            - '0.0'
            - '0.0'
            - '0.0'
            - '2000.0'
            - 'false'
          launcher_cmd_topic_name: '"launcher_cmd"'
          launcher_ref_topic_name: '"launcher_ref"'
          chassis_cmd_topic_name: '"chassis_cmd"'
          fire_notify_topic_name: '"fire_notify"'
```

所有依赖都是其他模块实例的 id，须在本实例之前列出：`motor_yaw`、`motor_pitch` 为
`QDU-Robomaster/DMMotor` 或 `QDU-Robomaster/RMMotor` 实例，四个摩擦轮与 `push_motor` 为
`QDU-Robomaster/RMMotor` 实例，`cmd` 为 `QDU-Robomaster/CMD` 实例。本例没有需要 BSP 用
`XR_REGISTER` 注册的对象。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/Dart`
（在 BSP 中）打印当前的构造函数。
