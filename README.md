# Dart

飞镖系统模块：由 yaw 云台、四个摩擦轮和推杆组成的飞镖架控制 / Dart system Module controlling a dart launcher built from a yaw gimbal, four friction wheels and a pusher

## 1. 模块作用 / Purpose

构造时，Dart 创建线程 `dartThread`（栈深 `param.task_stack_depth`，优先级 `LibXR::Thread::Priority::MEDIUM`）。线程每轮依次刷新电机反馈、更新云台、摩擦轮与推杆的控制输出并下发，然后休眠 2 ms。pitch 电机以 `MODE_CURRENT` 下发 0。

模式由 `Dart::DartMode` 给出：`RELAX`（0）、`YAW_COMMON`（1）、`YAW_SCAN`（2）、`GAME`（3）。`GetEvent()` 返回的 `LibXR::Event` 注册了这四个值，激活对应事件 ID 即切换模式，`SetMode(mode)` 与激活该事件等价。CMD 的 `CMD_EVENT_START_CTRL` 切换到 `RELAX`，`CMD_EVENT_LOST_CTRL` 切换到 `GAME`。切换到 `RELAX`、`YAW_COMMON`、`YAW_SCAN` 时，飞镖发射口状态复位为 `DEFAULT`；切换到 `GAME` 时，yaw 云台重新进入 `INITIALIZING`。

yaw 云台：yaw 输出轴角度由电机 `abs_angle` 的增量除以固定减速比 19.2032 累加得到，经角度环 `pid_yaw_angle` 与速度环 `pid_yaw_speed` 以 `MODE_CURRENT` 输出；`RELAX` 模式下 yaw 电机 `Relax()`。状态机如下：

- `INITIALIZING`：设定点每周期移动 2π/250，150 个周期后检测到 |扭矩| > 0.075 即认为到达机械限位，记为最小角，最大角为最小角加 65 rad，转入 `SCANNING`。
- `SCANNING`：以 8 rad/s 在最小角与最大角之间往返扫描。
- `NORMAL_CONTROL`：目标角为当前角加上上位机或遥控给出的 yaw 偏移，已标定时限制在最小角与最大角之间。收到新的非零上位机 yaw 时进入该状态；上位机数据超过 100 ms 未更新时，已标定则回到 `SCANNING`，未标定则回到 `INITIALIZING`。

摩擦轮：四个 `RMMotor` 做转速环（rpm），前两路目标为 `fric2_setpoint_speed`，后两路目标为 `fric1_setpoint_speed`，以 `MODE_CURRENT` 输出。停止和起转阶段 PID 输出限幅为 0.1，后右摩擦轮转速超过 `fric1_setpoint_speed` 后解除限幅并判定为就绪。发射模式为单发（`LaunchMode::SINGLE_SHOT`）：飞镖发射口状态为 `IS_OPENING` 或 `ON` 时启动摩擦轮，否则停止；该状态在 `GAME` 模式下来自裁判系统，在 `YAW_*` 模式下由遥控置位。

推杆：上电后先向负方向寻找限位（250 个周期后 |扭矩| > 0.02），推杆角度按 `push_motor_gear_ratio` 换算，最小位置为限位加 2 rad，最大位置为限位加 61 rad；角度环 `push_motor_angle_pid` 与速度环 `push_motor_speed_pid` 以 `MODE_CURRENT` 输出。摩擦轮就绪、发射口为 `ON` 且推杆空闲时，发射命令的上升沿使推杆推向最大位置；检测到发射后推杆停在当前位置，等待下一次上升沿。摩擦轮停止时推杆回到最小位置。

发射检测：摩擦轮就绪且推杆正在推出时，后左摩擦轮 |扭矩| > 0.1 视为一次发射，此后 100 ms 内 `launch_flag` 为 `true`。检测与发布只在 `GAME` 模式下执行。

遥控（`chassis_cmd`）：`YAW_COMMON` 与 `YAW_SCAN` 下 z > 0.7 时，发射口状态置为 `ON` 并产生发射命令；`YAW_COMMON` 下 x 直接作为 yaw 偏移。

Upon construction, Dart creates the thread `dartThread` (stack depth `param.task_stack_depth`, priority `LibXR::Thread::Priority::MEDIUM`). Each iteration refreshes the motor feedback, updates the control outputs of the gimbal, the friction wheels and the pusher, sends them, and then sleeps for 2 ms. The pitch motor receives 0 in `MODE_CURRENT`.

The modes are given by `Dart::DartMode`: `RELAX` (0), `YAW_COMMON` (1), `YAW_SCAN` (2) and `GAME` (3). `GetEvent()` returns a `LibXR::Event` on which these four values are registered; activating the corresponding event ID switches the mode, and `SetMode(mode)` is equivalent to activating that event. The CMD event `CMD_EVENT_START_CTRL` switches to `RELAX` and `CMD_EVENT_LOST_CTRL` switches to `GAME`. Switching to `RELAX`, `YAW_COMMON` or `YAW_SCAN` resets the dart gate status to `DEFAULT`; switching to `GAME` puts the yaw gimbal back into `INITIALIZING`.

Yaw gimbal: the yaw output shaft angle is accumulated from the increments of the motor `abs_angle` divided by the fixed reduction ratio 19.2032. The angle loop `pid_yaw_angle` and the speed loop `pid_yaw_speed` output in `MODE_CURRENT`; in `RELAX` mode the yaw motor is `Relax()`ed. The state machine is:

- `INITIALIZING`: the setpoint moves by 2π/250 per cycle; after 150 cycles, |torque| > 0.075 is taken as reaching the mechanical limit, which is recorded as the minimum angle. The maximum angle is the minimum angle plus 65 rad, and the state changes to `SCANNING`.
- `SCANNING`: sweeps back and forth between the minimum and maximum angle at 8 rad/s.
- `NORMAL_CONTROL`: the target angle is the current angle plus the yaw offset from the host or the remote controller, limited between the minimum and maximum angle once calibrated. A new non-zero host yaw enters this state; when the host data has not been updated for more than 100 ms, the state returns to `SCANNING` if calibrated and to `INITIALIZING` otherwise.

Friction wheels: four `RMMotor` run speed loops (rpm). The first two targets are `fric2_setpoint_speed` and the last two are `fric1_setpoint_speed`, output in `MODE_CURRENT`. While stopped and while spinning up, the PID output limit is 0.1; once the back-right wheel speed exceeds `fric1_setpoint_speed`, the limit is released and the wheels are ready. The launch mode is single shot (`LaunchMode::SINGLE_SHOT`): the friction wheels start when the dart gate status is `IS_OPENING` or `ON` and stop otherwise. In `GAME` mode the status comes from the referee system, and in the `YAW_*` modes it is set by the remote controller.

Pusher: after power-up it first searches for the limit in the negative direction (|torque| > 0.02 after 250 cycles). The pusher angle is converted with `push_motor_gear_ratio`; the minimum position is the limit plus 2 rad and the maximum position is the limit plus 61 rad. The angle loop `push_motor_angle_pid` and the speed loop `push_motor_speed_pid` output in `MODE_CURRENT`. When the friction wheels are ready, the gate status is `ON` and the pusher is idle, the rising edge of the fire command pushes the pusher toward the maximum position; after a launch is detected the pusher stops at the current position and waits for the next rising edge. When the friction wheels stop, the pusher returns to the minimum position.

Launch detection: while the friction wheels are ready and the pusher is pushing out, the back-left friction wheel |torque| > 0.1 counts as one launch, after which `launch_flag` is `true` for 100 ms. Detection and publishing run only in `GAME` mode.

Remote controller (`chassis_cmd`): in `YAW_COMMON` and `YAW_SCAN`, z > 0.7 sets the gate status to `ON` and produces a fire command; in `YAW_COMMON`, x is used directly as the yaw offset.

## 2. 构造接口 / Constructor

```cpp
Dart(Motor& motor_yaw,
     Motor& motor_pitch,
     RMMotor& motor_fric_front_left,
     RMMotor& motor_fric_front_right,
     RMMotor& motor_fric_back_left,
     RMMotor& motor_fric_back_right,
     RMMotor& push_motor,
     CMD& cmd,
     const Param& param = {...});  // 节选 / excerpt
```

依赖：

- `motor_yaw`、`motor_pitch`：`Motor`，yaw 与 pitch 电机。
- `motor_fric_front_left`、`motor_fric_front_right`、`motor_fric_back_left`、`motor_fric_back_right`：`RMMotor`，四个摩擦轮电机。
- `push_motor`：`RMMotor`，推杆电机。
- `cmd`：`CMD` 实例。

配置参数（`Param`；PID 为 `LibXR::PID<float>::Param`，字段为 `k, p, i, d, i_limit, out_limit, cycle`）：

- `task_stack_depth`：线程栈深，默认 4096。
- `pid_yaw_angle`：yaw 角度环，默认 `{.k = 1.0, .p = 900.0, .i = 0.0, .d = 0.0, .i_limit = 0.0, .out_limit = 1000.0, .cycle = false}`。
- `pid_yaw_speed`：yaw 速度环，默认 `{.k = 1.0, .p = 0.001, .i = 0.0, .d = 0.0, .i_limit = 0.0, .out_limit = 1.0, .cycle = false}`。
- `push_motor_gear_ratio`：推杆电机减速比，默认 36.0。
- `fric1_setpoint_speed`：后两路摩擦轮目标转速，单位 rpm，默认 4500.0，也是就绪判定阈值。
- `fric2_setpoint_speed`：前两路摩擦轮目标转速，单位 rpm，默认 4400.0。
- `fric_speed_pid_0` 至 `fric_speed_pid_3`：前左、前右、后左、后右摩擦轮的速度环，默认均为 `{.k = 1.0, .p = 0.001, .i = 0.0, .d = 0.0, .i_limit = 0.0, .out_limit = 1.0, .cycle = false}`。
- `push_motor_speed_pid`：推杆速度环，默认 `{.k = 1.0, .p = 0.0008, .i = 0.0, .d = 0.0, .i_limit = 0.0, .out_limit = 1.0, .cycle = false}`。
- `push_motor_angle_pid`：推杆角度环，默认 `{.k = 1.0, .p = 1000.0, .i = 0.0, .d = 0.0, .i_limit = 0.0, .out_limit = 2000.0, .cycle = false}`。
- `launcher_cmd_topic_name`：订阅的发射控制命令 Topic 名称，默认 `"launcher_cmd"`，与 CMD 的同名参数一致。
- `chassis_cmd_topic_name`：订阅的底盘命令 Topic 名称，默认 `"chassis_cmd"`，与 CMD 的同名参数一致。
- `launcher_ref_topic_name`：订阅的裁判系统发射数据 Topic 名称，默认 `"launcher_ref"`。
- `fire_notify_topic_name`：订阅的上位机开火通知 Topic 名称，默认 `"fire_notify"`，与 HostData 的 `host_fire_topic_name`（默认 `"host_fire_notify"`）取相同名称。

Dependencies:

- `motor_yaw`, `motor_pitch`: `Motor` objects for the yaw and pitch motors.
- `motor_fric_front_left`, `motor_fric_front_right`, `motor_fric_back_left`, `motor_fric_back_right`: `RMMotor` objects for the four friction wheel motors.
- `push_motor`: the `RMMotor` of the pusher.
- `cmd`: the `CMD` instance.

Configuration parameters (`Param`; the PIDs are `LibXR::PID<float>::Param` with fields `k, p, i, d, i_limit, out_limit, cycle`):

- `task_stack_depth`: thread stack depth, default 4096.
- `pid_yaw_angle`: yaw angle loop, default `{.k = 1.0, .p = 900.0, .i = 0.0, .d = 0.0, .i_limit = 0.0, .out_limit = 1000.0, .cycle = false}`.
- `pid_yaw_speed`: yaw speed loop, default `{.k = 1.0, .p = 0.001, .i = 0.0, .d = 0.0, .i_limit = 0.0, .out_limit = 1.0, .cycle = false}`.
- `push_motor_gear_ratio`: pusher motor reduction ratio, default 36.0.
- `fric1_setpoint_speed`: target speed of the back two friction wheels in rpm, default 4500.0; also the threshold for the ready decision.
- `fric2_setpoint_speed`: target speed of the front two friction wheels in rpm, default 4400.0.
- `fric_speed_pid_0` to `fric_speed_pid_3`: speed loops of the front-left, front-right, back-left and back-right friction wheels, all default to `{.k = 1.0, .p = 0.001, .i = 0.0, .d = 0.0, .i_limit = 0.0, .out_limit = 1.0, .cycle = false}`.
- `push_motor_speed_pid`: pusher speed loop, default `{.k = 1.0, .p = 0.0008, .i = 0.0, .d = 0.0, .i_limit = 0.0, .out_limit = 1.0, .cycle = false}`.
- `push_motor_angle_pid`: pusher angle loop, default `{.k = 1.0, .p = 1000.0, .i = 0.0, .d = 0.0, .i_limit = 0.0, .out_limit = 2000.0, .cycle = false}`.
- `launcher_cmd_topic_name`: name of the subscribed launcher command Topic, default `"launcher_cmd"`, matching the parameter of the same name of CMD.
- `chassis_cmd_topic_name`: name of the subscribed chassis command Topic, default `"chassis_cmd"`, matching the parameter of the same name of CMD.
- `launcher_ref_topic_name`: name of the subscribed referee launcher data Topic, default `"launcher_ref"`.
- `fire_notify_topic_name`: name of the subscribed host fire notification Topic, default `"fire_notify"`; it takes the same name as `host_fire_topic_name` of HostData (default `"host_fire_notify"`).

## 3. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `host_dart_gimbal_cmd` | 创建并订阅 | `Dart::DartGimbalCMD`（`yaw`） | 上位机给出的 yaw 偏移 |
| `param.fire_notify_topic_name`（默认 `fire_notify`） | 订阅 | `HostData::LauncherCMD`（`isfire`） | 上位机的发射命令 |
| `param.launcher_cmd_topic_name`（默认 `launcher_cmd`） | 订阅 | `CMD::LauncherCMD` | 发射控制命令 |
| `param.chassis_cmd_topic_name`（默认 `chassis_cmd`） | 订阅 | `CMD::ChassisCMD` | 遥控输入 |
| `param.launcher_ref_topic_name`（默认 `launcher_ref`） | 订阅（仅 `GAME`） | `Referee::LauncherPack` | 飞镖发射口状态 |
| `launch_flag` | 发布（仅 `GAME`） | `bool` | 发射检测标志 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `host_dart_gimbal_cmd` | Create and subscribe | `Dart::DartGimbalCMD` (`yaw`) | Yaw offset from the host |
| `param.fire_notify_topic_name` (default `fire_notify`) | Subscribe | `HostData::LauncherCMD` (`isfire`) | Fire command from the host |
| `param.launcher_cmd_topic_name` (default `launcher_cmd`) | Subscribe | `CMD::LauncherCMD` | Launcher command |
| `param.chassis_cmd_topic_name` (default `chassis_cmd`) | Subscribe | `CMD::ChassisCMD` | Remote controller input |
| `param.launcher_ref_topic_name` (default `launcher_ref`) | Subscribe (`GAME` only) | `Referee::LauncherPack` | Dart gate status |
| `launch_flag` | Publish (`GAME` only) | `bool` | Launch detection flag |

## 4. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/Dart` 写入的实例，依赖填写为其他模块实例的 id：`motor_yaw` 与 `motor_pitch` 取自 `QDU-Robomaster/DMMotor` 或 `QDU-Robomaster/RMMotor` 实例，四个摩擦轮与 `push_motor` 取自 `QDU-Robomaster/RMMotor` 实例，`cmd` 取自 `QDU-Robomaster/CMD` 实例，它们须在本实例之前列出。

An instance written by `xrobot instance add QDU-Robomaster/Dart`, with the dependencies set to the ids of other Module instances: `motor_yaw` and `motor_pitch` come from `QDU-Robomaster/DMMotor` or `QDU-Robomaster/RMMotor` instances, the four friction wheels and `push_motor` from `QDU-Robomaster/RMMotor` instances, and `cmd` from a `QDU-Robomaster/CMD` instance; they are listed before this instance.

```yaml
modules:
  - module: QDU-Robomaster/Dart
    id: Dart_0
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
          task_stack_depth: 4096
          pid_yaw_angle:
            k: 1.0
            p: 900.0
            i: 0.0
            d: 0.0
            i_limit: 0.0
            out_limit: 1000.0
            cycle: false
          pid_yaw_speed:
            k: 1.0
            p: 0.001
            i: 0.0
            d: 0.0
            i_limit: 0.0
            out_limit: 1.0
            cycle: false
          push_motor_gear_ratio: 36.0
          fric1_setpoint_speed: 4500.0
          fric2_setpoint_speed: 4400.0
          fric_speed_pid_0:
            k: 1.0
            p: 0.001
            i: 0.0
            d: 0.0
            i_limit: 0.0
            out_limit: 1.0
            cycle: false
          fric_speed_pid_1:
            k: 1.0
            p: 0.001
            i: 0.0
            d: 0.0
            i_limit: 0.0
            out_limit: 1.0
            cycle: false
          fric_speed_pid_2:
            k: 1.0
            p: 0.001
            i: 0.0
            d: 0.0
            i_limit: 0.0
            out_limit: 1.0
            cycle: false
          fric_speed_pid_3:
            k: 1.0
            p: 0.001
            i: 0.0
            d: 0.0
            i_limit: 0.0
            out_limit: 1.0
            cycle: false
          push_motor_speed_pid:
            k: 1.0
            p: 0.0008
            i: 0.0
            d: 0.0
            i_limit: 0.0
            out_limit: 1.0
            cycle: false
          push_motor_angle_pid:
            k: 1.0
            p: 1000.0
            i: 0.0
            d: 0.0
            i_limit: 0.0
            out_limit: 2000.0
            cycle: false
          launcher_cmd_topic_name: "launcher_cmd"
          launcher_ref_topic_name: "launcher_ref"
          chassis_cmd_topic_name: "chassis_cmd"
          fire_notify_topic_name: "fire_notify"
```

## 5. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/Motor`：yaw 与 pitch 电机的接口。
- `QDU-Robomaster/RMMotor`：摩擦轮与推杆电机。
- `QDU-Robomaster/CMD`：CMD 事件与 `ChassisCMD` 类型。
- `QDU-Robomaster/Referee`：`LauncherPack` 类型。
- `QDU-Robomaster/HostData`：`LauncherCMD` 类型；`fire_notify_topic_name` 与 HostData 的 `host_fire_topic_name` 同名时，该 Topic 由 `HostData` 创建。
- LibXR。

硬件：yaw 与 pitch 两个电机、四个摩擦轮电机和一个推杆电机，均通过 `Motor` 或 `RMMotor` 实例接入；yaw 云台与推杆的限位通过堵转扭矩标定。

Dependencies:

- `QDU-Robomaster/Motor`: interface of the yaw and pitch motors.
- `QDU-Robomaster/RMMotor`: friction wheel and pusher motors.
- `QDU-Robomaster/CMD`: CMD events and the `ChassisCMD` type.
- `QDU-Robomaster/Referee`: the `LauncherPack` type.
- `QDU-Robomaster/HostData`: the `LauncherCMD` type; when `fire_notify_topic_name` equals `host_fire_topic_name` of HostData, that Topic is created by `HostData`.
- LibXR.

Hardware: the yaw and pitch motors, four friction wheel motors and one pusher motor, all attached through `Motor` or `RMMotor` instances; the limits of the yaw gimbal and the pusher are calibrated from the stall torque.
