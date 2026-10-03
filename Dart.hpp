#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 飞镖系统模块：由 yaw 云台、四个摩擦轮和推杆组成的飞镖架控制 / Dart system Module controlling a dart launcher built from a yaw gimbal, four friction wheels and a pusher
depends:
- id: QDU-Robomaster/CMD
  ref: same-or-dev
- id: QDU-Robomaster/RMMotor
  ref: same-or-dev
- id: QDU-Robomaster/Referee
  ref: same-or-dev
- id: QDU-Robomaster/Motor
  ref: same-or-dev
- id: QDU-Robomaster/HostData
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cstdint>

#include "CMD.hpp"
#include "HostData.hpp"
#include "Motor.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "gpio.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "uart.hpp"

/**
 * @brief 飞镖系统模块：由 yaw 云台、四个摩擦轮和推杆组成的飞镖架控制。
 *        Dart system Module controlling a dart launcher built from a yaw gimbal, four
 *        friction wheels and a pusher.
 */
class Dart
{
 public:
  /**
   * @brief 飞镖模式，数值同时是 `GetEvent()` 上注册的事件 ID。
   *        Dart modes; the values are also the event IDs registered on `GetEvent()`.
   */
  enum class DartMode : uint8_t
  {
    RELAX = 0,       ///< 放松：yaw 电机 `Relax()`，发射口状态复位
                     ///< Relax: yaw motor `Relax()`ed, gate status reset
    YAW_COMMON = 1,  ///< 遥控 yaw 偏移控制
                     ///< Remote-controlled yaw offset
    YAW_SCAN = 2,    ///< 遥控模式下的 yaw 扫描
                     ///< Yaw scanning under remote control
    GAME = 3,        ///< 比赛：发射口状态来自裁判系统，yaw 重新初始化
                     ///< Game: gate status from the referee system, yaw re-initialized
  };

  /**
   * @brief 飞镖发射口状态，对应裁判系统发射口数据中的 `opening_status`。
   *        Dart gate status, matching `opening_status` of the referee launcher data.
   */
  enum class OPENING_STATUS : uint8_t
  {
    ON = 0,          ///< 已打开 Opened
    CLOSE = 1,       ///< 已关闭 Closed
    IS_OPENING = 2,  ///< 正在打开 Opening
    DEFAULT = 3,     ///< 默认状态 Default
  };

  /**
   * @brief 发射模式。
   *        Launch mode.
   */
  enum class LaunchMode : uint8_t
  {
    SINGLE_SHOT = 0,  ///< 单发模式 Single shot
    FULL_FIRE = 1     ///< 连发模式 Full fire
  };

  /**
   * @brief 推杆状态。
   *        Pusher states.
   */
  enum class PushState : uint8_t
  {
    IDLE,            ///< 空闲，位于最小位置 Idle at the minimum position
    MOVING_TO_MAX,   ///< 向最大位置移动 Moving to the maximum position
    AT_MAX_WAITING,  ///< 在最大位置等待（仅单发模式）
                     ///< Waiting at the maximum position (single shot only)
    MOVING_TO_MIN,   ///< 向最小位置复位 Returning to the minimum position
    STOP_MOVING,     ///< 停在当前位置，等待下一次发射命令
                     ///< Stopped at the current position, waiting for the next fire
                     ///< command
  };

  /**
   * @brief 上位机给出的 yaw 偏移，通过 Topic `host_dart_gimbal_cmd` 传递。
   *        Yaw offset from the host, passed through the Topic `host_dart_gimbal_cmd`.
   */
  struct DartGimbalCMD
  {
    float yaw;  ///< yaw 偏移 Yaw offset
  };

  /**
   * @brief yaw 云台事件。
   *        Yaw gimbal events.
   */
  enum class DartGimbalEvent : uint8_t
  {
    SET_MODE_RELAX = 0,   ///< 放松 Relax
    SET_MODE_COMMON = 1,  ///< 常规控制 Normal control
  };

  /**
   * @brief 摩擦轮事件。
   *        Friction wheel events.
   */
  enum class DartEvent : uint8_t
  {
    SET_MODE_FRIC_START,  ///< 启动摩擦轮 Start the friction wheels
    SET_MODE_FRIC_STOP,   ///< 停止摩擦轮 Stop the friction wheels
  };

  /**
   * @brief 摩擦轮状态。
   *        Friction wheel states.
   */
  enum class DartLauncherMode : uint8_t
  {
    FRIC_START,  ///< 摩擦轮起转 Friction wheels spinning up
    FRIC_STOP,   ///< 摩擦轮停止 Friction wheels stopped
  };

  /**
   * @brief yaw 电机状态机状态。
   *        States of the yaw motor state machine.
   */
  enum class YawMotorState : uint8_t
  {
    INITIALIZING,   ///< 初始化：向负方向移动寻找极限位置
                    ///< Initializing: move in the negative direction to find the limit
    SCANNING,       ///< 扫描：在最大角与最小角之间往返
                    ///< Scanning: sweep between the maximum and minimum angle
    NORMAL_CONTROL  ///< 正常控制：按上位机或遥控的 yaw 偏移控制
                    ///< Normal control: follow the yaw offset from the host or remote
  };

  /**
   * @brief 飞镖系统配置参数。
   *        Dart system configuration parameters.
   */
  struct Param
  {
    uint32_t task_stack_depth;  ///< 线程栈深
                                ///< Thread stack depth
    LibXR::PID<float>::Param pid_yaw_angle;  ///< yaw 角度环 PID
                                             ///< Yaw angle-loop PID
    LibXR::PID<float>::Param pid_yaw_speed;  ///< yaw 速度环 PID
                                             ///< Yaw speed-loop PID
    float push_motor_gear_ratio;  ///< 推杆电机减速比
                                  ///< Pusher motor reduction ratio
    float fric1_setpoint_speed;  ///< 后两路摩擦轮目标转速 (rpm)，也是就绪判定阈值
                                 ///< Target speed of the back two friction wheels (rpm),
                                 ///< also the ready threshold
    float fric2_setpoint_speed;  ///< 前两路摩擦轮目标转速 (rpm)
                                 ///< Target speed of the front two friction wheels (rpm)
    LibXR::PID<float>::Param fric_speed_pid_0;  ///< 前左摩擦轮速度环 PID
                                                ///< Front-left wheel speed-loop PID
    LibXR::PID<float>::Param fric_speed_pid_1;  ///< 前右摩擦轮速度环 PID
                                                ///< Front-right wheel speed-loop PID
    LibXR::PID<float>::Param fric_speed_pid_2;  ///< 后左摩擦轮速度环 PID
                                                ///< Back-left wheel speed-loop PID
    LibXR::PID<float>::Param fric_speed_pid_3;  ///< 后右摩擦轮速度环 PID
                                                ///< Back-right wheel speed-loop PID
    LibXR::PID<float>::Param push_motor_speed_pid;  ///< 推杆速度环 PID
                                                    ///< Pusher speed-loop PID
    LibXR::PID<float>::Param push_motor_angle_pid;  ///< 推杆角度环 PID
                                                    ///< Pusher angle-loop PID
    const char* launcher_cmd_topic_name;  ///< 订阅的发射控制命令 Topic 名称
                                          ///< Name of the subscribed launcher command
                                          ///< Topic
    const char* launcher_ref_topic_name;  ///< 订阅的裁判系统发射数据 Topic 名称
                                          ///< Name of the subscribed referee launcher
                                          ///< Topic
    const char* chassis_cmd_topic_name;   ///< 订阅的底盘控制命令 Topic 名称
                                          ///< Name of the subscribed chassis command
                                          ///< Topic
    const char* fire_notify_topic_name;   ///< 订阅的上位机开火通知 Topic 名称
                                          ///< Name of the subscribed host fire
                                          ///< notification Topic
  };

  /**
   * @brief 构造 Dart，创建控制线程并注册模式事件与 CMD 事件。
   *        Construct Dart, create the control thread and register the mode events and
   *        the CMD events.
   *
   * @param motor_yaw yaw 电机。
   *                  Yaw motor.
   * @param motor_pitch pitch 电机。
   *                    Pitch motor.
   * @param motor_fric_front_left 前左摩擦轮电机。
   *                              Front-left friction wheel motor.
   * @param motor_fric_front_right 前右摩擦轮电机。
   *                               Front-right friction wheel motor.
   * @param motor_fric_back_left 后左摩擦轮电机。
   *                             Back-left friction wheel motor.
   * @param motor_fric_back_right 后右摩擦轮电机。
   *                              Back-right friction wheel motor.
   * @param push_motor 推杆电机。
   *                   Pusher motor.
   * @param cmd CMD 实例。
   *            CMD instance.
   * @param param 配置参数。
   *              Configuration parameters.
   */
  Dart(
      Motor& motor_yaw,
      Motor& motor_pitch,
      RMMotor& motor_fric_front_left,
      RMMotor& motor_fric_front_right,
      RMMotor& motor_fric_back_left,
      RMMotor& motor_fric_back_right,
      RMMotor& push_motor,
      CMD& cmd,
      const Param& param = {.task_stack_depth = 4096, .pid_yaw_angle = {1.0, 900.0, 0.0, 0.0, 0.0, 1000.0, false}, .pid_yaw_speed = {1.0, 0.001, 0.0, 0.0, 0.0, 1.0, false}, .push_motor_gear_ratio = 36.0, .fric1_setpoint_speed = 4500.0, .fric2_setpoint_speed = 4400.0, .fric_speed_pid_0 = {1.0, 0.001, 0.0, 0.0, 0.0, 1.0, false}, .fric_speed_pid_1 = {1.0, 0.001, 0.0, 0.0, 0.0, 1.0, false}, .fric_speed_pid_2 = {1.0, 0.001, 0.0, 0.0, 0.0, 1.0, false}, .fric_speed_pid_3 = {1.0, 0.001, 0.0, 0.0, 0.0, 1.0, false}, .push_motor_speed_pid = {1.0, 0.0008, 0.0, 0.0, 0.0, 1.0, false}, .push_motor_angle_pid = {1.0, 1000.0, 0.0, 0.0, 0.0, 2000.0, false}, .launcher_cmd_topic_name = "launcher_cmd", .launcher_ref_topic_name = "launcher_ref", .chassis_cmd_topic_name = "chassis_cmd", .fire_notify_topic_name = "fire_notify"})
      : pid_yaw_angle_(param.pid_yaw_angle),
        pid_yaw_speed_(param.pid_yaw_speed),
        motor_yaw_(&motor_yaw),
        motor_pitch_(&motor_pitch),
        motor_fric_front_left_(&motor_fric_front_left),
        motor_fric_front_right_(&motor_fric_front_right),
        motor_fric_back_left_(&motor_fric_back_left),
        motor_fric_back_right_(&motor_fric_back_right),
        push_motor_(&push_motor),
        push_motor_gear_ratio_(param.push_motor_gear_ratio),
        fric1_setpoint_speed_(param.fric1_setpoint_speed),
        fric2_setpoint_speed_(param.fric2_setpoint_speed),
        fric_speed_pid_{param.fric_speed_pid_0, param.fric_speed_pid_1, param.fric_speed_pid_2,
                        param.fric_speed_pid_3},
        push_motor_speed_pid_(param.push_motor_speed_pid),
        push_motor_angle_pid_(param.push_motor_angle_pid),
        cmd_(&cmd)
  {
    launcher_cmd_topic_name_ = param.launcher_cmd_topic_name;
    launcher_ref_topic_name_ = param.launcher_ref_topic_name;
    chassis_cmd_topic_name_ = param.chassis_cmd_topic_name;
    fire_notify_topic_name_ = param.fire_notify_topic_name;
    ref_data_.dc.opening_status = 3;

    last_online_time_ = LibXR::Timebase::GetMicroseconds();
    thread_.Create(this, ThreadFunction, "dartThread", param.task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);

    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Dart* dart, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          dart->EventHandler(static_cast<DartMode>(DartMode::RELAX));
          dart->cnt_start++;
        },
        this);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Dart* dart, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          dart->EventHandler(static_cast<DartMode>(DartMode::GAME));
          dart->cnt_lost++;
        },
        this);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Dart* dart, uint32_t event_id)
        {
          UNUSED(in_isr);
          dart->EventHandler(static_cast<DartMode>(event_id));
        },
        this);
    dart_event_.Register(static_cast<uint32_t>(DartMode::RELAX), callback);
    dart_event_.Register(static_cast<uint32_t>(DartMode::YAW_COMMON), callback);
    dart_event_.Register(static_cast<uint32_t>(DartMode::YAW_SCAN), callback);
    dart_event_.Register(static_cast<uint32_t>(DartMode::GAME), callback);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);
  }

  /**
   * @brief 控制线程函数：订阅 Topic，每 2 ms 执行一轮更新与控制。
   *        Control thread function that subscribes to the Topics and runs one update and
   *        control iteration every 2 ms.
   *
   * @param dart Dart 实例指针。
   *             Pointer to the Dart instance.
   */
  static void ThreadFunction(Dart* dart)
  {
    LibXR::Topic::ASyncSubscriber<DartGimbalCMD> dart_gimbal_suber(
        "host_dart_gimbal_cmd");
    LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> launch_notify_suber(
        dart->launcher_cmd_topic_name_);
    LibXR::Topic::ASyncSubscriber<Referee::LauncherPack> launcher_ref(
        dart->launcher_ref_topic_name_);
    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber(dart->chassis_cmd_topic_name_);
    LibXR::Topic::ASyncSubscriber<HostData::LauncherCMD> fire_notify_suber(
        dart->fire_notify_topic_name_);
    dart_gimbal_suber.StartWaiting();
    launch_notify_suber.StartWaiting();
    launcher_ref.StartWaiting();
    cmd_suber.StartWaiting();
    fire_notify_suber.StartWaiting();
    while (1)
    {
      if (cmd_suber.Available())
      {
        dart->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }
      if (dart_gimbal_suber.Available())
      {
        float new_yaw = dart_gimbal_suber.GetData().yaw;
        // 检查是否是不同的数据
        if (std::abs(new_yaw - dart->dart_gimbal_cmd_.yaw) > 1e-6f && new_yaw != 0.0f)
        {
          dart->dart_gimbal_cmd_.yaw = new_yaw;
          dart->yaw_motor_state_ = YawMotorState::NORMAL_CONTROL;
          dart->last_gimbal_data_time_ = LibXR::Timebase::GetMilliseconds();
        }

        dart_gimbal_suber.StartWaiting();
      }

      auto now_time = LibXR::Timebase::GetMilliseconds();
      if (dart->yaw_motor_state_ == YawMotorState::NORMAL_CONTROL &&
          std::abs(dart->dart_gimbal_cmd_.yaw) > 1e-6f &&  // 数据非0
          (now_time - dart->last_gimbal_data_time_).ToMillisecond() > 100)
      {
        // 只有在从未初始化过时才重置限位值
        if (dart->min_yaw_motor_angle_ == 0.0f || dart->max_yaw_motor_angle_ == 0.0f)
        {
          dart->yaw_motor_state_ = YawMotorState::INITIALIZING;
          dart->delay_time_gimbal_ = 0;
        }
        else
        {
          // 已经有有效的限位值，直接进入扫描模式
          dart->yaw_motor_state_ = YawMotorState::SCANNING;
          dart->yaw_motor_setpoint_angle_ = dart->min_yaw_motor_angle_;
          dart->scan_direction_ = true;
        }
      }
      if (dart->mode_ == DartMode::GAME)
      {
        if (launcher_ref.Available())
        {
          dart->ref_data_.dc = launcher_ref.GetData().dc;
          launcher_ref.StartWaiting();
        }
      }
      if (fire_notify_suber.Available())
      {
        dart->fire_cmd_ = fire_notify_suber.GetData().isfire;
        fire_notify_suber.StartWaiting();
      }

      dart->UpdateYaw();
      dart->UpdatePitch();
      dart->UpdateFric();
      dart->UpdatePushMotor();
      dart->DR16CONTROL();
      if (dart->mode_ == DartMode::GAME)
      {
        dart->DetectLaunch();
      }
      dart->ControlYaw();
      dart->ControlPitch();
      dart->ControlFric();
      dart->ControlPushMotor();

      LibXR::Thread::Sleep(2);
    }
  }

  // === 云台 ===
  /**
   * @brief 更新 yaw 电机反馈，并按增量与减速比累加 yaw 输出轴角度。
   *        Update the yaw motor feedback and accumulate the yaw output shaft angle from
   *        the increments and the reduction ratio.
   */
  void UpdateYaw()
  {
    auto now = LibXR::Timebase::GetMicroseconds();
    dt_gimbal_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    const float LAST_YAW_MOTOR_ANGLE =
        LibXR::CycleValue<float>(motor_yaw_feedback_.abs_angle);
    motor_yaw_->Update();
    motor_yaw_feedback_ = motor_yaw_->GetFeedback();
    const float DELTA_YAW_MOTOR_ANGLE =
        LibXR::CycleValue<float>(motor_yaw_feedback_.abs_angle) - LAST_YAW_MOTOR_ANGLE;
    this->yaw_motor_angle_ += DELTA_YAW_MOTOR_ANGLE / YAW_MOTOR_GEAR_RATIO;
  }

  /**
   * @brief 更新 pitch 电机反馈。
   *        Update the pitch motor feedback.
   */
  void UpdatePitch()
  {
    motor_pitch_->Update();
    motor_pitch_feedback_ = motor_pitch_->GetFeedback();
  }

  /**
   * @brief 按 yaw 状态机计算 yaw 输出并以 `MODE_CURRENT` 下发；放松时调用 `Relax()`。
   *        Compute the yaw output from the yaw state machine and send it in
   *        `MODE_CURRENT`; call `Relax()` when relaxed.
   */
  void ControlYaw()
  {
    if (current_mode_ == DartGimbalEvent::SET_MODE_RELAX)
    {
      motor_yaw_->Relax();
      return;
    }
    if (mode_ == DartMode::RELAX)
    {
      motor_yaw_->Relax();
      return;
    }

    float out_yaw = 0.0f;

    switch (yaw_motor_state_)
    {
      case YawMotorState::INITIALIZING:
      {
        // 向负方向移动寻找极限位置
        yaw_motor_setpoint_angle_ -= LibXR::TWO_PI / 250.0f;
        delay_time_gimbal_++;

        // 检测扭矩是否变大（超过阈值），表示到达机械极限
        if (delay_time_gimbal_ > 150 && std::abs(motor_yaw_feedback_.torque) > 0.075f)
        {
          min_yaw_motor_angle_ = yaw_motor_angle_;
          max_yaw_motor_angle_ = min_yaw_motor_angle_ + 65.0f;
          yaw_motor_setpoint_angle_ = max_yaw_motor_angle_;
          scan_direction_ = true;  // 开始向max方向扫描
          yaw_motor_state_ = YawMotorState::SCANNING;
        }
        // 使用角度PID控制到设定点
        float target_yaw_speed = pid_yaw_angle_.Calculate(yaw_motor_setpoint_angle_,
                                                          yaw_motor_angle_, dt_gimbal_);
        out_yaw = pid_yaw_speed_.Calculate(target_yaw_speed, motor_yaw_feedback_.velocity,
                                           dt_gimbal_);
        break;
      }

      case YawMotorState::SCANNING:
      {
        // 在max和min之间扫描
        if (scan_direction_)
        {
          // 向max方向扫描
          yaw_motor_setpoint_angle_ += SCAN_SPEED * dt_gimbal_;
          if (yaw_motor_angle_ >= max_yaw_motor_angle_ - 1.0f)
          {
            scan_direction_ = false;                           // 切换方向
            yaw_motor_setpoint_angle_ = max_yaw_motor_angle_;  // 限制在边界
          }
        }
        else
        {
          // 向min方向扫描
          yaw_motor_setpoint_angle_ -= SCAN_SPEED * dt_gimbal_;
          if (yaw_motor_angle_ <= min_yaw_motor_angle_ + 2.0f)
          {
            scan_direction_ = true;                            // 切换方向
            yaw_motor_setpoint_angle_ = min_yaw_motor_angle_;  // 限制在边界
          }
        }
        // 确保设定点在范围内
        yaw_motor_setpoint_angle_ = std::clamp(
            yaw_motor_setpoint_angle_, min_yaw_motor_angle_, max_yaw_motor_angle_);
        // 使用角度PID控制到设定点
        float target_yaw_speed = pid_yaw_angle_.Calculate(yaw_motor_setpoint_angle_,
                                                          yaw_motor_angle_, dt_gimbal_);
        out_yaw = pid_yaw_speed_.Calculate(target_yaw_speed, motor_yaw_feedback_.velocity,
                                           dt_gimbal_);
        break;
      }

      case YawMotorState::NORMAL_CONTROL:
      {
        // 正常控制模式，使用上位机指令，但限制在min和max之间
        float target_yaw_angle = dart_gimbal_cmd_.yaw + yaw_motor_angle_;
        // 如果已经完成初始化，限制目标角度在min和max之间
        if (min_yaw_motor_angle_ != 0.0f || max_yaw_motor_angle_ != 0.0f)
        {
          // 限制绝对目标位置在范围内
          target_yaw_angle =
              std::clamp(target_yaw_angle, min_yaw_motor_angle_, max_yaw_motor_angle_);
        }
        Solve(out_yaw, target_yaw_angle, dt_gimbal_);
        break;
      }
    }

    auto yaw_motor_cmd =
        Motor::MotorCmd({.mode = Motor::ControlMode::MODE_CURRENT, .velocity = out_yaw});

    auto motor_control = [&](Motor* motor, const Motor::Feedback& fb,
                             const Motor::MotorCmd& cmd) { motor->Control(cmd); };

    motor_control(motor_yaw_, motor_yaw_feedback_, yaw_motor_cmd);
  }

  /**
   * @brief 向 pitch 电机以 `MODE_CURRENT` 下发 0。
   *        Send 0 to the pitch motor in `MODE_CURRENT`.
   */
  void ControlPitch()
  {
    motor_pitch_->Control(
        Motor::MotorCmd({.mode = Motor::ControlMode::MODE_CURRENT, .velocity = 0.0f}));
  }

  /**
   * @brief 由目标 yaw 角计算角度环与速度环的串级输出。
   *        Compute the cascaded angle-loop and speed-loop output for a target yaw angle.
   *
   * @param yaw_output 输出：yaw 电机的控制量。
   *                   Output: control value of the yaw motor.
   * @param target_yaw_angle 目标 yaw 角。
   *                         Target yaw angle.
   * @param dt_ 控制周期，单位 s。
   *            Control period in s.
   */
  void Solve(float& yaw_output, float target_yaw_angle, float dt_)
  {
    float yaw_error = target_yaw_angle - yaw_motor_angle_;
    float target_yaw_speed = pid_yaw_angle_.Calculate(yaw_error, 0.0f, dt_);
    float fb_yaw =
        pid_yaw_speed_.Calculate(target_yaw_speed, motor_yaw_feedback_.velocity, dt_);
    yaw_output = fb_yaw;
  }

  // === 发射机构 ===
  /**
   * @brief 更新四个摩擦轮电机的反馈。
   *        Update the feedback of the four friction wheel motors.
   */
  void UpdateFric()
  {
    auto now = LibXR::Timebase::GetMilliseconds();
    dt_launcher_ = (now - last_online_time_launcher_).ToSecondf();
    last_online_time_launcher_ = now;

    motor_fric_front_right_->Update();
    motor_fric_front_left_->Update();
    motor_fric_back_left_->Update();
    motor_fric_back_right_->Update();

    param_motor_fric_front_left_ = motor_fric_front_left_->GetFeedback();
    param_motor_fric_front_right_ = motor_fric_front_right_->GetFeedback();
    param_motor_fric_back_left_ = motor_fric_back_left_->GetFeedback();
    param_motor_fric_back_right_ = motor_fric_back_right_->GetFeedback();
  }

  /**
   * @brief 更新推杆电机反馈，并按增量与减速比累加推杆角度。
   *        Update the pusher motor feedback and accumulate the pusher angle from the
   *        increments and the reduction ratio.
   */
  void UpdatePushMotor()
  {
    const float LAST_PUSH_MOTOR_ANGLE =
        LibXR::CycleValue<float>(param_push_motor_.abs_angle);
    push_motor_->Update();
    param_push_motor_ = push_motor_->GetFeedback();
    const float DELTA_PUSH_MOTOR_ANGLE =
        LibXR::CycleValue<float>(param_push_motor_.abs_angle) - LAST_PUSH_MOTOR_ANGLE;
    this->push_motor_angle_ += DELTA_PUSH_MOTOR_ANGLE / push_motor_gear_ratio_;
  }

  /**
   * @brief 检测发射并发布 `launch_flag`：发射后 100 ms 内为 true。
   *        Detect a launch and publish `launch_flag`, which is true for 100 ms after a
   *        launch.
   */
  void DetectLaunch()
  {
    // 检测是否发生发射
    bool should_mark_launch = false;
    auto current_time = LibXR::Timebase::GetMilliseconds();

    if (fric_ready_ && (push_state_ == PushState::AT_MAX_WAITING ||
                        push_state_ == PushState::MOVING_TO_MAX))
    {
      // 检测扭矩变大
      if (std::abs(motor_fric_back_left_->GetFeedback().torque) > 0.1f)
      {
        // 如果还没有开始100ms计时，则开始
        if (!launch_detected_)
        {
          launch_detected_ = true;
          launch_detect_timestamp_ = current_time;
        }
      }
    }

    // 处理100ms信号输出
    if (launch_detected_)
    {
      // 如果在100ms内，输出true
      if (current_time - launch_detect_timestamp_ < 100)
      {
        should_mark_launch = true;
      }
      else
      {
        // 100ms已过，重置状态，允许下次检测
        launch_detected_ = false;
        should_mark_launch = false;
      }
    }

    marked_launch_ = should_mark_launch;
    launcher_topic_.Publish(marked_launch_);
  }
  /**
   * @brief 按发射口状态启停摩擦轮，计算转速环输出并下发。
   *        Start or stop the friction wheels from the gate status, compute the speed-loop
   *        outputs and send them.
   */
  void ControlFric()
  {
    // 只在推杆电机复位完成时停止摩擦轮（在ControlPushMotor中处理）
    if (launch_mode_ == LaunchMode::SINGLE_SHOT)
    {
      if (ref_data_.dc.opening_status ==
              static_cast<uint8_t>(OPENING_STATUS::IS_OPENING) ||
          ref_data_.dc.opening_status == static_cast<uint8_t>(OPENING_STATUS::ON))
      {
        fric_mode_ = DartLauncherMode::FRIC_START;
      }
      else
      {
        fric_mode_ = DartLauncherMode::FRIC_STOP;
      }
    }
    else if (launch_mode_ == LaunchMode::FULL_FIRE)
    {
      // FULL_FIRE模式下直接根据fire_cmd控制
      if (cmd_data_.x > 0.5f)
      {
        fric_mode_ = DartLauncherMode::FRIC_START;
      }
    }

    switch (fric_mode_)
    {
      case DartLauncherMode::FRIC_STOP:
        fric_target_speed_[0] = 0;
        fric_target_speed_[1] = 0;
        fric_target_speed_[2] = 0;
        fric_target_speed_[3] = 0;
        for (auto& i : fric_speed_pid_)
        {
          i.SetOutLimit(0.1f);
          fric_ready_ = false;
        }
        break;
      case DartLauncherMode::FRIC_START:
        fric_target_speed_[0] = fric2_setpoint_speed_;
        fric_target_speed_[1] = fric2_setpoint_speed_;
        fric_target_speed_[2] = fric1_setpoint_speed_;
        fric_target_speed_[3] = fric1_setpoint_speed_;
        if (param_motor_fric_back_right_.velocity > fric1_setpoint_speed_)
        {
          for (auto& i : fric_speed_pid_)
          {
            i.SetOutLimit(0.0f);
            fric_ready_ = true;
          }
        }
        break;
    }

    fric_output_[0] = fric_speed_pid_[0].Calculate(
        fric_target_speed_[0], param_motor_fric_front_left_.velocity, dt_launcher_);
    fric_output_[1] = fric_speed_pid_[1].Calculate(
        fric_target_speed_[1], param_motor_fric_front_right_.velocity, dt_launcher_);
    fric_output_[2] = fric_speed_pid_[2].Calculate(
        fric_target_speed_[2], param_motor_fric_back_left_.velocity, dt_launcher_);
    fric_output_[3] = fric_speed_pid_[3].Calculate(
        fric_target_speed_[3], param_motor_fric_back_right_.velocity, dt_launcher_);

    cmd_fric_front_left_.velocity = fric_output_[0];
    cmd_fric_front_right_.velocity = fric_output_[1];
    cmd_fric_back_left_.velocity = fric_output_[2];
    cmd_fric_back_right_.velocity = fric_output_[3];

    motor_fric_front_left_->Control(cmd_fric_front_left_);
    motor_fric_front_right_->Control(cmd_fric_front_right_);
    motor_fric_back_left_->Control(cmd_fric_back_left_);
    motor_fric_back_right_->Control(cmd_fric_back_right_);
  }

  /**
   * @brief 推杆初始化与发射状态机，计算角度环与速度环输出并下发。
   *        Pusher initialization and launch state machine; compute the angle-loop and
   *        speed-loop outputs and send them.
   */
  void ControlPushMotor()
  {
    if (!push_motor_init_)
    {
      push_motor_setpoint_angle_ -= LibXR::TWO_PI / 250.0f;
      push_motor_angle_pid_.SetOutLimit(2000.0f);
      delay_time_launcher_++;
      if (delay_time_launcher_ > 250)
      {
        if (std::abs(param_push_motor_.torque) > 0.02)
        {
          push_motor_init_ = true;
          min_push_motor_angle_ = push_motor_angle_ + 2.0f;
          max_push_motor_angle_ = push_motor_angle_ + 61.0f;
          push_motor_setpoint_angle_ = min_push_motor_angle_;
          // 初始化完成后保持停止状态，等待发射命令
          fric_mode_ = DartLauncherMode::FRIC_STOP;
        }
      }
    }
    else
    {
      // 推杆已初始化，处理发射逻辑

      // 如果模式是FRIC_START，启动摩擦轮并等待准备就绪
      if (fric_mode_ == DartLauncherMode::FRIC_START)
      {
        // 检查摩擦轮是否准备好
        if (!fric_ready_ &&
            param_motor_fric_back_right_.velocity > fric1_setpoint_speed_ &&
            param_motor_fric_back_left_.velocity > fric1_setpoint_speed_)
        {
          fric_ready_ = true;
          for (auto& i : fric_speed_pid_)
          {
            i.SetOutLimit(0.0f);
          }
        }

        // 如果摩擦轮已准备好且有发射命令，处理状态机
        if (fric_ready_)
        {
          switch (launch_mode_)
          {
            case LaunchMode::SINGLE_SHOT:
            {
              // 单发模式 - 只有在fire_cmd从0变为1时才触发发射
              static bool last_fire_cmd = false;

              // 检测fire_cmd上升沿
              if (!last_fire_cmd && fire_cmd_)
              {
                if (ref_data_.dc.opening_status ==
                    static_cast<uint8_t>(OPENING_STATUS::ON))
                {
                  // 触发发射，只在IDLE状态下才开始新的发射循环
                  if (push_state_ == PushState::IDLE)
                  {
                    push_state_ = PushState::MOVING_TO_MAX;
                  }
                }
              }
              last_fire_cmd = fire_cmd_;

              switch (push_state_)
              {
                case PushState::MOVING_TO_MAX:
                  push_motor_setpoint_angle_ = max_push_motor_angle_;
                  if (push_motor_angle_ > max_push_motor_angle_ - 2.0f)
                  {
                    push_state_ = PushState::AT_MAX_WAITING;
                  }
                  if (launch_detected_)
                  {
                    push_state_ = PushState::STOP_MOVING;
                    // 在 STOP_MOVING 状态中重置 fire_cmd，等待新的上升沿
                    fire_cmd_ = false;
                    last_fire_cmd = false;  // 重置静态变量以准备下一次检测
                    cnt++;
                  }
                  break;

                case PushState::AT_MAX_WAITING:
                  push_motor_setpoint_angle_ = max_push_motor_angle_;
                  // 如果检测到发射，停止
                  if (launch_detected_)
                  {
                    push_state_ = PushState::STOP_MOVING;
                    // 在 STOP_MOVING 状态中重置 fire_cmd，等待新的上升沿
                    fire_cmd_ = false;
                    last_fire_cmd = false;  // 重置静态变量以准备下一次检测
                    cnt++;
                  }
                  break;

                case PushState::MOVING_TO_MIN:
                  push_motor_setpoint_angle_ = min_push_motor_angle_;
                  if (push_motor_angle_ < min_push_motor_angle_ + 2.0f)
                  {
                    push_state_ = PushState::IDLE;
                    // 推杆复位完成，停止摩擦轮
                    fric_mode_ = DartLauncherMode::FRIC_STOP;
                    fire_cmd_ = false;  // 重置发射命令，等待下次触发
                    is_firing_ = false;
                  }
                  break;
                case PushState::STOP_MOVING:
                  // 停止在当前位置
                  push_motor_setpoint_angle_ = push_motor_angle_;
                  // 在STOP_MOVING状态中，等待fire_cmd的上升沿以重置状态机
                  static bool last_fire_cmd_in_stop = false;
                  if (!last_fire_cmd_in_stop && fire_cmd_)
                  {
                    // 检测到上升沿，直接进入 MOVING_TO_MAX 开始新发射
                    push_state_ = PushState::MOVING_TO_MAX;
                    fire_cmd_ = false;  // 可选：消费命令，或留给 MOVING_TO_MAX 逻辑处理
                  }
                  last_fire_cmd_in_stop = fire_cmd_;
                  break;
                case PushState::IDLE:
                default:
                  push_motor_setpoint_angle_ = min_push_motor_angle_;
                  break;
              }
              break;
            }
            case LaunchMode::FULL_FIRE:
            {
              // 连发模式
              if (push_state_ == PushState::IDLE)
              {
                push_state_ = PushState::MOVING_TO_MAX;
              }

              switch (push_state_)
              {
                case PushState::MOVING_TO_MAX:
                  push_motor_setpoint_angle_ = max_push_motor_angle_;
                  if (push_motor_angle_ > max_push_motor_angle_ - 1.0f)
                  {
                    push_state_ = PushState::AT_MAX_WAITING;
                  }
                  break;

                case PushState::AT_MAX_WAITING:
                  push_motor_setpoint_angle_ = max_push_motor_angle_;
                  // 如果检测到发射，立即复位准备下一发
                  if (launch_detected_)
                  {
                    push_state_ = PushState::MOVING_TO_MIN;
                    launch_detected_ = false;
                  }
                  // 如果fire_cmd变为false，也复位
                  if (!fire_cmd_)
                  {
                    push_state_ = PushState::MOVING_TO_MIN;
                  }
                  break;

                case PushState::MOVING_TO_MIN:
                  fric_mode_ = DartLauncherMode::FRIC_STOP;
                  push_motor_setpoint_angle_ = min_push_motor_angle_;
                  if (push_motor_angle_ < min_push_motor_angle_ + 2.0f)
                  {
                    push_state_ = PushState::IDLE;
                    // 推杆复位完成，如果不再发射则停止摩擦轮
                    if (!fire_cmd_)
                    {
                      fric_mode_ = DartLauncherMode::FRIC_STOP;
                      is_firing_ = false;  // 重置发射状态
                    }
                  }
                  break;

                case PushState::IDLE:
                default:
                  push_motor_setpoint_angle_ = min_push_motor_angle_;
                  break;
              }
              break;
            }
          }
        }
        else
        {
          // 摩擦轮未准备好或没有发射命令，保持在IDLE状态
          if (push_state_ != PushState::MOVING_TO_MIN &&
              push_state_ != PushState::AT_MAX_WAITING)
          {
            push_state_ = PushState::IDLE;
            push_motor_setpoint_angle_ = min_push_motor_angle_;
          }
        }
      }
      else
      {
        // FRIC_STOP模式，停止所有动作
        push_motor_setpoint_angle_ = min_push_motor_angle_;
        push_state_ = PushState::IDLE;
        fire_cmd_ = false;
        is_firing_ = false;
        fric_ready_ = false;
      }
    }

    push_motor_setpoint_speed_ = push_motor_angle_pid_.Calculate(
        push_motor_setpoint_angle_, push_motor_angle_, dt_launcher_);
    push_motor_output_ = push_motor_speed_pid_.Calculate(
        push_motor_setpoint_speed_, push_motor_->GetFeedback().velocity, dt_launcher_);
    cmd_push_motor_.velocity = push_motor_output_;

    push_motor_->Control(cmd_push_motor_);
  }
  /**
   * @brief 获取飞镖事件对象，`DartMode` 的四个值注册在其上。
   *        Get the dart event object on which the four values of `DartMode` are
   *        registered.
   * @return 事件对象的引用。
   *         Reference to the event object.
   */
  LibXR::Event& GetEvent() { return dart_event_; }

  /**
   * @brief 切换模式，等价于激活对应的事件 ID。
   *        Switch the mode, equivalent to activating the corresponding event ID.
   *
   * @param mode `DartMode` 的数值。
   *             Value of `DartMode`.
   */
  void SetMode(uint32_t mode) { dart_event_.Active(mode); }
  /**
   * @brief 处理模式切换：更新模式，复位发射口状态，`GAME` 时重新初始化 yaw。
   *        Handle a mode switch: update the mode, reset the gate status and re-initialize
   *        yaw in `GAME`.
   *
   * @param mode 新模式。
   *             New mode.
   */
  void EventHandler(DartMode mode)
  {
    mode_ = static_cast<DartMode>(mode);
    if (mode == DartMode::YAW_COMMON || mode == DartMode::YAW_SCAN ||
        mode == DartMode::RELAX)
    {
      {
        ref_data_.dc.opening_status = static_cast<uint8_t>(OPENING_STATUS::DEFAULT);
      }
    }
    if (mode_ == DartMode::GAME)
    {
      delay_time_gimbal_ = 0;
      yaw_motor_state_ = YawMotorState::INITIALIZING;
    }
  }

  /**
   * @brief 按遥控输入更新发射命令与 yaw 偏移（仅 `YAW_COMMON` 与 `YAW_SCAN`）。
   *        Update the fire command and the yaw offset from the remote controller input
   *        (`YAW_COMMON` and `YAW_SCAN` only).
   */
  void DR16CONTROL()
  {
    if ((mode_ == DartMode::YAW_COMMON) || (mode_ == DartMode::YAW_SCAN))
    {
      if (cmd_data_.z > 0.7f)
      {
        ref_data_.dc.opening_status = static_cast<uint8_t>(OPENING_STATUS::ON);
        fire_cmd_ = true;
        dart_gimbal_cmd_.yaw = 0.0f;
      }
      else
      {
        fire_cmd_ = false;
      }
    }
    if (mode_ == DartMode::YAW_COMMON)
    {
      yaw_motor_state_ = YawMotorState::NORMAL_CONTROL;
      dart_gimbal_cmd_.yaw = cmd_data_.x;
    }
  }

 private:
  // === 云台成员 ===
  DartGimbalEvent current_mode_ = DartGimbalEvent::SET_MODE_COMMON;

  float dt_gimbal_ = 0.0f;
  LibXR::MicrosecondTimestamp last_online_time_;
  LibXR::MillisecondTimestamp last_gimbal_data_time_ = 0;
  LibXR::PID<float> pid_yaw_angle_;
  LibXR::PID<float> pid_yaw_speed_;

  Motor* motor_yaw_;
  Motor* motor_pitch_;

  Motor::Feedback motor_yaw_feedback_;
  Motor::Feedback motor_pitch_feedback_;

  float yaw_motor_angle_ = 0.0f;
  const float YAW_MOTOR_GEAR_RATIO = 19.2032f;
  DartGimbalCMD dart_gimbal_cmd_ = {0.0f};

  // 有限状态机相关成员变量
  YawMotorState yaw_motor_state_ = YawMotorState::NORMAL_CONTROL;
  bool scan_direction_ = false;  // true: 向max方向, false: 向min方向
  uint32_t delay_time_gimbal_ = 0;
  float yaw_motor_setpoint_angle_ = 0.0f;
  float min_yaw_motor_angle_ = 0.0f;
  float max_yaw_motor_angle_ = 0.0f;
  const float SCAN_SPEED = 8.0f;  // 扫描速度 (rad/s)

  LibXR::Topic dart_gimbal_data_tp_ =
      LibXR::Topic::CreateTopic<DartGimbalCMD>("host_dart_gimbal_cmd");

  // === 发射机构成员 ===
  CMD::ChassisCMD cmd_data_{};
  float dt_launcher_ = 0.0f;
  LibXR::MillisecondTimestamp last_online_time_launcher_ = 0;

  RMMotor* motor_fric_front_left_;
  RMMotor* motor_fric_front_right_;
  RMMotor* motor_fric_back_left_;
  RMMotor* motor_fric_back_right_;
  RMMotor* push_motor_;

  Motor::Feedback param_motor_fric_front_left_;
  Motor::Feedback param_motor_fric_front_right_;
  Motor::Feedback param_motor_fric_back_left_;
  Motor::Feedback param_motor_fric_back_right_;
  Motor::Feedback param_push_motor_;

  Motor::MotorCmd cmd_fric_front_left_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0.0f};
  Motor::MotorCmd cmd_fric_front_right_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0.0f};
  Motor::MotorCmd cmd_fric_back_left_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0.0f};
  Motor::MotorCmd cmd_fric_back_right_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0.0f};
  Motor::MotorCmd cmd_push_motor_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 36.0f,
                      .velocity = 0.0f};
  float push_motor_gear_ratio_;

  bool push_motor_init_ = false;
  bool fric_ready_ = false;
  bool fire_cmd_ = false;
  bool is_firing_ = false;
  bool launch_detected_ = false;

  LaunchMode launch_mode_ = LaunchMode::SINGLE_SHOT;
  PushState push_state_ = PushState::IDLE;
  LibXR::MillisecondTimestamp launch_complete_timestamp_ = 0;
  LibXR::MillisecondTimestamp launch_detect_timestamp_ = 0;
  LibXR::MillisecondTimestamp at_max_timestamp_ = 0;

  float push_motor_angle_ = 0.0f;
  float min_push_motor_angle_ = 0.0f;
  float max_push_motor_angle_ = 0.0f;
  float push_motor_setpoint_speed_ = 0.0f;
  float push_motor_setpoint_angle_ = 0.0f;
  float fric1_setpoint_speed_ = 0.0f;
  float fric2_setpoint_speed_ = 0.0f;
  float fric_target_speed_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float fric_output_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float push_motor_output_ = 0.0f;

  LibXR::Topic launcher_topic_ = LibXR::Topic::CreateTopic<bool>("launch_flag");
  bool marked_launch_ = false;

  LibXR::PID<float> fric_speed_pid_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  LibXR::PID<float> push_motor_speed_pid_;
  LibXR::PID<float> push_motor_angle_pid_;

  CMD* cmd_;

  DartLauncherMode fric_mode_ = DartLauncherMode::FRIC_STOP;
  LibXR::Event dart_event_;
  Referee::LauncherPack ref_data_{
      .dc = {.opening_status = static_cast<uint8_t>(OPENING_STATUS::DEFAULT)}};
  uint32_t delay_time_launcher_ = 0;
  DartMode mode_ = DartMode::RELAX;
  const char* launcher_cmd_topic_name_ = nullptr;
  const char* launcher_ref_topic_name_ = nullptr;
  const char* chassis_cmd_topic_name_ = nullptr;
  const char* fire_notify_topic_name_ = nullptr;
  LibXR::Thread thread_;
  LibXR::Mutex mutex_;
  uint16_t cnt = 0;
  uint16_t cnt_start = 0;
  uint16_t cnt_lost = 0;
};
