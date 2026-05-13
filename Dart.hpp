#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: InteSET_MODE_RELAXgrated Dart system combining gimbal and launcher functionality
constructor_args:
  -task_stack_depth: 4096
  -pid_yaw_angle:
      k: 1.0
      p: 900.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  -pid_yaw_speed:
      k: 1.0
      p: 0.001
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  -motor_yaw: '@&motor_yaw'
  -motor_pit: '@&motor_pitch'
  -motor_fric_front_left: '@&motor_fric_front_left'
  -motor_fric_front_right: '@&motor_fric_front_right'
  -motor_fric_back_left: '@&motor_fric_back_left'
  -motor_fric_back_right: '@&motor_fric_back_right'
  -push_motor: '@&push_motor'
  -push_motor_gear_ratio: 36.0
  -fric1_setpoint_speed: 4500.0
  -fric2_setpoint_speed: 4400.0
  -fric_speed_pid_0:
      k: 1.0
      p: 0.001
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  -fric_speed_pid_1:
      k: 1.0
      p: 0.001
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  -fric_speed_pid_2:
      k: 1.0
      p: 0.001
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  -fric_speed_pid_3:
      k: 1.0
      p: 0.001
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  -push_motor_speed_pid:
      k: 1.0
      p: 0.0008
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  -push_motor_angle_pid:
      k: 1.0
      p: 1000.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 2000.0
      cycle: false
  -cmd:‘&@cmd’
template_args: []
required_hardware:
  - dr16
  - can
depends:
  - qdu-feature/CMD
  - qdu_feature/RMMotor
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cstdint>

#include "CMD.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "app_framework.hpp"
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

class Dart : public LibXR::Application {
 public:
  enum class LaunchMode : uint8_t {
    SINGLE_SHOT = 0,  // 单发模式
    FULL_FIRE = 1     // 连发模式
  };

  enum class PushState : uint8_t {
    IDLE,            // 空闲状态，在最小位置
    MOVING_TO_MAX,   // 向最大位置移动
    AT_MAX_WAITING,  // 在最大位置等待（仅单发模式）
    MOVING_TO_MIN,   // 向最小位置复位
    STOP_MOVING,
  };

  struct DartGimbalCMD {
    float yaw;
  };

  enum class DartGimbalEvent : uint8_t {
    SET_MODE_RELAX = 0,
    SET_MODE_COMMON = 1,
  };

  enum class DartEvent : uint8_t {
    SET_MODE_FRIC_START,
    SET_MODE_FRIC_STOP,
  };

  enum class DartMode : uint8_t {
    FRIC_START,
    FRIC_STOP,
  };

  // Yaw motor状态机状态
  enum class YawMotorState : uint8_t {
    INITIALIZING,   // 初始化状态：向正方向移动寻找极限位置
    SCANNING,       // 扫描状态：在max和min之间来回扫描
    NORMAL_CONTROL  // 正常控制状态：接收上位机指令进行控制
  };

  Dart(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
       uint32_t task_stack_depth, LibXR::PID<float>::Param pid_yaw_angle,
       LibXR::PID<float>::Param pid_yaw_speed, Motor* motor_yaw,
       Motor* motor_pitch, RMMotor* motor_fric_front_left_,
       RMMotor* motor_fric_front_right_, RMMotor* motor_fric_back_left_,
       RMMotor* motor_fric_back_right_, RMMotor* push_motor,
       float push_motor_gear_ratio_, float fric1_setpoint_speed,
       float fric2_setpoint_speed, LibXR::PID<float>::Param fric_speed_pid_0,
       LibXR::PID<float>::Param fric_speed_pid_1,
       LibXR::PID<float>::Param fric_speed_pid_2,
       LibXR::PID<float>::Param fric_speed_pid_3,
       LibXR::PID<float>::Param push_motor_speed_pid_,
       LibXR::PID<float>::Param push_motor_angle_pid, CMD* cmd)
      : pid_yaw_angle_(pid_yaw_angle),
        pid_yaw_speed_(pid_yaw_speed),
        motor_yaw_(motor_yaw),
        motor_pitch_(motor_pitch),
        user_key_(hw.Find<LibXR::GPIO>("USER_KEY")),
        motor_fric_front_left_(motor_fric_front_left_),
        motor_fric_front_right_(motor_fric_front_right_),
        motor_fric_back_left_(motor_fric_back_left_),
        motor_fric_back_right_(motor_fric_back_right_),
        push_motor_(push_motor),
        push_motor_gear_ratio_(push_motor_gear_ratio_),
        fric1_setpoint_speed_(fric1_setpoint_speed),
        fric2_setpoint_speed_(fric2_setpoint_speed),
        fric_speed_pid_{fric_speed_pid_0, fric_speed_pid_1, fric_speed_pid_2,
                        fric_speed_pid_3},
        push_motor_speed_pid_(push_motor_speed_pid_),
        push_motor_angle_pid_(push_motor_angle_pid),
        cmd_(cmd) {
    UNUSED(hw);
    UNUSED(app);
    ref_data_.dc.opening_status = 3;

    last_online_time_ = LibXR::Timebase::GetMicroseconds();
    thread_.Create(this, ThreadFunction, "dartThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);

    // Launcher event callbacks
    auto user_key_callback = LibXR::GPIO::Callback::Create(
        [](bool in_isr, Dart* self) {
          UNUSED(in_isr);
          if (self->push_motor_init_) {
            // 切换发射状态：第一次按键开始发射，第二次按键停止发射
            if (self->is_firing_) {
              // 正在发射，停止发射
              self->fire_cmd_ = false;
              self->is_firing_ = false;
            } else {
              // 未发射，开始发射
              self->fire_cmd_ = true;
              self->is_firing_ = true;
              self->mode_ = DartMode::FRIC_START;
            }
          }
        },
        this);
    user_key_->RegisterCallback(user_key_callback);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Dart* dart, uint32_t event_id) {
          UNUSED(in_isr);
          dart->EventHandler(event_id);
        },
        this);
    dart_event_.Register(static_cast<uint32_t>(DartEvent::SET_MODE_FRIC_START),
                         callback);
    dart_event_.Register(static_cast<uint32_t>(DartEvent::SET_MODE_FRIC_STOP),
                         callback);
  }

  static void ThreadFunction(Dart* dart) {
    LibXR::Topic::ASyncSubscriber<CMD::GimbalCMD> dart_gimbal_suber(
        "host_dart_gimbal_cmd");
    LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> launch_notify_suber(
        "launcher_cmd");
    LibXR::Topic::ASyncSubscriber<Referee::LauncherPack> launcher_ref(
        "launcher_ref");
    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber("chassis_cmd");
    LibXR::Topic::ASyncSubscriber<bool> fire_notify_suber("fire_notify");
    dart_gimbal_suber.StartWaiting();
    launch_notify_suber.StartWaiting();
    launcher_ref.StartWaiting();
    cmd_suber.StartWaiting();
    fire_notify_suber.StartWaiting();
    while (1) {
      if (cmd_suber.Available()) {
        dart->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }
      if (dart_gimbal_suber.Available()) {
        dart->dart_gimbal_cmd_.yaw = dart_gimbal_suber.GetData().yaw;
        dart->yaw_motor_state_ = YawMotorState::NORMAL_CONTROL;
        // dart->cnt++;
        dart_gimbal_suber.StartWaiting();
      }
      // else {
      //   dart->dart_gimbal_cmd_.yaw = 0.0f;
      // }
      // if (launch_notify_suber.Available()) {
      //   dart->fire_cmd_ = launch_notify_suber.GetData().isfire;
      //   dart->yaw_motor_state_ = YawMotorState::NORMAL_CONTROL;
      //   launch_notify_suber.StartWaiting();
      // } else {
      //   dart->dart_gimbal_cmd_.yaw = 0.0f;
      // }

      if (launcher_ref.Available()) {
        dart->ref_data_.dc = launcher_ref.GetData().dc;
        launcher_ref.StartWaiting();
      }
      if (fire_notify_suber.Available()) {
        dart->fire_cmd_ = fire_notify_suber.GetData();
        fire_notify_suber.StartWaiting();
      }

      dart->UpdateYaw();
      dart->UpdatePitch();
      dart->UpdateFric();
      dart->UpdatePushMotor();
      dart->DetectLaunch();
      dart->ControlYaw();
      dart->ControlPitch();
      dart->ControlFric();
      dart->ControlPushMotor();

      LibXR::Thread::Sleep(2);
    }
  }

  void OnMonitor() override {}

  // === Gimbal Functions ===
  void UpdateYaw() {
    auto now = LibXR::Timebase::GetMicroseconds();
    dt_gimbal_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    const float LAST_YAW_MOTOR_ANGLE =
        LibXR::CycleValue<float>(motor_yaw_feedback_.abs_angle);
    motor_yaw_->Update();
    motor_yaw_feedback_ = motor_yaw_->GetFeedback();
    const float DELTA_YAW_MOTOR_ANGLE =
        LibXR::CycleValue<float>(motor_yaw_feedback_.abs_angle) -
        LAST_YAW_MOTOR_ANGLE;
    this->yaw_motor_angle_ += DELTA_YAW_MOTOR_ANGLE / YAW_MOTOR_GEAR_RATIO;
  }

  void UpdatePitch() {
    motor_pitch_->Update();
    motor_pitch_feedback_ = motor_pitch_->GetFeedback();
  }

  void ControlYaw() {
    if (current_mode_ == DartGimbalEvent::SET_MODE_RELAX) {
      motor_yaw_->Relax();
      return;
    }

    float out_yaw = 0.0f;

    switch (yaw_motor_state_) {
      case YawMotorState::INITIALIZING: {
        // 向正方向移动寻找极限位置
        yaw_motor_setpoint_angle_ -= LibXR::TWO_PI / 250.0f;
        delay_time_gimbal_++;

        // 检测扭矩是否变大（超过阈值），表示到达机械极限
        if (delay_time_gimbal_ > 150 &&
            std::abs(motor_yaw_feedback_.torque) > 0.075f) {
          min_yaw_motor_angle_ = yaw_motor_angle_;
          max_yaw_motor_angle_ = min_yaw_motor_angle_ + 55.0f;
          yaw_motor_setpoint_angle_ = max_yaw_motor_angle_;
          scan_direction_ = true;  // 开始向max方向扫描
          yaw_motor_state_ = YawMotorState::SCANNING;
        }
        // 使用角度PID控制到设定点
        float target_yaw_speed = pid_yaw_angle_.Calculate(
            yaw_motor_setpoint_angle_, yaw_motor_angle_, dt_gimbal_);
        out_yaw = pid_yaw_speed_.Calculate(
            target_yaw_speed, motor_yaw_feedback_.velocity, dt_gimbal_);
        break;
      }

      case YawMotorState::SCANNING: {
        // 在max和min之间扫描
        if (scan_direction_) {
          // 向max方向扫描
          yaw_motor_setpoint_angle_ += SCAN_SPEED * dt_gimbal_;
          if (yaw_motor_angle_ >= max_yaw_motor_angle_ - 1.0f) {
            scan_direction_ = false;                           // 切换方向
            yaw_motor_setpoint_angle_ = max_yaw_motor_angle_;  // 限制在边界
          }
        } else {
          // 向min方向扫描
          yaw_motor_setpoint_angle_ -= SCAN_SPEED * dt_gimbal_;
          if (yaw_motor_angle_ <= min_yaw_motor_angle_ + 2.0f) {
            scan_direction_ = true;                            // 切换方向
            yaw_motor_setpoint_angle_ = min_yaw_motor_angle_;  // 限制在边界
          }
        }
        // 确保设定点在范围内
        yaw_motor_setpoint_angle_ =
            std::clamp(yaw_motor_setpoint_angle_, min_yaw_motor_angle_,
                       max_yaw_motor_angle_);
        // 使用角度PID控制到设定点
        float target_yaw_speed = pid_yaw_angle_.Calculate(
            yaw_motor_setpoint_angle_, yaw_motor_angle_, dt_gimbal_);
        out_yaw = pid_yaw_speed_.Calculate(
            target_yaw_speed, motor_yaw_feedback_.velocity, dt_gimbal_);
        break;
      }

      case YawMotorState::NORMAL_CONTROL: {
        // 正常控制模式，使用上位机指令，但限制在min和max之间
        float target_yaw_angle = dart_gimbal_cmd_.yaw + yaw_motor_angle_;
        // 如果已经完成初始化，限制目标角度在min和max之间
        if (min_yaw_motor_angle_ != 0.0f || max_yaw_motor_angle_ != 0.0f) {
          // 限制绝对目标位置在范围内
          target_yaw_angle = std::clamp(target_yaw_angle, min_yaw_motor_angle_,
                                        max_yaw_motor_angle_);
        }
        Solve(out_yaw, target_yaw_angle, dt_gimbal_);
        break;
      }
    }

    auto yaw_motor_cmd = Motor::MotorCmd(
        {.mode = Motor::ControlMode::MODE_CURRENT, .velocity = out_yaw});

    auto motor_control = [&](Motor* motor, const Motor::Feedback& fb,
                             const Motor::MotorCmd& cmd) {
      motor->Control(cmd);
    };

    motor_control(motor_yaw_, motor_yaw_feedback_, yaw_motor_cmd);
  }

  void ControlPitch() {
    motor_pitch_->Control(Motor::MotorCmd(
        {.mode = Motor::ControlMode::MODE_CURRENT, .velocity = 0.0f}));
  }

  /**
   * @brief 解算 PID 控制输出
   *
   * @param yaw_output Yaw 轴输出引用
   * @param target_yaw_angle 目标 Yaw 角度
   * @param dt_ 时间间隔
   */
  void Solve(float& yaw_output, float target_yaw_angle, float dt_) {
    float yaw_error = target_yaw_angle - yaw_motor_angle_;
    float target_yaw_speed = pid_yaw_angle_.Calculate(yaw_error, 0.0f, dt_);
    float fb_yaw = pid_yaw_speed_.Calculate(target_yaw_speed,
                                            motor_yaw_feedback_.velocity, dt_);
    yaw_output = fb_yaw;
  }

  // === Launcher Functions ===
  void UpdateFric() {
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

  void UpdatePushMotor() {
    const float LAST_PUSH_MOTOR_ANGLE =
        LibXR::CycleValue<float>(param_push_motor_.abs_angle);
    push_motor_->Update();
    param_push_motor_ = push_motor_->GetFeedback();
    const float DELTA_PUSH_MOTOR_ANGLE =
        LibXR::CycleValue<float>(param_push_motor_.abs_angle) -
        LAST_PUSH_MOTOR_ANGLE;
    this->push_motor_angle_ += DELTA_PUSH_MOTOR_ANGLE / push_motor_gear_ratio_;
  }

  void DetectLaunch() {
    // 检测是否发生发射
    bool should_mark_launch = false;
    auto current_time = LibXR::Timebase::GetMilliseconds();

    if (fric_ready_ && (push_state_ == PushState::AT_MAX_WAITING ||
                        push_state_ == PushState::MOVING_TO_MAX)) {
      // 检测扭矩变大
      if (std::abs(motor_fric_back_left_->GetFeedback().torque) > 0.1f) {
        // 如果还没有开始100ms计时，则开始
        if (!launch_detected_) {
          launch_detected_ = true;
          launch_detect_timestamp_ = current_time;
        }
      }
    }

    // 处理100ms信号输出
    if (launch_detected_) {
      // 如果在100ms内，输出true
      if (current_time - launch_detect_timestamp_ < 100) {
        should_mark_launch = true;
      } else {
        // 100ms已过，重置状态，允许下次检测
        launch_detected_ = false;
        should_mark_launch = false;
      }
    }

    marked_launch_ = should_mark_launch;
    launcher_topic_.Publish(marked_launch_);
  }
  void ControlFric() {
    // 只在推杆电机复位完成时停止摩擦轮（在ControlPushMotor中处理）
    if (launch_mode_ == LaunchMode::SINGLE_SHOT) {
      if (ref_data_.dc.opening_status == 2 ||
          ref_data_.dc.opening_status == 0) {
        mode_ = DartMode::FRIC_START;
      } else {
        mode_ = DartMode::FRIC_STOP;
      }
    } else if (launch_mode_ == LaunchMode::FULL_FIRE) {
      // FULL_FIRE模式下直接根据fire_cmd控制
      if (cmd_data_.x > 0.5f) {
        mode_ = DartMode::FRIC_START;
      }
    }

    switch (mode_) {
      case DartMode::FRIC_STOP:
        fric_target_speed_[0] = 0;
        fric_target_speed_[1] = 0;
        fric_target_speed_[2] = 0;
        fric_target_speed_[3] = 0;
        for (auto& i : fric_speed_pid_) {
          i.SetOutLimit(0.1f);
          fric_ready_ = false;
        }
        break;
      case DartMode::FRIC_START:
        fric_target_speed_[0] = fric2_setpoint_speed_;
        fric_target_speed_[1] = fric2_setpoint_speed_;
        fric_target_speed_[2] = fric1_setpoint_speed_;
        fric_target_speed_[3] = fric1_setpoint_speed_;
        if (param_motor_fric_back_right_.velocity > fric1_setpoint_speed_) {
          for (auto& i : fric_speed_pid_) {
            i.SetOutLimit(0.0f);
            fric_ready_ = true;
          }
        }
        break;
    }

    fric_output_[0] = fric_speed_pid_[0].Calculate(
        fric_target_speed_[0], param_motor_fric_front_left_.velocity,
        dt_launcher_);
    fric_output_[1] = fric_speed_pid_[1].Calculate(
        fric_target_speed_[1], param_motor_fric_front_right_.velocity,
        dt_launcher_);
    fric_output_[2] = fric_speed_pid_[2].Calculate(
        fric_target_speed_[2], param_motor_fric_back_left_.velocity,
        dt_launcher_);
    fric_output_[3] = fric_speed_pid_[3].Calculate(
        fric_target_speed_[3], param_motor_fric_back_right_.velocity,
        dt_launcher_);

    cmd_fric_front_left_.velocity = fric_output_[0];
    cmd_fric_front_right_.velocity = fric_output_[1];
    cmd_fric_back_left_.velocity = fric_output_[2];
    cmd_fric_back_right_.velocity = fric_output_[3];

    motor_fric_front_left_->Control(cmd_fric_front_left_);
    motor_fric_front_right_->Control(cmd_fric_front_right_);
    motor_fric_back_left_->Control(cmd_fric_back_left_);
    motor_fric_back_right_->Control(cmd_fric_back_right_);
  }

  void ControlPushMotor() {
    if (!push_motor_init_) {
      push_motor_setpoint_angle_ -= LibXR::TWO_PI / 250.0f;
      push_motor_angle_pid_.SetOutLimit(2000.0f);
      delay_time_launcher_++;
      if (delay_time_launcher_ > 250) {
        if (std::abs(param_push_motor_.torque) > 0.02) {
          push_motor_init_ = true;
          min_push_motor_angle_ = push_motor_angle_ + 2.0f;
          max_push_motor_angle_ = push_motor_angle_ + 61.0f;
          push_motor_setpoint_angle_ = min_push_motor_angle_;
          // 初始化完成后保持停止状态，等待发射命令
          mode_ = DartMode::FRIC_STOP;
        }
      }
    } else {
      // 推杆已初始化，处理发射逻辑

      // 如果模式是FRIC_START，启动摩擦轮并等待准备就绪
      if (mode_ == DartMode::FRIC_START) {
        // 检查摩擦轮是否准备好
        if (!fric_ready_ &&
            param_motor_fric_back_right_.velocity > fric1_setpoint_speed_ &&
            param_motor_fric_back_left_.velocity > fric1_setpoint_speed_) {
          fric_ready_ = true;
          for (auto& i : fric_speed_pid_) {
            i.SetOutLimit(0.0f);
          }
        }

        // 如果摩擦轮已准备好且有发射命令，处理状态机
        if (fric_ready_) {
          switch (launch_mode_) {
            case LaunchMode::SINGLE_SHOT: {
              // 单发模式 - 只有在fire_cmd从0变为1时才触发发射
              static bool last_fire_cmd = false;

              // 检测fire_cmd上升沿
              if (!last_fire_cmd && fire_cmd_) {
                if (ref_data_.dc.opening_status == 0) {
                  // 触发发射，只在IDLE状态下才开始新的发射循环
                  if (push_state_ == PushState::IDLE) {
                    push_state_ = PushState::MOVING_TO_MAX;
                  }
                }
              }
              last_fire_cmd = fire_cmd_;

              switch (push_state_) {
                case PushState::MOVING_TO_MAX:
                  push_motor_setpoint_angle_ = max_push_motor_angle_;
                  if (push_motor_angle_ > max_push_motor_angle_ - 2.0f) {
                    push_state_ = PushState::AT_MAX_WAITING;
                  }
                  if (launch_detected_) {
                    push_state_ = PushState::STOP_MOVING;
                    // launch_detected_ = false;
                    //  在STOP_MOVING状态中，fire_cmd应被重置，等待新的上升沿
                    fire_cmd_ = false;
                    last_fire_cmd = false;  // 重置静态变量以准备下一次检测
                    cnt++;
                  }
                  break;

                case PushState::AT_MAX_WAITING:
                  push_motor_setpoint_angle_ = max_push_motor_angle_;
                  // 如果检测到发射，停止
                  if (launch_detected_) {
                    push_state_ = PushState::STOP_MOVING;
                    // launch_detected_ = false;
                    //  在STOP_MOVING状态中，fire_cmd应被重置，等待新的上升沿
                    fire_cmd_ = false;
                    last_fire_cmd = false;  // 重置静态变量以准备下一次检测
                    cnt++;
                  }
                  break;

                case PushState::MOVING_TO_MIN:
                  push_motor_setpoint_angle_ = min_push_motor_angle_;
                  if (push_motor_angle_ < min_push_motor_angle_ + 2.0f) {
                    push_state_ = PushState::IDLE;
                    // 推杆复位完成，停止摩擦轮
                    mode_ = DartMode::FRIC_STOP;
                    fire_cmd_ = false;  // 重置发射命令，等待下次触发
                    is_firing_ = false;
                  }
                  break;
                case PushState::STOP_MOVING:
                  // 停止在当前位置
                  push_motor_setpoint_angle_ = push_motor_angle_;
                  // 在STOP_MOVING状态中，等待fire_cmd的上升沿以重置状态机
                  static bool last_fire_cmd_in_stop = false;
                  if (!last_fire_cmd_in_stop && fire_cmd_) {
                    // 检测到上升沿，直接进入 MOVING_TO_MAX 开始新发射
                    push_state_ = PushState::MOVING_TO_MAX;
                    fire_cmd_ =
                        false;  // 可选：消费命令，或留给 MOVING_TO_MAX 逻辑处理
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
            case LaunchMode::FULL_FIRE: {
              // 连发模式
              if (push_state_ == PushState::IDLE) {
                push_state_ = PushState::MOVING_TO_MAX;
              }

              switch (push_state_) {
                case PushState::MOVING_TO_MAX:
                  push_motor_setpoint_angle_ = max_push_motor_angle_;
                  if (push_motor_angle_ > max_push_motor_angle_ - 1.0f) {
                    push_state_ = PushState::AT_MAX_WAITING;
                  }
                  break;

                case PushState::AT_MAX_WAITING:
                  push_motor_setpoint_angle_ = max_push_motor_angle_;
                  // 如果检测到发射，立即复位准备下一发
                  if (launch_detected_) {
                    push_state_ = PushState::MOVING_TO_MIN;
                    launch_detected_ = false;
                  }
                  // 如果fire_cmd变为false，也复位
                  if (!fire_cmd_) {
                    push_state_ = PushState::MOVING_TO_MIN;
                  }
                  break;

                case PushState::MOVING_TO_MIN:
                  mode_ = DartMode::FRIC_STOP;
                  push_motor_setpoint_angle_ = min_push_motor_angle_;
                  if (push_motor_angle_ < min_push_motor_angle_ + 2.0f) {
                    push_state_ = PushState::IDLE;
                    // 推杆复位完成，如果不再发射则停止摩擦轮
                    if (!fire_cmd_) {
                      mode_ = DartMode::FRIC_STOP;
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
        } else {
          // 摩擦轮未准备好或没有发射命令，保持在IDLE状态
          if (push_state_ != PushState::MOVING_TO_MIN &&
              push_state_ != PushState::AT_MAX_WAITING) {
            push_state_ = PushState::IDLE;
            push_motor_setpoint_angle_ = min_push_motor_angle_;
          }
        }
      } else {
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
        push_motor_setpoint_speed_, push_motor_->GetFeedback().velocity,
        dt_launcher_);
    cmd_push_motor_.velocity = push_motor_output_;

    push_motor_->Control(cmd_push_motor_);
  }
  LibXR::Event& GetEvent() { return dart_event_; }

  void SetMode(uint32_t mode) { dart_event_.Active(mode); }
  void EventHandler(uint32_t event_id) {
    // SetMode(static_cast<uint32_t>(static_cast<DartEvent>(event_id)));
    DartEvent event = static_cast<DartEvent>(event_id);
    if (event == DartEvent::SET_MODE_FRIC_START) {
      if (launch_mode_ == LaunchMode::FULL_FIRE) {
        mode_ = DartMode::FRIC_START;
        fire_cmd_ = true;  // 启动发射命令
      }
    } else if (event == DartEvent::SET_MODE_FRIC_STOP) {
      mode_ = DartMode::FRIC_STOP;
      fire_cmd_ = false;  // 停止发射命令
    }
  }

 private:
  // === Gimbal Members ===
  DartGimbalEvent current_mode_ = DartGimbalEvent::SET_MODE_COMMON;

  float dt_gimbal_ = 0.0f;
  LibXR::MicrosecondTimestamp last_online_time_;
  LibXR::PID<float> pid_yaw_angle_;
  LibXR::PID<float> pid_yaw_speed_;

  Motor* motor_yaw_;
  Motor* motor_pitch_;

  Motor::Feedback motor_yaw_feedback_;
  Motor::Feedback motor_pitch_feedback_;

  float yaw_motor_angle_ = 0.0f;
  const float YAW_MOTOR_GEAR_RATIO = 19.2032f;
  // CMD::LauncherCMD launcher_cmd_;
  DartGimbalCMD dart_gimbal_cmd_ = {0.0f};

  // 有限状态机相关成员变量
  YawMotorState yaw_motor_state_ = YawMotorState::INITIALIZING;
  bool scan_direction_ = false;  // true: 向max方向, false: 向min方向
  uint32_t delay_time_gimbal_ = 0;
  float yaw_motor_setpoint_angle_ = 0.0f;
  float min_yaw_motor_angle_ = 0.0f;
  float max_yaw_motor_angle_ = 0.0f;
  const float SCAN_SPEED = 8.0f;  // 扫描速度 (rad/s)

  LibXR::Topic dart_gimbal_data_tp_ =
      LibXR::Topic::CreateTopic<DartGimbalCMD>("host_dart_gimbal_cmd");

  // === Launcher Members ===
  CMD::ChassisCMD cmd_data_{};
  float dt_launcher_ = 0.0f;
  LibXR::MillisecondTimestamp last_online_time_launcher_ = 0;
  LibXR::GPIO* user_key_;

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

  DartMode mode_ = DartMode::FRIC_STOP;
  LibXR::Event dart_event_;
  Referee::LauncherPack ref_data_{};
  uint32_t delay_time_launcher_ = 0;

  LibXR::Thread thread_;
  LibXR::Mutex mutex_;
  uint16_t cnt = 0;
};
