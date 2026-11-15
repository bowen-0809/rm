/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-10-27 10:22:15
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-11-10 20:13:57
 * @FilePath: \undefinedc:\wangbw\rm\project\prj2\base\Tasks\main_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
*******************************************************************************
* @file      :main_task.cpp
* @brief     :
* @history   :
*  Version     Date            Author          Note
*  V0.9.0      yyyy-mm-dd      <author>        1. <note>
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2024 Hello World Team，Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"
#include "system_user.hpp"

#include "DT7.hpp"
#include "HW_can.hpp"
#include "chassis.hpp"
#include "iwdg.h"
#include "m3508.hpp"
#include "math.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

uint32_t tick = 0;

namespace remote_control = hello_world::devices::remote_control;
static const uint8_t kRxBufLen = remote_control::kRcRxDataLen;
static uint8_t rx_buf[kRxBufLen];
remote_control::DT7 *rc_ptr;
static volatile bool rc_valid = false;
static volatile uint32_t last_rc_tick = 0;

/* 底盘模式 */
enum class ChassisMode : uint8_t {
  kSeparate = 0,  // 左拨杆 UP：分离，底盘系运动
  kFollow = 1,    // 左拨杆 MID：跟随，底盘 x 追云台 x
  kSpin = 2,      // 左拨杆 DOWN：小陀螺
};

static constexpr uint32_t kModeDebounceTicks = 50;  // 50 ms
static constexpr float kSpinWz = 3.141593f;         // 小陀螺 30 rpm
/* 小陀螺平移方向修正：实测前进偏左、后退偏右，统一向顺时针修 5° */
static constexpr float kSpinTranslateTrim = 0.0872665f;  // rad，约 5°

/*
 * 阶段 6：跟随模式
 * yaw_err = gimbal_yaw - offset，底盘用 wz 把底盘 x 轴转向云台 x 轴。
 */
static constexpr float kFollowKp = 4.20f;       // 正常跟随
static constexpr float kFollowKpLarge = 6.00f;  // 初始大角度差时快速对齐
static constexpr float kFollowLargeThreshold = 0.20f; // rad，约 11.5°
static constexpr float kFollowKd = 1.20f;       // 对“实际-期望”角速度差阻尼
static constexpr float kFollowYawRateFF = 1.05f; // 稳态按云台转速前馈并略补足
static constexpr float kFollowWzLimit = 9.0f;   // 底盘内部最大角速度
static constexpr float kFollowDeadband = 0.0067f; // rad，约 0.38°
static constexpr float kFollowMinWz = 1.00f;    // 保持低速，避免重新出现对齐震荡
static constexpr float kFollowStopExit = 0.030f; // rad，约 1.72°，减少对齐后微调
static constexpr float kFollowSign = 1.0f;
static bool follow_settled = false;

/*
 * 物理对正云台 x 轴与底盘 x 轴时实测：
 *   dbg_follow_err = -0.495155334 rad
 * 说明自动 offset 比实际零位少了这部分，跟随模式加回来。
 * 只影响跟随模式，不影响分离/小陀螺。
 */
static constexpr float kFollowYawTrim = 0.0f;  // 4:1 比例修正后重新标定

/* 三种模式全部基于云台坐标系 */
static constexpr bool kSeparateUseGimbalFrame = true;
static constexpr bool kSpinUseGimbalFrame = true;

/* 真实 Yaw 由 CAN1 0x13 更新；使用固定机械零位 */
static constexpr float kFixedYawOffset = -0.709306f;
volatile float gimbal_yaw = 0.0f;
volatile float gimbal_yaw_offset = 0.0f;
volatile uint8_t gimbal_yaw_valid = 0u;
volatile uint32_t gimbal_yaw_tick = 0u;
static bool yaw_offset_initialized = false;
volatile float dbg_yaw = 0.0f;
volatile float dbg_vx_g = 0.0f;
volatile float dbg_vy_g = 0.0f;

/* 底盘速度上限 */
static constexpr float kMaxVx = 2.0f;   // m/s，整体平移速度翻倍
static constexpr float kMaxVy = 2.0f;   // m/s，整体平移速度翻倍
static constexpr float kMaxWz = 2.0f;   // rad/s，拨轮自转速度翻倍

/* 底盘速度斜坡，避免满杆时瞬间给轮子太大目标导致打滑 */
static constexpr float kMaxAccel = 2.5f;          // m/s^2
static constexpr float kMaxDecel = 6.0f;          // m/s^2，回中/减速更快
static constexpr float kMaxAngularAccel = 12.0f;  // rad/s^2，进一步减小跟随滞后
static constexpr float kMaxAngularDecel = 15.0f;  // rad/s^2
/* 小陀螺单独放缓加减速，给云台 Yaw 留出补偿时间 */
static constexpr float kSpinAngularAccel = 2.5f;  // rad/s^2
static constexpr float kSpinAngularDecel = 4.0f;  // rad/s^2
static float last_vx_cmd = 0.0f;
static float last_vy_cmd = 0.0f;
static float last_wz_cmd = 0.0f;

static float SlewLimit(float target, float current, float max_delta) {
  const float diff = target - current;
  if (diff > max_delta) {
    return current + max_delta;
  }
  if (diff < -max_delta) {
    return current - max_delta;
  }
  return target;
}

/* 横移/自转/俯仰方向校准：方向反了就改符号 */
static constexpr float kRcLhSign = -1.0f;
static constexpr float kRcWheelSign = 1.0f;
/* 实测 Pitch 位置：最高点约 -2.88 rad，最低点约 +0.20 rad。
 * 实车确认右摇杆上下方向还需要取反，所以这里取 -1。 */
static constexpr float kPitchSign = 1.0f;  // 右摇杆上下方向取反后修正
/* Pitch 右摇杆给角速度指令，云台侧积分成目标 */
static constexpr float kPitchStickRate = 0.65f;   // rad/s，满杆速度略提升
static constexpr float kPitchStickDeadband = 0.060f; // 摇杆中位死区
/* 右摇杆向左时云台应向左转；实测方向相反，所以这里取反 */
static constexpr float kYawCmdSign = -1.0f;
/* Yaw 摇杆速度比例：当前转速降低一半 */
static constexpr float kYawStickSpeedScale = 0.5f;
/* 右摇杆回中死区，避免微小中位偏移让 yaw_des 持续积分 */
static constexpr float kYawStickDeadband = 0.03f;
/* 通过 GimbalCmd.reserved 发送底盘实际 wz，int8 单位 0.050 rad/s */
static constexpr float kChassisWzLsb = 0.050f;
static constexpr float kChassisWzFFDeadband = 0.060f;
/* IMU Yaw 世界闭环启用后，不再把底盘前馈混入 yaw_des */
static constexpr float kYawFeedforwardSign = 0.0f;

/**
 * @brief 四个底盘轮电机
 * @note  轮序与 PDF 拓扑一致：
 *        wheel[0] = 左前 ID1
 *        wheel[1] = 左后 ID2
 *        wheel[2] = 右后 ID3
 *        wheel[3] = 右前 ID4
 */
hello_world::motor::M3508 wheel[4] = {
    hello_world::motor::M3508(1),
    hello_world::motor::M3508(2),
    hello_world::motor::M3508(3),
    hello_world::motor::M3508(4),
};

hello_world::chassis::Chassis chassis;

/* 从四个轮子的实际转速反算底盘实际 Yaw 角速度 */
static constexpr float kChassisWheelRadius = 0.07786f;
static constexpr float kChassisGearRatio = 14.0f;
static constexpr float kChassisHalfBasePlusTrack = 0.5f;
static constexpr float kChassisWheelDirection[4] = {1.0f, 1.0f, -1.0f, -1.0f};
static float chassis_wz_actual_filtered = 0.0f;

static float CalcChassisYawRateActual(void) {
  constexpr float kPi = 3.14159265358979323846f;
  const float wheel_to_rotor =
      (60.0f * kChassisGearRatio) / (2.0f * kPi * kChassisWheelRadius);
  float wheel_speed[4];
  for (int i = 0; i < 4; ++i) {
    wheel_speed[i] =
        wheel[i].rotorRpm() / (wheel_to_rotor * kChassisWheelDirection[i]);
  }
  return ((wheel_speed[2] + wheel_speed[3]) -
          (wheel_speed[0] + wheel_speed[1])) /
         (4.0f * kChassisHalfBasePlusTrack);
}

/* 阶段 1 调试变量：方便在调试器里直接看，后续可删 */
volatile float dbg_wheel_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float dbg_wheel_current[4] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float dbg_wheel_temp[4] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile uint8_t dbg_wheel_online[4] = {0u, 0u, 0u, 0u};
volatile float dbg_wheel_target[4] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile int16_t dbg_wheel_cmd[4] = {0, 0, 0, 0};

/* 调试观测变量 */
volatile uint8_t dbg_rc_valid = 0u;
volatile uint8_t dbg_rc_l_switch = 0u;
volatile uint8_t dbg_rc_r_switch = 0u;
volatile float dbg_rc_lv = 0.0f;
volatile float dbg_rc_lh = 0.0f;
volatile float dbg_rc_wheel = 0.0f;
volatile uint8_t dbg_test_enable = 0u;
volatile uint8_t dbg_mode = 0u;
volatile float dbg_vx = 0.0f;
volatile float dbg_vy = 0.0f;
volatile float dbg_wz = 0.0f;
volatile float dbg_chassis_wz_actual = 0.0f;
volatile float dbg_follow_err = 0.0f;
volatile float dbg_yaw_for_chassis = 0.0f;

/* 底盘 -> 云台命令调试 */
volatile float dbg_rc_rh = 0.0f;
volatile float dbg_rc_rv = 0.0f;
volatile int16_t dbg_gimbal_yaw_cmd = 0;
volatile int16_t dbg_gimbal_pitch_cmd = 0;
volatile uint32_t dbg_gimbal_cmd_count = 0u;

void RobotInit(void) {
  rc_ptr = new remote_control::DT7();
  if (rc_ptr == nullptr) {
    Error_Handler();
  }
}

void MainInit(void) {
  RobotInit();

  // 设置电流限幅；启动后先发零电流
  for (auto &motor : wheel) {
    motor.reset();
    motor.setCurrentLimit(3000.0f);  // 约 3.7 A，后续按需调整
  }

  chassis.init(wheel);

  // 开启CAN1和CAN2
  CanFilter_Init(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  CanFilter_Init(&hcan2);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  // 上电先发一帧零电流，避免 C620 保持上一帧
  const int16_t zero_current[4] = {0, 0, 0, 0};
  CAN_SendWheelCurrents(zero_current);

  // 开启遥控器接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, kRxBufLen);

  // 开启定时器
  HAL_TIM_Base_Start_IT(&htim6);
}

void MainTask(void) {
  static constexpr uint32_t kStartupTicks = 2000;  // 2 s 启动保护
  constexpr float kDt = 0.001f;                    // TIM6 1 kHz

  static ChassisMode mode = ChassisMode::kSeparate;
  static uint8_t mode_stable_cnt = 0;

  bool output_enable = false;
  float vx = 0.0f;
  float vy = 0.0f;
  float wz = 0.0f;
  float vx_g = 0.0f;
  float vy_g = 0.0f;
  float yaw = 0.0f;

  /* 软超时：50 ms 没收到遥控帧就认为遥控失效 */
  if (rc_valid && (tick - last_rc_tick > 50u)) {
    rc_valid = false;
  }

  /* Yaw 反馈 100 ms 超时，避免云台掉线后使用旧角度 */
  if (gimbal_yaw_valid && (tick - gimbal_yaw_tick > 100u)) {
    gimbal_yaw_valid = 0u;
  }

  /* 固定机械零位 */
  if (!yaw_offset_initialized) {
    gimbal_yaw_offset = kFixedYawOffset;
    yaw_offset_initialized = true;
  }

  /* 模式切换：左拨杆 UP=分离，MID=跟随，DOWN=小陀螺，50 ms 去抖 */
  if (rc_valid && rc_ptr != nullptr) {
    uint8_t raw_mode = static_cast<uint8_t>(ChassisMode::kSeparate);
    switch (rc_ptr->rc_l_switch()) {
      case remote_control::kSwitchStateUp:
        raw_mode = static_cast<uint8_t>(ChassisMode::kSeparate);
        break;
      case remote_control::kSwitchStateMid:
        raw_mode = static_cast<uint8_t>(ChassisMode::kFollow);
        break;
      case remote_control::kSwitchStateDown:
        raw_mode = static_cast<uint8_t>(ChassisMode::kSpin);
        break;
      default:
        raw_mode = static_cast<uint8_t>(ChassisMode::kSeparate);
        break;
    }

    if (raw_mode != static_cast<uint8_t>(mode)) {
      if (++mode_stable_cnt >= kModeDebounceTicks) {
        mode = static_cast<ChassisMode>(raw_mode);
        mode_stable_cnt = 0;
        chassis.reset();
        last_vx_cmd = 0.0f;
        last_vy_cmd = 0.0f;
        last_wz_cmd = 0.0f;
        follow_settled = false;
      }
    } else {
      mode_stable_cnt = 0;
    }
  }
  dbg_mode = static_cast<uint8_t>(mode);

  if (tick >= kStartupTicks && rc_valid && rc_ptr != nullptr) {
    const bool estop =
        (rc_ptr->rc_r_switch() == remote_control::kSwitchStateDown);
    output_enable = !estop;
  }

  dbg_follow_err = 0.0f;
  dbg_yaw_for_chassis = 0.0f;

  if (output_enable) {
    vx_g = rc_ptr->rc_lv() * kMaxVx;
    vy_g = rc_ptr->rc_lh() * kMaxVy * kRcLhSign;

    /* 真实 Yaw：0x13 反馈减去机械零位偏移，再归一化到 [-pi, pi] */
    if (gimbal_yaw_valid) {
      yaw = gimbal_yaw - gimbal_yaw_offset;
      yaw = atan2f(sinf(yaw), cosf(yaw));
    } else {
      yaw = 0.0f;
    }

    /* 跟随模式专用的 Yaw 零位补偿 */
    float yaw_follow = atan2f(sinf(yaw + kFollowYawTrim),
                              cosf(yaw + kFollowYawTrim));

    /*
     * 选择底盘平移使用的坐标系：
     * - 分离：默认底盘系，云台转动不影响底盘运动方向；
     * - 跟随：云台系，底盘 x 轴追云台 x 轴；
     * - 小陀螺：暂按 PDF 用云台系。
     */
    float yaw_for_chassis = 0.0f;
    switch (mode) {
      case ChassisMode::kFollow:
        yaw_for_chassis = yaw_follow;
        break;
      case ChassisMode::kSpin:
        /* 修正小陀螺旋转带来的平移方向偏置 */
        yaw_for_chassis =
            kSpinUseGimbalFrame ? (yaw - kSpinTranslateTrim) : 0.0f;
        break;
      case ChassisMode::kSeparate:
      default:
        yaw_for_chassis = kSeparateUseGimbalFrame ? yaw : 0.0f;
        break;
    }

    const float c = cosf(yaw_for_chassis);
    const float s = sinf(yaw_for_chassis);

    /* 云台系 -> 底盘系 */
    vx = vx_g * c - vy_g * s;
    vy = vx_g * s + vy_g * c;
    dbg_yaw_for_chassis = yaw_for_chassis;

    switch (mode) {
      case ChassisMode::kSpin:
        /* 小陀螺：固定旋转分量，平移仍由左摇杆给 */
        wz = kSpinWz;
        break;

      case ChassisMode::kFollow:
        /*
         * 跟随：底盘 x 轴转向云台 x 轴。
         * yaw > 0 表示云台相对底盘逆时针偏转，底盘也应逆时针转；
         * 如果实车方向相反，把 kFollowSign 改成 -1。
         */
        if (gimbal_yaw_valid) {
          dbg_follow_err = yaw_follow;
          float follow_err = yaw_follow;
          const float follow_abs = fabsf(follow_err);
          if (follow_abs < kFollowDeadband) {
            follow_settled = true;
            follow_err = 0.0f;
          } else if (follow_settled && follow_abs < kFollowStopExit) {
            /* 已对齐后的小误差不再反复微调 */
            follow_err = 0.0f;
          } else {
            follow_settled = false;
          }

          if (follow_err == 0.0f) {
            wz = 0.0f;
          } else {
            /* P 控制 + 底盘实际角速度阻尼 + 云台转速前馈 */
            float rc_yaw_follow = rc_ptr->rc_rh();
            if (fabsf(rc_yaw_follow) < kYawStickDeadband) {
              rc_yaw_follow = 0.0f;
            }
            const float operator_yaw_rate =
                rc_yaw_follow * 3.0f * kYawStickSpeedScale * kYawCmdSign;
            /* 初始大角度差时提高 Kp，缩短切入跟随后的对齐时间 */
            const float follow_kp =
                (follow_abs > kFollowLargeThreshold) ? kFollowKpLarge
                                                     : kFollowKp;
            /* D 项对“实际转速 - 期望转速”阻尼，避免把稳态前馈抵消掉 */
            const float follow_rate_err =
                chassis_wz_actual_filtered - operator_yaw_rate;
            wz = kFollowSign * follow_kp * follow_err -
                 kFollowKd * follow_rate_err +
                 kFollowYawRateFF * operator_yaw_rate;

            /* 最小速度随误差减小而衰减，避免最后几度形成极限环 */
            const float taper = fminf(1.0f, fabsf(follow_err) / 0.05f);
            const float min_wz = kFollowMinWz * taper;
            if (fabsf(wz) < min_wz) {
              const float dir =
                  kFollowSign * (follow_err > 0.0f ? 1.0f : -1.0f);
              wz = dir * min_wz;
            }
            wz = fmaxf(-kFollowWzLimit, fminf(kFollowWzLimit, wz));
          }
        } else {
          /* 没有真实 Yaw 时不跟随，避免用错误角度乱转 */
          wz = 0.0f;
        }
        break;

      case ChassisMode::kSeparate:
      default:
        wz = rc_ptr->rc_wheel() * kMaxWz * kRcWheelSign;
        break;
    }
  }

  if (output_enable) {
    const float vx_step =
        (fabsf(vx) < fabsf(last_vx_cmd) ? kMaxDecel : kMaxAccel) * kDt;
    const float vy_step =
        (fabsf(vy) < fabsf(last_vy_cmd) ? kMaxDecel : kMaxAccel) * kDt;
    const bool spin_mode_now = (mode == ChassisMode::kSpin);
    const float angular_accel =
        spin_mode_now ? kSpinAngularAccel : kMaxAngularAccel;
    const float angular_decel =
        spin_mode_now ? kSpinAngularDecel : kMaxAngularDecel;
    const float wz_step = (fabsf(wz) < fabsf(last_wz_cmd)
                               ? angular_decel
                               : angular_accel) *
                          kDt;
    vx = SlewLimit(vx, last_vx_cmd, vx_step);
    vy = SlewLimit(vy, last_vy_cmd, vy_step);
    wz = SlewLimit(wz, last_wz_cmd, wz_step);
    last_vx_cmd = vx;
    last_vy_cmd = vy;
    last_wz_cmd = wz;

    const float wz_actual_raw = CalcChassisYawRateActual();
    chassis_wz_actual_filtered +=
        0.20f * (wz_actual_raw - chassis_wz_actual_filtered);
  } else {
    last_vx_cmd = 0.0f;
    last_vy_cmd = 0.0f;
    last_wz_cmd = 0.0f;
    chassis_wz_actual_filtered = 0.0f;
    follow_settled = false;
  }
  dbg_chassis_wz_actual = chassis_wz_actual_filtered;

  dbg_test_enable = output_enable ? 1u : 0u;
  dbg_rc_valid = rc_valid ? 1u : 0u;
  if (rc_ptr != nullptr) {
    dbg_rc_lv = rc_ptr->rc_lv();
    dbg_rc_lh = rc_ptr->rc_lh();
    dbg_rc_wheel = rc_ptr->rc_wheel();
    dbg_rc_rh = rc_ptr->rc_rh();
    dbg_rc_rv = rc_ptr->rc_rv();
    dbg_rc_l_switch = static_cast<uint8_t>(rc_ptr->rc_l_switch());
    dbg_rc_r_switch = static_cast<uint8_t>(rc_ptr->rc_r_switch());
  }

  dbg_yaw = yaw;
  dbg_vx_g = vx_g;
  dbg_vy_g = vy_g;
  dbg_vx = vx;
  dbg_vy = vy;
  dbg_wz = wz;

  if (output_enable) {
    chassis.setVelocity(vx, vy, wz);
    chassis.update(kDt);
  } else {
    chassis.reset();
  }

  /* 始终发 0x200：启动阶段和未使能时就是零电流 */
  const int16_t* current_cmd = chassis.currentCommand();
  CAN_SendWheelCurrents(current_cmd);

  /* 100 Hz 发送云台命令/心跳 */
  if ((tick % 10u) == 0u) {
    static uint8_t gimbal_counter = 0;
    const bool rc_ok = rc_valid && rc_ptr != nullptr;
    const bool gimbal_estop =
        rc_ok && (rc_ptr->rc_r_switch() == remote_control::kSwitchStateDown);
    float rc_yaw = rc_ok ? rc_ptr->rc_rh() : 0.0f;
    if (fabsf(rc_yaw) < kYawStickDeadband) {
      rc_yaw = 0.0f;
    }
    const float yaw_rate_cmd =
        rc_ok ? (rc_yaw * 3000.0f * kYawStickSpeedScale * kYawCmdSign +
                 kYawFeedforwardSign * chassis_wz_actual_filtered * 1000.0f)
              : 0.0f;
    const int16_t yaw_rate = static_cast<int16_t>(yaw_rate_cmd);
    float rc_pitch = rc_ok ? rc_ptr->rc_rv() : 0.0f;
    if (fabsf(rc_pitch) < kPitchStickDeadband) {
      rc_pitch = 0.0f;
    }
    /* pitch_mrad 表示 Pitch 角速度，单位 mrad/s */
    const int16_t pitch =
        static_cast<int16_t>(rc_pitch * kPitchStickRate * 1000.0f * kPitchSign);
    /* 只有分离模式允许云台编码器 Yaw 保持；跟随/小陀螺需要世界指向 */
    /* reserved 字节发送底盘实际 wz，供云台 Yaw 前馈使用 */
    /* 用实际底盘角速度做前馈，避免指令方向与实际轮速不一致 */
    float wz_ff = chassis_wz_actual_filtered;
    if (fabsf(wz_ff) < kChassisWzFFDeadband) {
      wz_ff = 0.0f;
    }
    float wz_scaled = wz_ff / kChassisWzLsb;
    if (wz_scaled > 127.0f) {
      wz_scaled = 127.0f;
    } else if (wz_scaled < -128.0f) {
      wz_scaled = -128.0f;
    }
    /* 四舍五入而不是直接截断，减小底盘 wz 前馈的量化误差 */
    const float wz_rounded =
        (wz_scaled >= 0.0f) ? (wz_scaled + 0.5f) : (wz_scaled - 0.5f);
    const uint8_t gimbal_wz_byte =
        static_cast<uint8_t>(static_cast<int8_t>(wz_rounded));

    dbg_gimbal_yaw_cmd = yaw_rate;
    dbg_gimbal_pitch_cmd = pitch;
    dbg_gimbal_cmd_count = dbg_gimbal_cmd_count + 1u;

    /* counter 高 2 位编码模式，低 6 位继续计数，供云台选择不同前馈增益 */
    const uint8_t gimbal_mode_code = static_cast<uint8_t>(mode) & 0x03u;
    const uint8_t gimbal_counter_with_mode = static_cast<uint8_t>(
        (gimbal_counter++ & 0x3Fu) | (gimbal_mode_code << 6));

    /* enable 的 bit2:1 冗余编码模式，云台可同时从 counter 和 enable 读取 */
    uint8_t gimbal_enable =
        (output_enable && !gimbal_estop) ? 1u : 0u;
    if (gimbal_enable != 0u)
    {
      gimbal_enable = static_cast<uint8_t>(gimbal_enable |
                                           (gimbal_mode_code << 1));
    }

    CAN_SendGimbalCmd(yaw_rate, pitch, gimbal_enable,
                      gimbal_estop ? 1u : 0u, gimbal_counter_with_mode,
                      gimbal_wz_byte);
  }

  /* 调试观测变量 */
  for (int i = 0; i < 4; ++i) {
    dbg_wheel_rpm[i] = wheel[i].rotorRpm();
    dbg_wheel_current[i] = wheel[i].current();
    dbg_wheel_temp[i] = wheel[i].temperature();
    dbg_wheel_online[i] = wheel[i].online() ? 1u : 0u;
    dbg_wheel_target[i] = wheel[i].targetRpm();
    dbg_wheel_cmd[i] = current_cmd[i];
  }

  tick++;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  if (htim == &htim6) {
    MainTask();
  }
}
uint8_t rx_data = 0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == &huart3) {
    if (Size == remote_control::kRcRxDataLen) {
      HAL_IWDG_Refresh(&hiwdg);
      rc_ptr->decode(rx_buf);
      last_rc_tick = tick;
      rc_valid = true;
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, kRxBufLen);
  }
}
