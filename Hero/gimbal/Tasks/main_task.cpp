/**
 *******************************************************************************
 * @file      : main_task.cpp
 * @brief     : 云台任务：Yaw/Pitch 关节控制 + IMU 读取 + 下板命令/心跳
 * @attention :
 *  1. 上板不再使用任何 USART。
 *  2. 下板通过 CAN1 0x7FE 发送 yaw_rate/pitch/enable/estop，同时作为 IWDG 心跳。
 *  3. Yaw 使用 DM4310 速度模式，可连续 360° 旋转；Pitch 使用 DM4340 MIT 位置控制。
 *******************************************************************************
 */

#include "main_task.hpp"
#include "system_user.hpp"

#include "HW_can.hpp"
#include "dm4310_drv.hpp"
#include "dm4340_drv.hpp"
#include "gimbal_cmd.hpp"
#include "imu_task.hpp"
#include "iwdg.h"

#include <cmath>

uint32_t tick = 0;

Joint_Motor_t yaw_motor;
DM4340 pitch_motor(&hcan2, 9);

volatile GimbalCmd gimbal_cmd = {};
volatile bool gimbal_cmd_valid = false;
volatile uint32_t gimbal_cmd_tick = 0;

/* 调试变量 */
volatile uint8_t dbg_cmd_valid = 0u;
volatile uint8_t dbg_enable = 0u;
volatile int16_t dbg_yaw_rate_mrad = 0;
volatile int16_t dbg_pitch_mrad = 0;
volatile float dbg_imu_yaw = 0.0f;
volatile float dbg_imu_pitch = 0.0f;
volatile float dbg_imu_roll = 0.0f;
volatile float dbg_yaw_pos = 0.0f;
volatile float dbg_yaw_imu_i = 0.0f;
volatile uint8_t dbg_yaw_static_hold = 0u;
volatile uint8_t dbg_gimbal_mode = 0u;
volatile float dbg_pitch_pos = 0.0f;
volatile float dbg_pitch_zero = 0.0f;
volatile float dbg_pitch_target = 0.0f;
volatile float dbg_pitch_slew = 0.0f;
volatile float dbg_pitch_err = 0.0f;
volatile float dbg_pitch_imu_des = 0.0f;
volatile float dbg_pitch_imu_err = 0.0f;
volatile float dbg_pitch_enc_des = 0.0f;
volatile float dbg_pitch_grav_trim = 0.0f;
volatile float dbg_pitch_torque = 0.0f;
volatile float dbg_pitch_vel = 0.0f;
volatile uint8_t dbg_pitch_online = 0u;
volatile uint8_t dbg_pitch_zero_ok = 0u;
volatile uint8_t dbg_pitch_enabled = 0u;
volatile uint8_t dbg_pitch_home_done = 0u;
volatile uint32_t dbg_can2_esr = 0u;
volatile uint8_t dbg_yaw_error = 0u;
/* DM 反馈 ERR：0=失能，1=使能正常；8~E 才是故障 */
volatile uint8_t dbg_pitch_error = 0u;

static bool pitch_zero_initialized = false;

/*
 * DM4340 输出轴是绝对编码器，位置在掉电后仍然有效。
 * 实测机械中点约为 -1.34 rad，直接把它固定为 Pitch 水平零位，
 * 这样无论上电时 Pitch 停在最高点还是最低点，目标都会回到中点。
 */
static constexpr float kPitchMidpoint = -1.34f;

/* 控制参数，先给保守值，现场再调 */
static constexpr float kYawKd = 2.5f;
static constexpr float kPitchKp = 180.0f;  // 提高保持刚度，减小停止下坠
static constexpr float kPitchKd = 4.0f;
static constexpr float kPitchSlewRate = 3.4f;  // rad/s，Pitch 目标斜坡
static constexpr float kPitchRateLimit = 0.65f; // rad/s，与底盘速度一致
static constexpr float kPitchCmdDeadband = 0.030f;
static constexpr float kPitchRateFilterAlpha = 0.10f;
static constexpr float kPitchImuKp = 0.80f;
static constexpr float kPitchImuKd = 0.05f;
static constexpr float kPitchImuDeadband = 0.005f;
static constexpr float kPitchImuToEncSign = -1.0f; // IMU Pitch 增大 -> 编码器目标减小
static constexpr float kPitchGravityKi = 4.0f;    // 静止重力补偿积分
static constexpr float kPitchGravityTrimMax = 0.15f; // rad，补偿目标限幅
static constexpr uint32_t kPitchHomeTicks = 1500;  // 上电后先回中 1.5 s
/*
 * 实测机械范围：
 *   最高点 dbg_pitch_pos ≈ -2.88 rad
 *   最低点 dbg_pitch_pos ≈ +0.20 rad
 * 取中点为水平零位：(-2.88 + 0.20) / 2 ≈ -1.34 rad。
 * 相对零位的机械限位约为 [-1.54, +1.54] rad；
 * 这里留约 0.09 rad（5°）余量，使用 [-1.45, +1.45]。
 */
static constexpr float kPitchMin = -1.50f;
static constexpr float kPitchMax = 1.04f;

/*
 * 实测绝对机械止点（DM 反馈坐标）：
 *   上止点 ≈ -2.88 rad，下止点 ≈ +0.20 rad。
 * 目标再做一次绝对限幅，留 0.04 rad 余量；这样即使零位 offset
 * 有偏差，也不会把 Pitch 目标送到机械止点里。
 */
static constexpr float kPitchAbsMin = -2.84f;
static constexpr float kPitchAbsMax = -0.30f;
static constexpr float kYawRateLimit = 7.5f;
static constexpr float kYawJointGearRatio = 4.0f;  // Yaw 电机 : 云台关节
static constexpr uint32_t kCmdTimeoutTicks = 100;  // 100 ms

/*
 * 混合控制：
 *   Yaw   = IMU 世界角外环 + DM4310 速度内环
 *   Pitch = 右摇杆角速度 + IMU Pitch 外环 + DM4340 编码器位置内环
 */
static constexpr bool kUseImuYaw = true;
static constexpr bool kUseImuPitch = true;
static constexpr float kYawImuKp = 1.00f;
static constexpr float kYawImuKpFast = 3.00f;      // 大误差快速追赶
static constexpr float kYawImuFastThreshold = 0.10f; // rad
static constexpr float kYawImuKd = 1.40f;  // 其他模式阻尼
static constexpr float kYawImuKdFollow = 0.50f; // 跟随模式单独降低 Kd
static constexpr float kYawImuKi = 0.00f;  // 先关闭积分，避免侧拨轮/小陀螺过补偿
static constexpr float kYawImuIMax = 0.35f;       // rad/s，积分限幅
static constexpr float kYawImuIErrMax = 0.15f;    // rad，大误差时不积分
static constexpr float kYawCmdDeadband = 0.010f;  // rad/s，忽略中位微小 Yaw 指令
static constexpr float kYawImuDeadband = 0.003f;  // rad，约 0.17°
static constexpr float kChassisWzLsb = 0.050f;    // reserved 字节中底盘 wz 的分辨率
static constexpr float kChassisWzFFDeadband = 0.060f; // 静止时屏蔽前馈量化抖动
static constexpr float kYawChassisFFSign = -1.0f; // 重新按实际底盘转向取反
static constexpr float kYawChassisFFGainSeparate = 0.30f; // 分离模式前馈
static constexpr float kYawChassisFFGainFollow = 1.00f;   // 跟随模式前馈
static constexpr float kYawChassisFFGainSpin = 0.30f;     // 小陀螺模式前馈
static float yaw_des = 0.0f;
static bool yaw_des_initialized = false;
static bool yaw_centered_prev = true;
static float last_yaw_err = 0.0f;
static float yaw_err_d_filtered = 0.0f;
static float yaw_err_integral = 0.0f;

/* 静止保持：无摇杆、底盘基本静止且误差较小时，关闭 IMU 外环微修正 */
static constexpr uint32_t kYawStaticEnterTicks = 300; // 300 ms
static constexpr float kYawStaticEnterErr = 0.020f;  // rad，约 1.15°
static constexpr float kYawStaticExitErr = 0.050f;   // rad，约 2.86°
static constexpr float kYawStaticChassisMax = 0.10f; // rad/s
static bool yaw_static_active = false;
static uint32_t yaw_static_ticks = 0;

static bool yaw_enabled = false;
static bool pitch_enabled = false;
static uint32_t last_pitch_enable_tick = 0;
static float pitch_target_slew = 0.0f;
static bool pitch_target_slew_init = false;
static float pitch_enc_des = 0.0f;
static float pitch_imu_des = 0.0f;
static bool pitch_imu_des_init = false;
static bool pitch_rate_centered_prev = true;
static float pitch_rate_filtered = 0.0f;
static float pitch_imu_err_last = 0.0f;
static float pitch_imu_d_filtered = 0.0f;
static float pitch_grav_trim = 0.0f;
static bool pitch_home_done = false;
static bool pitch_home_started = false;
static uint32_t pitch_home_start_tick = 0;

static float Clamp(float value, float min, float max)
{
  if (value < min)
  {
    return min;
  }
  if (value > max)
  {
    return max;
  }
  return value;
}

void MainInit(void)
{
  joint_motor_init(&yaw_motor, 3, MIT_MODE);

  CanFilter_Init(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  CanFilter_Init(&hcan2);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  /* 先完成 IMU 初始化，再使能电机，避免上电后一段无控制指令的使能状态 */
  ImuInit();

  /* 上电先使能 Yaw/Pitch，随后 MainTask 根据命令决定是否继续使能 */
  enable_motor_mode(&hcan1, 3, MIT_MODE);
  pitch_motor.enable();
  yaw_enabled = true;
  pitch_enabled = true;

  HAL_TIM_Base_Start_IT(&htim6);
}

void MainTask(void)
{
  /* IMU 更新：BMI088 是阻塞 SPI，当前放在 1 kHz 任务里，后续可移到主循环 */
  ImuUpdate();

  /* 第一次收到 Pitch 反馈时，把当前位置记为零位，避免上电突然往电机 0 位跑 */
  if (!pitch_zero_initialized && pitch_motor.online())
  {
    dbg_pitch_zero = kPitchMidpoint;
    pitch_zero_initialized = true;
  }

  const GimbalCmd cmd = [] {
    GimbalCmd value;
    value.yaw_rate_mrad = gimbal_cmd.yaw_rate_mrad;
    value.pitch_mrad = gimbal_cmd.pitch_mrad;
    value.enable = gimbal_cmd.enable;
    value.estop = gimbal_cmd.estop;
    value.counter = gimbal_cmd.counter;
    value.reserved = gimbal_cmd.reserved;
    return value;
  }();
  const bool cmd_ok =
      gimbal_cmd_valid && ((tick - gimbal_cmd_tick) < kCmdTimeoutTicks);
  const bool enable = cmd_ok && (cmd.enable != 0u) && (cmd.estop == 0u);
  /* counter 高 2 位或 enable 的 bit2:1：0 分离 / 1 跟随 / 2 小陀螺 */
  uint8_t chassis_mode = static_cast<uint8_t>((cmd.counter >> 6) & 0x03u);
  const uint8_t enable_mode =
      static_cast<uint8_t>((cmd.enable >> 1) & 0x03u);
  if (enable_mode != 0u)
  {
    chassis_mode = enable_mode;
  }

  dbg_gimbal_mode = chassis_mode;
  dbg_cmd_valid = cmd_ok ? 1u : 0u;
  dbg_enable = enable ? 1u : 0u;
  dbg_yaw_rate_mrad = cmd.yaw_rate_mrad;
  dbg_pitch_mrad = cmd.pitch_mrad;

  if (enable)
  {
    if (!yaw_enabled)
    {
      enable_motor_mode(&hcan1, 3, MIT_MODE);
      yaw_enabled = true;
    }

    float yaw_rate_cmd =
        Clamp(static_cast<float>(cmd.yaw_rate_mrad) * 0.001f,
              -kYawRateLimit, kYawRateLimit);
    if (fabsf(yaw_rate_cmd) < kYawCmdDeadband)
    {
      yaw_rate_cmd = 0.0f;
    }
    const float pitch_rate_raw =
        Clamp(static_cast<float>(cmd.pitch_mrad) * 0.001f,
              -kPitchRateLimit, kPitchRateLimit);
    const bool pitch_centered =
        (fabsf(pitch_rate_raw) < kPitchCmdDeadband);
    if (pitch_centered)
    {
      pitch_rate_filtered = 0.0f;
    }
    else
    {
      pitch_rate_filtered +=
          kPitchRateFilterAlpha * (pitch_rate_raw - pitch_rate_filtered);
    }
    float pitch_rate_cmd = pitch_rate_filtered;
    if (fabsf(pitch_rate_cmd) < kPitchCmdDeadband)
    {
      pitch_rate_cmd = 0.0f;
    }

    /*
     * 上电回中：第一次使能后先忽略遥控器 Pitch 输入 1.5 s，
     * 目标固定为水平中点，避免上电时 Pitch 停在最高点或被摇杆直接带走。
     */
    if (!pitch_home_done)
    {
      if (pitch_zero_initialized && pitch_motor.online())
      {
        if (!pitch_home_started)
        {
          pitch_home_start_tick = tick;
          pitch_home_started = true;
        }
        if ((tick - pitch_home_start_tick) >= kPitchHomeTicks)
        {
          pitch_home_done = true;
        }
      }
      else
      {
        pitch_home_started = false;
      }
    }

    float yaw_rate = yaw_rate_cmd;
    bool yaw_centered_now = false;
    float yaw_err_for_static = 0.0f;

    /*
     * Pitch 右摇杆角速度 + IMU Pitch 世界角闭环：
     * 1. 摇杆速度积分成 IMU 目标角；
     * 2. 同一速度前馈到编码器目标；
     * 3. IMU 角度误差经 P/D 后修正编码器目标；
     * 4. DM4340 继续用编码器位置内环保持和承重。
     */
    if (!pitch_home_done)
    {
      pitch_enc_des = 0.0f;
      pitch_imu_des = euler_angles[PITCH];
      pitch_imu_des_init = true;
      pitch_rate_centered_prev = pitch_centered;
      pitch_imu_err_last = 0.0f;
      pitch_imu_d_filtered = 0.0f;
    }
    else
    {
      if (!pitch_imu_des_init)
      {
        pitch_imu_des = euler_angles[PITCH];
        pitch_imu_des_init = true;
      }

      if (pitch_centered && !pitch_rate_centered_prev)
      {
        /* 松杆时锁住当前 IMU 角和编码器位置，立即停止 */
        pitch_imu_des = euler_angles[PITCH];
        pitch_enc_des = Clamp(pitch_motor.position() - dbg_pitch_zero,
                              kPitchMin, kPitchMax);
        pitch_imu_err_last = 0.0f;
        pitch_imu_d_filtered = 0.0f;
        pitch_rate_filtered = 0.0f;
      }

      if (!pitch_centered)
      {
        pitch_imu_des += pitch_rate_cmd * 0.001f;
        pitch_enc_des += kPitchImuToEncSign * pitch_rate_cmd * 0.001f;
      }

      float pitch_imu_err = pitch_imu_des - euler_angles[PITCH];
      if (fabsf(pitch_imu_err) < kPitchImuDeadband)
      {
        pitch_imu_err = 0.0f;
        pitch_imu_err_last = 0.0f;
        pitch_imu_d_filtered = 0.0f;
      }
      const float pitch_imu_d_raw =
          (pitch_imu_err - pitch_imu_err_last) / 0.001f;
      pitch_imu_err_last = pitch_imu_err;
      pitch_imu_d_filtered +=
          0.10f * (pitch_imu_d_raw - pitch_imu_d_filtered);

      const float pitch_imu_rate =
          kPitchImuKp * pitch_imu_err +
          kPitchImuKd * pitch_imu_d_filtered;
      pitch_enc_des += kPitchImuToEncSign * pitch_imu_rate * 0.001f;
      pitch_enc_des = Clamp(pitch_enc_des, kPitchMin, kPitchMax);

      /* 静止时用编码器位置误差积分补偿重力造成的稳态下坠 */
      if (pitch_centered && pitch_motor.online())
      {
        const float pitch_actual_rel =
            pitch_motor.position() - dbg_pitch_zero;
        pitch_grav_trim +=
            kPitchGravityKi * (pitch_enc_des - pitch_actual_rel) * 0.001f;
        pitch_grav_trim = Clamp(pitch_grav_trim,
                                -kPitchGravityTrimMax,
                                kPitchGravityTrimMax);
      }
      pitch_rate_centered_prev = pitch_centered;
    }

    if (kUseImuYaw)
    {
      if (!yaw_des_initialized)
      {
        yaw_des = euler_angles[YAW];
        yaw_des_initialized = true;
        yaw_centered_prev = (fabsf(yaw_rate_cmd) < kYawCmdDeadband);
      }

      /*
       * 长转后突然回中时，不继续使用“指令积分目标”，
       * 而是把当前 IMU 世界 Yaw 直接锁为新目标，避免长时间
       * 指令/实际误差累积，回中后出现大幅慢漂。
       */
      const bool yaw_centered =
          (fabsf(yaw_rate_cmd) < kYawCmdDeadband);
      if (yaw_centered && !yaw_centered_prev)
      {
        yaw_des = euler_angles[YAW];
        last_yaw_err = 0.0f;
        yaw_err_d_filtered = 0.0f;
        yaw_err_integral = 0.0f;
      }
      if (!yaw_centered)
      {
        yaw_des += yaw_rate_cmd * 0.001f;
        /* 主动转云台时不积分，避免积分器与摇杆指令对抗 */
        yaw_err_integral = 0.0f;
      }
      yaw_centered_prev = yaw_centered;

      float yaw_err = yaw_des - euler_angles[YAW];
      yaw_err = atan2f(sinf(yaw_err), cosf(yaw_err));
      yaw_centered_now = yaw_centered;
      if (fabsf(yaw_err) < kYawImuDeadband)
      {
        yaw_err = 0.0f;
        yaw_err_d_filtered = 0.0f;
        last_yaw_err = 0.0f;
      }

      const float yaw_err_d_raw =
          atan2f(sinf(yaw_err - last_yaw_err),
                 cosf(yaw_err - last_yaw_err)) /
          0.001f;
      last_yaw_err = yaw_err;
      yaw_err_d_filtered += 0.10f * (yaw_err_d_raw - yaw_err_d_filtered);
      yaw_err_for_static = yaw_err;

      /* 小误差时缓慢积分，消除底盘转动时云台的稳态跟随误差 */
      if (!yaw_centered || (fabsf(yaw_err) >= kYawImuIErrMax))
      {
        yaw_err_integral = 0.0f;
      }
      else
      {
        yaw_err_integral += kYawImuKi * yaw_err * 0.001f;
        yaw_err_integral = Clamp(yaw_err_integral, -kYawImuIMax, kYawImuIMax);
      }

      /* 大误差快速追赶，小误差保持低增益避免超调 */
      const float yaw_kp =
          (fabsf(yaw_err) > kYawImuFastThreshold) ? kYawImuKpFast
                                                  : kYawImuKp;
      const float yaw_kd =
          (chassis_mode == 1u) ? kYawImuKdFollow : kYawImuKd;
      yaw_rate = Clamp(yaw_rate_cmd + yaw_kp * yaw_err +
                           yaw_kd * yaw_err_d_filtered +
                           yaw_err_integral,
                       -kYawRateLimit, kYawRateLimit);
    }

    /* Pitch 继续使用 DM4340 编码器关节控制 */
    (void)kUseImuPitch;

    /* 底盘实际 wz 前馈：直接加到 Yaw 速度，不污染 yaw_des 目标 */
    float chassis_wz_ff =
        static_cast<float>(static_cast<int8_t>(cmd.reserved)) * kChassisWzLsb;
    if (fabsf(chassis_wz_ff) < kChassisWzFFDeadband)
    {
      chassis_wz_ff = 0.0f;
    }
    float chassis_ff_gain = kYawChassisFFGainSeparate;
    if (chassis_mode == 1u)
    {
      chassis_ff_gain = kYawChassisFFGainFollow;
    }
    else if (chassis_mode == 2u)
    {
      chassis_ff_gain = kYawChassisFFGainSpin;
    }
    /* 模式位异常时也按高转速兜底，避免小陀螺误用分离增益 */
    if (fabsf(chassis_wz_ff) > 2.5f)
    {
      chassis_ff_gain = kYawChassisFFGainSpin;
    }
    yaw_rate = Clamp(yaw_rate +
                         kYawChassisFFSign * chassis_ff_gain * chassis_wz_ff,
                     -kYawRateLimit, kYawRateLimit);

    /*
     * 静止保持：无摇杆、底盘基本静止且误差较小时，暂时关闭 IMU 外环微修正。
     * 这能避免 IMU 缓慢漂移或量化噪声持续驱动 Yaw 电机。
     */
    if (kUseImuYaw)
    {
      const bool static_ok =
          (chassis_mode != 1u) &&
          yaw_centered_now &&
          (fabsf(chassis_wz_ff) < kYawStaticChassisMax);
      const float static_err = fabsf(yaw_err_for_static);
      const float static_limit =
          yaw_static_active ? kYawStaticExitErr : kYawStaticEnterErr;

      if (!static_ok || (static_err > static_limit))
      {
        yaw_static_active = false;
        yaw_static_ticks = 0;
      }
      else if (!yaw_static_active)
      {
        if (yaw_static_ticks < kYawStaticEnterTicks)
        {
          yaw_static_ticks++;
        }
        if (yaw_static_ticks >= kYawStaticEnterTicks)
        {
          yaw_static_active = true;
        }
      }

      if (yaw_static_active)
      {
        yaw_rate = 0.0f;
      }
    }
    dbg_yaw_static_hold = yaw_static_active ? 1u : 0u;

    /* 右摇杆/IMU 输出的是云台关节速度，换到电机侧 */
    const float yaw_motor_rate = yaw_rate * kYawJointGearRatio;

    /* Yaw：MIT 速度控制，kp=0，kd 提供阻尼，可连续 360° 旋转 */
    mit_ctrl(&hcan1, 3, 0.0f, yaw_motor_rate, 0.0f, kYawKd, 0.0f);

    /*
     * Pitch：命令有效时先无条件使能，不要用“当前是否在线”作为使能前提。
     * 否则 DM4340 一旦在失能后停止反馈，online() 永远为 false，
     * 就再也不会重新使能，表现为位置数据一直不变、Pitch 掉在下限位。
     */
    if (!pitch_enabled)
    {
      pitch_motor.enable();
      pitch_enabled = true;
      last_pitch_enable_tick = HAL_GetTick();
    }
    else if (!pitch_motor.online() &&
             (HAL_GetTick() - last_pitch_enable_tick > 100u))
    {
      /* 反馈没恢复时每 100 ms 重发一次使能，防止使能帧丢失 */
      pitch_motor.enable();
      last_pitch_enable_tick = HAL_GetTick();
    }

    /*
     * 只有收到过反馈并完成零位捕获后才发 MIT 目标；
     * 暂时没有反馈时保持使能，等待 0x19 恢复。
     */
    if (!pitch_motor.online())
    {
      pitch_target_slew_init = false;
    }

    if (pitch_zero_initialized && pitch_motor.online())
    {
      float pitch_target = dbg_pitch_zero + pitch_enc_des + pitch_grav_trim;
      if (pitch_target < kPitchAbsMin)
      {
        pitch_target = kPitchAbsMin;
      }
      else if (pitch_target > kPitchAbsMax)
      {
        pitch_target = kPitchAbsMax;
      }

      /* 目标斜坡：上电时从当前位置平滑走到水平，避免大角度猛冲 */
      if (!pitch_target_slew_init)
      {
        pitch_target_slew = pitch_motor.position();
        pitch_target_slew_init = true;
      }
      const float step = kPitchSlewRate * 0.001f;
      if (pitch_target > pitch_target_slew + step)
      {
        pitch_target_slew += step;
      }
      else if (pitch_target < pitch_target_slew - step)
      {
        pitch_target_slew -= step;
      }
      else
      {
        pitch_target_slew = pitch_target;
      }

      dbg_pitch_target = pitch_target;
      dbg_pitch_slew = pitch_target_slew;
      dbg_pitch_err = pitch_target - pitch_motor.position();
      pitch_motor.mit(pitch_target_slew, 0.0f, kPitchKp, kPitchKd, 0.0f);
    }
  }
  else
  {
    if (yaw_enabled)
    {
      disable_motor_mode(&hcan1, 3, MIT_MODE);
      yaw_enabled = false;
    }
    if (pitch_enabled)
    {
      pitch_motor.disable();
      pitch_enabled = false;
    }
    pitch_target_slew_init = false;
    pitch_enc_des = 0.0f;
    pitch_grav_trim = 0.0f;
    pitch_imu_des_init = false;
    pitch_rate_centered_prev = true;
    pitch_rate_filtered = 0.0f;
    pitch_imu_err_last = 0.0f;
    pitch_imu_d_filtered = 0.0f;
    pitch_home_done = false;
    pitch_home_started = false;
    yaw_des_initialized = false;
    yaw_centered_prev = true;
    last_yaw_err = 0.0f;
    yaw_err_d_filtered = 0.0f;
    yaw_err_integral = 0.0f;
    yaw_static_active = false;
    yaw_static_ticks = 0;
    dbg_yaw_static_hold = 0u;
  }

  dbg_imu_yaw = euler_angles[YAW];
  dbg_imu_pitch = euler_angles[PITCH];
  dbg_imu_roll = euler_angles[ROLL];
  dbg_yaw_pos = yaw_motor.para.pos;
  dbg_yaw_imu_i = yaw_err_integral;
  dbg_pitch_imu_des = pitch_imu_des;
  dbg_pitch_imu_err = pitch_imu_des - euler_angles[PITCH];
  dbg_pitch_enc_des = pitch_enc_des;
  dbg_pitch_grav_trim = pitch_grav_trim;
  dbg_pitch_pos = pitch_motor.position();
  dbg_pitch_torque = pitch_motor.torque();
  dbg_pitch_vel = pitch_motor.velocity();
  dbg_pitch_online = pitch_motor.online() ? 1u : 0u;
  dbg_pitch_zero_ok = pitch_zero_initialized ? 1u : 0u;
  dbg_pitch_enabled = pitch_enabled ? 1u : 0u;
  dbg_pitch_home_done = pitch_home_done ? 1u : 0u;
  dbg_can2_esr = hcan2.Instance->ESR;
  dbg_yaw_error = static_cast<uint8_t>(yaw_motor.para.state);
  dbg_pitch_error = pitch_motor.error();

  tick++;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim6)
  {
    MainTask();
  }
}
