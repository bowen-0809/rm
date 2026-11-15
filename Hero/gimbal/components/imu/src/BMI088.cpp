/**
 *******************************************************************************
 * @file      : BMI088.cpp
 * @brief     : BMI088 设备类
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2023-12-08      Caikunzhen      1. 完成第一版测试
 *  V1.0.1      2023-12-12      Caikunzhen      1. 添加旋转配置
 *  V1.1.0      2024-07-13      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. SPI 的波特率需小于 10MHz，CPOL=Low，CPHA=1Edge 或 CPOL=High，CPHA=2Edge，同
 *  时需配置好加速度计与陀螺仪的片选引脚
 *  2. 由于内部使用了硬件句柄，因此如果计划将实例作为全局变量时（全局变量初始化时对应的
 *  硬件句柄可能会还未初始化完毕），建议采取一下方法：
 *    1）声明指针，后续通过 `new` 的方式进行初始化
 *    2）声明指针，后续通过返回函数（CreateXXXIns）中的静态变量（因为该变量只有在第一
 *    次调用该函数时才会运行初始化程序）进行初始化
 *    3）使用无参构造函数，后续调用 `init` 方法进行初始化
 *    4）使用无参构造函数，后续使用拷贝赋值函数或是移动赋值函数进行初始化
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "BMI088.hpp"

#include "BMI088_reg.hpp"
#include "tick.hpp"
#include <cstring>
#include "spi.h"

namespace hello_world
{
namespace imu
{
/* Private macro -------------------------------------------------------------*/

/**
 * @brief       设置变量的某些位为 1
 * @param        data: 待设置的变量
 * @param        mask: 位掩码
 * @retval       None
 * @note        类型 T 必须支持位运算 |
 */
template <typename T1, typename T2>
inline void SetBits(T1& data, const T2& mask)
{
  data = T1(data | mask);
}
/* Private constants ---------------------------------------------------------*/

static const uint8_t kMaxMultiTransLen = 8u;
static const uint8_t kPaddingValue = 0x55;
static uint8_t kPaddingData[kMaxMultiTransLen] = {
    kPaddingValue,
    kPaddingValue,
    kPaddingValue,
    kPaddingValue,
    kPaddingValue,
    kPaddingValue,
    kPaddingValue,
    kPaddingValue,
};

static constexpr float kGravAcc = 9.7936f;       ///< 重力加速度（杭州）
static constexpr float kRad2DegCoff = 180 / PI;  ///< 弧度制转角度制系数
static constexpr float kDeg2RadCoff = PI / 180;  ///< 角度制转弧度制系数

static const float kAccSens[4] = {
    3 * kGravAcc / 32768,
    6 * kGravAcc / 32768,
    12 * kGravAcc / 32768,
    24 * kGravAcc / 32768,
};

static const float kGyroSens[5] = {
    2000 * kDeg2RadCoff / 32768,
    1000 * kDeg2RadCoff / 32768,
    500 * kDeg2RadCoff / 32768,
    250 * kDeg2RadCoff / 32768,
    125 * kDeg2RadCoff / 32768,
};

static const float kTempSens = 0.125f;
static const float kTempOffset = 23.0f;

static const uint8_t kWriteSign = 0x00;
static const uint8_t kReadSign = 0x80;

static const uint8_t kGyroRawDataLen = 6u;
static const uint8_t kAccRawDataLen = 6u;
static const uint8_t kTempRawDataLen = 2u;

static const uint8_t kSpiTimeout = 2u;  ///* 单位：ms

static const uint16_t kAccResetTimeMs = 2u;  ///* 加速度计重启耗时，单位：ms
static const uint8_t kGyroResetTimeMs = 1u;  ///* 陀螺仪重启耗时，单位：ms
static const uint16_t kCommWaitTimeUs = 2u;  ///* 通信间隔时间，单位：us
/** 寄存器写入后读取间隔时间，单位：us */
static const uint16_t kCheckWaitTimeUs = 120u;

/** 加速度计自检测短延时，单位：ms */
static const uint8_t kAccSelfTestShortDelayMs = 2u;
/** 加速度计自检测长延时，单位：ms */
static const uint8_t kAccSelfTestLongDelayMs = 50u;
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

BMI088::BMI088(const BMI088HWConfig &hw_config, const float rot_mat_flatten[9],
               const BMI088Config &config)
    : hspi_(hw_config.hspi),
      acc_cs_port_(hw_config.acc_cs_port),
      acc_cs_pin_(hw_config.acc_cs_pin),
      gyro_cs_port_(hw_config.gyro_cs_port),
      gyro_cs_pin_(hw_config.gyro_cs_pin),
      config_(config)
{
  memcpy(rot_mat_flatten_, rot_mat_flatten, sizeof(float) * 9);
  arm_mat_init_f32(&rot_mat_, 3, 3, rot_mat_flatten_);

  accCsH();
  gyroCsH();
}

BMI088::BMI088(const BMI088 &other)
    : hspi_(other.hspi_),
      acc_cs_port_(other.acc_cs_port_),
      acc_cs_pin_(other.acc_cs_pin_),
      gyro_cs_port_(other.gyro_cs_port_),
      gyro_cs_pin_(other.gyro_cs_pin_),
      config_(other.config_)
{
  memcpy(rot_mat_flatten_, other.rot_mat_flatten_, sizeof(float) * 9);
  arm_mat_init_f32(&rot_mat_, 3, 3, rot_mat_flatten_);

  accCsH();
  gyroCsH();
}

BMI088 &BMI088::operator=(const BMI088 &other)
{
  if (this != &other) {
    hspi_ = other.hspi_;
    acc_cs_port_ = other.acc_cs_port_;
    acc_cs_pin_ = other.acc_cs_pin_;
    gyro_cs_port_ = other.gyro_cs_port_;
    gyro_cs_pin_ = other.gyro_cs_pin_;
    config_ = other.config_;

    memcpy(rot_mat_flatten_, other.rot_mat_flatten_, sizeof(float) * 9);
    arm_mat_init_f32(&rot_mat_, 3, 3, rot_mat_flatten_);

    accCsH();
    gyroCsH();
  }

  return *this;
}

BMI088::BMI088(BMI088 &&other)
    : hspi_(other.hspi_),
      acc_cs_port_(other.acc_cs_port_),
      acc_cs_pin_(other.acc_cs_pin_),
      gyro_cs_port_(other.gyro_cs_port_),
      gyro_cs_pin_(other.gyro_cs_pin_),
      config_(other.config_)
{
  other.hspi_ = nullptr;
  other.acc_cs_port_ = nullptr;
  other.gyro_cs_port_ = nullptr;

  memcpy(rot_mat_flatten_, other.rot_mat_flatten_, sizeof(float) * 9);
  arm_mat_init_f32(&rot_mat_, 3, 3, rot_mat_flatten_);

  accCsH();
  gyroCsH();
}

BMI088 &BMI088::operator=(BMI088 &&other)
{
  if (this != &other) {
    hspi_ = other.hspi_;
    acc_cs_port_ = other.acc_cs_port_;
    acc_cs_pin_ = other.acc_cs_pin_;
    gyro_cs_port_ = other.gyro_cs_port_;
    gyro_cs_pin_ = other.gyro_cs_pin_;
    config_ = other.config_;

    other.hspi_ = nullptr;
    other.acc_cs_port_ = nullptr;
    other.gyro_cs_port_ = nullptr;

    memcpy(rot_mat_flatten_, other.rot_mat_flatten_, sizeof(float) * 9);
    arm_mat_init_f32(&rot_mat_, 3, 3, rot_mat_flatten_);

    accCsH();
    gyroCsH();
  }

  return *this;
}

void BMI088::init(const BMI088HWConfig &hw_config,
                  const float rot_mat_flatten[9],
                  const BMI088Config &config)
{
  hspi_ = hw_config.hspi;
  acc_cs_port_ = hw_config.acc_cs_port;
  acc_cs_pin_ = hw_config.acc_cs_pin;
  gyro_cs_port_ = hw_config.gyro_cs_port;
  gyro_cs_pin_ = hw_config.gyro_cs_pin;
  config_ = config;

  memcpy(rot_mat_flatten_, rot_mat_flatten, sizeof(float) * 9);
  arm_mat_init_f32(&rot_mat_, 3, 3, rot_mat_flatten_);

  accCsH();
  gyroCsH();
}

BMI088ErrState BMI088::imuInit(bool self_test) const
{
  BMI088ErrState err_state = kBMI088ErrStateNoErr;

  SetBits(err_state, gyroInit(self_test));
  SetBits(err_state, accInit(self_test));

  return err_state;
}

void BMI088::getData(
    float acc_data[3], float gyro_data[3], float *temp_ptr) const
{
  float tmp_data[3];

  if (acc_data != nullptr) {
    tick::DelayUs(kCommWaitTimeUs);
    getAccData(tmp_data);
    arm_mat_vec_mult_f32(&rot_mat_, tmp_data, acc_data);
  }

  if (gyro_data != nullptr) {
    tick::DelayUs(kCommWaitTimeUs);
    getGyroData(tmp_data);
    arm_mat_vec_mult_f32(&rot_mat_, tmp_data, gyro_data);
  }

  if (temp_ptr != nullptr) {
    tick::DelayUs(kCommWaitTimeUs);
    *temp_ptr = getTemp();
  }
}

BMI088ErrState BMI088::accInit(bool self_test) const
{
  tick::DelayUs(kCommWaitTimeUs);
  accRead(BMI088_ACC_ACC_CHIP_ID);  // 假读取

  tick::DelayUs(kCommWaitTimeUs);
  if (accRead(BMI088_ACC_ACC_CHIP_ID) != BMI088_ACC_ACC_CHIP_ID_VALUE) {
    return kBMI088ErrStateAccNotFound;
  }

  if (self_test) {
    if (accSelfTest() != kBMI088ErrStateNoErr) {
      return kBMI088ErrStateAccSelfTestFailed;
    }
  }

  /* 软重启 */
  tick::DelayUs(kCommWaitTimeUs);
  accWrite(BMI088_ACC_ACC_SOFTRESET, BMI088_ACC_ACC_SOFTRESET_VALUE);
  HAL_Delay(kAccResetTimeMs);

  tick::DelayUs(kCommWaitTimeUs);
  accRead(BMI088_ACC_ACC_CHIP_ID);  // 假读取

  /* 配置一些寄存器 */
  uint8_t acc_conf = (config_.acc_osr << _BMI088_ACC_ACC_CONF_ACC_OSR_SHFITS) |
                     (config_.acc_odr << _BMI088_ACC_ACC_CONF_ACC_ODR_SHFITS);
  uint8_t acc_init_config[4][2] = {
      {BMI088_ACC_ACC_RANGE, config_.acc_range},
      {BMI088_ACC_ACC_CONF, acc_conf},
      {BMI088_ACC_ACC_PWR_CTRL, BMI088_ACC_ACC_PWR_CTRL_ON},
      {BMI088_ACC_ACC_PWR_CONF, BMI088_ACC_ACC_PWR_CONF_ACTIVE_MODE}};

  for (uint8_t i = 0; i < 4; i++) {
    tick::DelayUs(kCommWaitTimeUs);
    accWrite(acc_init_config[i][0], acc_init_config[i][1]);
    tick::DelayUs(kCheckWaitTimeUs);
    if (accRead(acc_init_config[i][0]) != acc_init_config[i][1]) {
      return kBMI088ErrStateAccConfigErr;
    }
  }

  return kBMI088ErrStateNoErr;
}

BMI088ErrState BMI088::gyroInit(bool self_test) const
{
  tick::DelayUs(kCommWaitTimeUs);
  if (gyroRead(BMI088_GYRO_GYRO_CHIP_ID) != BMI088_GYRO_GYRO_CHIP_ID_VALUE) {
    return kBMI088ErrStateGyroNotFound;
  }

  if (self_test) {
    if (gyroSelfTest() != kBMI088ErrStateNoErr) {
      return kBMI088ErrStateGyroSelfTestFailed;
    }
  }

  /* 软重启 */
  tick::DelayUs(kCommWaitTimeUs);
  gyroWrite(BMI088_GYRO_GYRO_SOFTRESET, BMI088_GYRO_GYRO_SOFTRESET_VALUE);
  HAL_Delay(kGyroResetTimeMs);

  /* 配置一些寄存器 */
  uint8_t gyro_bandwidth = config_.gyro_odr_fbw |
                           BMI088_GYRO_GYRO_BANDWIDTH_MUST_SET;
  uint8_t gyro_init_config[3][2] = {
      {BMI088_GYRO_GYRO_RANGE, config_.gyro_range},
      {BMI088_GYRO_GYRO_BANDWIDTH, gyro_bandwidth},
      {BMI088_GYRO_GYRO_LPM1, BMI088_GYRO_GYRO_LPM1_NORMAL}};

  for (uint8_t i = 0; i < 3; i++) {
    tick::DelayUs(kCommWaitTimeUs);
    gyroWrite(gyro_init_config[i][0], gyro_init_config[i][1]);
    tick::DelayUs(kCheckWaitTimeUs);
    if (gyroRead(gyro_init_config[i][0]) != gyro_init_config[i][1]) {
      return kBMI088ErrStateGyroConfigErr;
    }
  }

  return kBMI088ErrStateNoErr;
}

BMI088ErrState BMI088::accSelfTest(void) const
{
  /* 配置一些寄存器 */
  uint8_t acc_init_config[4][2] = {
      {BMI088_ACC_ACC_RANGE, BMI088_ACC_ACC_RANGE_ACC_RANGE_24G},
      {BMI088_ACC_ACC_CONF, BMI088_ACC_ACC_CONF_NORMAL |
                                BMI088_ACC_ACC_CONF_ACC_ODR_1600_HZ},
      {BMI088_ACC_ACC_PWR_CTRL, BMI088_ACC_ACC_PWR_CTRL_ON},
      {BMI088_ACC_ACC_PWR_CONF, BMI088_ACC_ACC_PWR_CONF_ACTIVE_MODE}};

  for (uint8_t i = 0; i < 4; i++) {
    tick::DelayUs(kCommWaitTimeUs);
    accWrite(acc_init_config[i][0], acc_init_config[i][1]);
    tick::DelayUs(kCheckWaitTimeUs);
    if (accRead(acc_init_config[i][0]) != acc_init_config[i][1]) {
      return kBMI088ErrStateAccSelfTestFailed;
    }
  }

  HAL_Delay(kAccSelfTestShortDelayMs);

  float pos_self_test_data[3], neg_self_test_data[3];

  /* 开始自检测 */
  accWrite(BMI088_ACC_ACC_SELF_TEST, BMI088_ACC_ACC_SELF_TEST_POSITIVE_SIGNAL);
  HAL_Delay(kAccSelfTestLongDelayMs);
  getAccData(pos_self_test_data);

  accWrite(BMI088_ACC_ACC_SELF_TEST, BMI088_ACC_ACC_SELF_TEST_NEGATIVE_SIGNAL);
  HAL_Delay(kAccSelfTestLongDelayMs);
  getAccData(neg_self_test_data);

  float min_diff_thres[3] = {kGravAcc, kGravAcc, 0.5 * kGravAcc};
  for (uint8_t i = 0; i < 3; i++) {
    if (pos_self_test_data[i] - neg_self_test_data[i] < min_diff_thres[i]) {
      return kBMI088ErrStateAccSelfTestFailed;
    }
  }

  return kBMI088ErrStateNoErr;
}

BMI088ErrState BMI088::gyroSelfTest(void) const
{
  /* 开始自检测 */
  tick::DelayUs(kCommWaitTimeUs);
  gyroWrite(BMI088_GYRO_GYRO_SELF_TEST, BMI088_GYRO_GYRO_SELF_TEST_TRIG_BIST);

  uint8_t test_result;

  /* 等待自检完成 */
  while (1) {
    tick::DelayUs(kCommWaitTimeUs);
    test_result = gyroRead(BMI088_GYRO_GYRO_SELF_TEST);

    if (test_result & BMI088_GYRO_GYRO_SELF_TEST_BIST_RDY) {
      if (test_result & BMI088_GYRO_GYRO_SELF_TEST_BIST_FAIL) {
        return kBMI088ErrStateGyroSelfTestFailed;
      } else {
        return kBMI088ErrStateNoErr;
      }
    }
  }
}

void BMI088::getGyroData(float gyro_data[3]) const
{
  uint8_t gyro_raw_data[kGyroRawDataLen];
  gyroMultiRead(BMI088_GYRO_RATE_X_LSB, kGyroRawDataLen, gyro_raw_data);

  for (uint8_t i = 0; i < 3; i++) {
    gyro_data[i] = kGyroSens[config_.gyro_range] *
                   static_cast<int16_t>(
                       gyro_raw_data[2 * i] | gyro_raw_data[2 * i + 1] << 8);
  }
}

void BMI088::getAccData(float acc_data[3]) const
{
  uint8_t acc_raw_data[kAccRawDataLen];
  accMultiRead(BMI088_ACC_ACC_X_LSB, kAccRawDataLen, acc_raw_data);

  for (uint8_t i = 0; i < 3; i++) {
    acc_data[i] = kAccSens[config_.acc_range] *
                  static_cast<int16_t>(
                      acc_raw_data[2 * i] | acc_raw_data[2 * i + 1] << 8);
  }
}

float BMI088::getTemp(void) const
{
  uint8_t temp_raw_data[kTempRawDataLen];
  accMultiRead(BMI088_ACC_TEMP_MSB, kTempRawDataLen, temp_raw_data);

  int16_t temp_raw =
      static_cast<uint16_t>((temp_raw_data[0] << _BMI088_ACC_TEMP_MSB_SHIFTS) |
                            (temp_raw_data[1] >> _BMI088_ACC_TEMP_LSB_SHIFTS));

  if (temp_raw > 1023) {
    temp_raw -= 2048;
  }

  return temp_raw * kTempSens + kTempOffset;
}

void BMI088::gyroWrite(uint8_t mem_addr, uint8_t value) const
{
  uint8_t tx_data[2] = {mem_addr |= kWriteSign, value};
  gyroCsL();
  HAL_SPI_Transmit(hspi_, tx_data, 2, kSpiTimeout);
  gyroCsH();
}

uint8_t BMI088::gyroRead(uint8_t mem_addr) const
{
  uint8_t rx_data[2], tx_data[2] = {mem_addr |= kReadSign, kPaddingValue};
  gyroCsL();
  HAL_SPI_TransmitReceive(hspi_, tx_data, rx_data, 2, kSpiTimeout);
  gyroCsH();

  return rx_data[1];
}

void BMI088::gyroMultiRead(
    uint8_t start_mem_addr, uint8_t len, uint8_t *rx_data) const
{
  gyroCsL();
  start_mem_addr |= kReadSign;
  HAL_SPI_Transmit(hspi_, &start_mem_addr, 1, kSpiTimeout);
  HAL_SPI_TransmitReceive(hspi_, kPaddingData, rx_data, len, kSpiTimeout);
  gyroCsH();
}

void BMI088::accWrite(uint8_t mem_addr, uint8_t value) const
{
  uint8_t pTxData[2] = {mem_addr |= kWriteSign, value};
  accCsL();
  HAL_SPI_Transmit(hspi_, pTxData, 2, kSpiTimeout);
  accCsH();
}

uint8_t BMI088::accRead(uint8_t mem_addr) const
{
  uint8_t rx_data[3], tx_data[3] = {
                          mem_addr |= kReadSign, kPaddingValue, kPaddingValue};
  accCsL();
  HAL_SPI_TransmitReceive(hspi_, tx_data, rx_data, 3, kSpiTimeout);
  accCsH();

  return rx_data[2];
}

void BMI088::accMultiRead(
    uint8_t start_mem_addr, uint8_t len, uint8_t *rx_data) const
{
  uint8_t tx_data[2] = {start_mem_addr |= kReadSign, kPaddingValue};

  accCsL();
  HAL_SPI_TransmitReceive(hspi_, tx_data, rx_data, 2, kSpiTimeout);
  HAL_SPI_TransmitReceive(hspi_, kPaddingData, rx_data, len, kSpiTimeout);
  accCsH();
}
/* Private function definitions ----------------------------------------------*/
}  // namespace imu
}  // namespace hello_world
