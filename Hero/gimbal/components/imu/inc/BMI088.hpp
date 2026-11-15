/**
 *******************************************************************************
 * @file      : BMI088.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_IMU_BMI088_HPP_
#define HW_COMPONENTS_DEVICES_IMU_BMI088_HPP_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "arm_math.h"

namespace hello_world
{
namespace imu
{
/* Exported macro ------------------------------------------------------------*/

enum BMI088ErrState : uint8_t {  ///* 通过或运算拼接
  kBMI088ErrStateNoErr = 0u,
  kBMI088ErrStateAccNotFound = 1u << 0,
  kBMI088ErrStateGyroNotFound = 1u << 1,
  kBMI088ErrStateAccSelfTestFailed = 1u << 2,
  kBMI088ErrStateGyroSelfTestFailed = 1u << 3,
  kBMI088ErrStateAccConfigErr = 1u << 4,
  kBMI088ErrStateGyroConfigErr = 1u << 5,
};

enum BMI088AccRange : uint8_t {  ///* BMI088 加速度计量程，单位：g
  kBMI088AccRange3G = 0x0,
  kBMI088AccRange6G = 0x1,
  kBMI088AccRange12G = 0x2,
  kBMI088AccRange24G = 0x3,
};

/** BMI088 加速度计输出频率（Output Data Rate），单位：Hz */
enum BMI088AccOdr : uint8_t {
  kBMI088AccOdr12_5 = 0x5,
  kBMI088AccOdr25 = 0x6,
  kBMI088AccOdr50 = 0x7,
  kBMI088AccOdr100 = 0x8,
  kBMI088AccOdr200 = 0x9,
  kBMI088AccOdr400 = 0xA,
  kBMI088AccOdr800 = 0xB,
  kBMI088AccOdr1600 = 0xC,
};

enum BMI088AccOsr : uint8_t {  ///* BMI088 加速度计过采样率（Oversampling Rate）
  kBMI088AccOsr4 = 0x8,        ///* 4倍过采样
  kBMI088AccOsr2 = 0x9,        ///* 2倍过采样
  kBMI088AccOsrNormal = 0xA,   ///* 不过采样
};

enum BMI088GyroRange : uint8_t {  ///* BMI088 陀螺仪量程，单位：°/s
  kBMI088GyroRange2000Dps = 0x0,
  kBMI088GyroRange1000Dps = 0x1,
  kBMI088GyroRange500Dps = 0x2,
  kBMI088GyroRange250Dps = 0x3,
  kBMI088GyroRange125Dps = 0x4,
};

/** BMI088 陀螺仪输出频率（Output Data Rate）与滤波器带宽（Filter Bandwidth），
 * 单位：Hz */
enum BMI088GyroOdrFbw : uint8_t {
  kBMI088GyroOdrFbw2000_532 = 0x0,
  kBMI088GyroOdrFbw2000_230 = 0x1,
  kBMI088GyroOdrFbw1000_116 = 0x2,
  kBMI088GyroOdrFbw400_47 = 0x3,
  kBMI088GyroOdrFbw200_23 = 0x4,
  kBMI088GyroOdrFbw100_12 = 0x5,
  kBMI088GyroOdrFbw200_64 = 0x6,
  kBMI088GyroOdrFbw100_32 = 0x7,
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

struct BMI088HWConfig {  ///* BMI088 硬件配置
  SPI_HandleTypeDef* hspi = nullptr;     ///* IMU 对应的 SPI 句柄的指针
  GPIO_TypeDef* acc_cs_port = nullptr;   ///* 加速度计片选端口
  uint32_t acc_cs_pin = GPIO_PIN_0;      ///* 加速度计片选引脚
  GPIO_TypeDef* gyro_cs_port = nullptr;  ///* 陀螺仪片选端口
  uint32_t gyro_cs_pin = GPIO_PIN_0;     ///* 陀螺仪片选引脚
};

struct BMI088Config {  ///* BMI088 设备配置
  BMI088AccRange acc_range = kBMI088AccRange3G;
  BMI088AccOdr acc_odr = kBMI088AccOdr1600;
  BMI088AccOsr acc_osr = kBMI088AccOsr4;
  BMI088GyroRange gyro_range = kBMI088GyroRange1000Dps;
  BMI088GyroOdrFbw gyro_odr_fbw = kBMI088GyroOdrFbw1000_116;
};

class BMI088
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  BMI088(void) = default;
  /**
   * @brief       BMI088 初始化
   * @param        hw_config: 硬件配置
   * @param        rot_mat_flatten: 旋转矩阵（展平）,
   *               [r00 r01 r02 r10 r11 r12 r20 r21 r22]，使用后可释放
   * @param        config: 设备配置
   * @retval       None
   * @note        None
   */
  BMI088(const BMI088HWConfig& hw_config, const float rot_mat_flatten[9],
         const BMI088Config& config = BMI088Config());
  BMI088(const BMI088& other);
  BMI088& operator=(const BMI088& other);
  BMI088(BMI088&& other);
  BMI088& operator=(BMI088&& other);

  virtual ~BMI088(void) {}

  /* 配置方法 */
  /**
   * @brief       BMI088 初始化，使用默认构造函数后请务必调用此函数
   * @param        hw_config: 硬件配置
   * @param        rot_mat_flatten: 旋转矩阵（展平）,
   *               [r00 r01 r02 r10 r11 r12 r20 r21 r22]，使用后可释放
   * @param        config: 设备配置
   * @retval       None
   * @note        None
   */
  void init(const BMI088HWConfig& hw_config, const float rot_mat_flatten[9],
            const BMI088Config& config = BMI088Config());

  /* 功能性方法 */

  /**
   * @brief       进行 BMI088 芯片配置
   * @param        self_test: 是否开启传感器自检测，自检测会额外消耗时间
   * @retval       错误状态，为以下值的或运算：
   *   @arg        kBMI088ErrStateAccNotFound: 加速度计未找到
   *   @arg        kBMI088ErrStateAccSelfTestFailed: 加速度计自检测失败
   *   @arg        kBMI088ErrStateAccConfigErr: 加速度计配置错误
   *   @arg        kBMI088ErrStateGyroNotFound: 陀螺仪未找到
   *   @arg        kBMI088ErrStateGyroSelfTestFailed: 陀螺仪自检测失败
   *   @arg        kBMI088ErrStateGyroConfigErr: 陀螺仪配置错误
   *   @arg        kBMI088ErrStateNoErr: 无错误
   * @note        该函数内部会阻塞运行，请不要在中断中调用
   */
  BMI088ErrState imuInit(bool self_test = false) const;

  /* 数据修改与获取 */

  /**
   * @brief       获取传感器数据
   * @param        acc_data: 加速度计三轴数据，[ax ay az]，单位：m/s^2
   * @param        gyro_data: 陀螺仪三轴数据，[wx wy wz]，单位：rad/s
   * @param        temp_ptr: 温度数据指针，单位：℃
   * @retval       None
   * @note        传入 null_ptr 则不获取对应数据
   */
  void getData(float acc_data[3], float gyro_data[3], float* temp) const;

 private:
  /* 功能性方法 */

  inline void accCsL(void) const
  {
    HAL_GPIO_WritePin(acc_cs_port_, acc_cs_pin_, GPIO_PIN_RESET);
  }
  inline void accCsH(void) const
  {
    HAL_GPIO_WritePin(acc_cs_port_, acc_cs_pin_, GPIO_PIN_SET);
  }
  inline void gyroCsL(void) const
  {
    HAL_GPIO_WritePin(gyro_cs_port_, gyro_cs_pin_, GPIO_PIN_RESET);
  }
  inline void gyroCsH(void) const
  {
    HAL_GPIO_WritePin(gyro_cs_port_, gyro_cs_pin_, GPIO_PIN_SET);
  }

  /**
   * @brief       加速度计配置
   * @param        self_test: 是否进行自检测
   * @retval       错误状态，为以下值的或运算：
   *   @arg        kBMI088ErrStateAccNotFound: 加速度计未找到
   *   @arg        kBMI088ErrStateAccSelfTestFailed: 加速度计自检测失败
   *   @arg        kBMI088ErrStateAccConfigErr: 加速度计配置错误
   *   @arg        kBMI088ErrStateNoErr: 无错误
   * @note        None
   */
  BMI088ErrState accInit(bool self_test) const;

  /**
   * @brief       陀螺仪配置
   * @param        self_test: 是否进行自检测，自检测会额外消耗时间
   * @retval       错误状态，为以下值的或运算：
   *   @arg        kBMI088ErrStateGyroNotFound: 陀螺仪未找到
   *   @arg        kBMI088ErrStateGyroSelfTestFailed: 陀螺仪自检测失败
   *   @arg        kBMI088ErrStateGyroConfigErr: 陀螺仪配置错误
   *   @arg        kBMI088ErrStateNoErr: 无错误
   * @note        None
   */
  BMI088ErrState gyroInit(bool self_test) const;

  /**
   * @brief       加速度计自检测
   * @retval       错误状态，可能为以下值：
   *   @arg        kBMI088ErrStateAccSelfTestFailed: 加速度计自检测失败
   *   @arg        kBMI088ErrStateNoErr: 无错误
   * @note        None
   */
  BMI088ErrState accSelfTest(void) const;

  /**
   * @brief       陀螺仪自检测
   * @retval       错误状态，可能为以下值：
   *   @arg        kBMI088ErrStateGyroSelfTestFailed: 陀螺仪自检测失败
   *   @arg        kBMI088ErrStateNoErr: 无错误
   * @note        None
   */
  BMI088ErrState gyroSelfTest(void) const;

  /* 数据修改与获取 */

  /**
   * @brief       获取陀螺仪数据
   * @param        gyro_data: 陀螺仪三轴数据，[wx wy wz]，单位：rad/s
   * @retval       None
   * @note        None
   */
  void getGyroData(float gyro_data[3]) const;

  /**
   * @brief       获取加速度计数据
   * @param        acc_data: 加速度计三轴数据，[ax ay az]，单位：m/s^2
   * @retval       None
   * @note        None
   */
  void getAccData(float acc_data[3]) const;

  /**
   * @brief       获取温度数据
   * @retval       温度，单位：℃
   * @note        None
   */
  float getTemp(void) const;

  /**
   * @brief       向陀螺仪寄存器写入数据
   * @param        mem_addr: 寄存器地址
   * @param        value: 待写入数据
   * @retval       None
   * @note        None
   */
  void gyroWrite(uint8_t mem_addr, uint8_t value) const;

  /**
   * @brief       读取陀螺仪寄存器数据
   * @param        mem_addr: 寄存器地址
   * @retval       读取数据
   * @note        None
   */
  uint8_t gyroRead(uint8_t mem_addr) const;

  /**
   * @brief       陀螺仪多寄存器数据读取
   * @param        start_mem_addr: 寄存器起始地址
   * @param        len: 带读取数据长度
   * @param        rx_data: 读取得到的数据
   * @retval       None
   * @note        None
   */
  void gyroMultiRead(
      uint8_t start_mem_addr, uint8_t len, uint8_t* rx_data) const;

  /**
   * @brief       向加速度计寄存器写入数据
   * @param        mem_addr: 寄存器地址
   * @param        value: 待写入数据
   * @retval       None
   * @note        None
   */
  void accWrite(uint8_t mem_addr, uint8_t value) const;

  /**
   * @brief       读取加速度计寄存器数据
   * @param        mem_addr: 寄存器地址
   * @retval       读取数据
   * @note        None
   */
  uint8_t accRead(uint8_t mem_addr) const;

  /**
   * @brief       加速度计多寄存器数据读取
   * @param        start_mem_addr: 寄存器起始地址
   * @param        len: 带读取数据长度
   * @param        rx_data: 读取得到的数据
   * @retval       None
   * @note        None
   */
  void accMultiRead(
      uint8_t start_mem_addr, uint8_t len, uint8_t* rx_data) const;

  /* 硬件配置 */

  SPI_HandleTypeDef* hspi_ = nullptr;     ///* IMU 对应的 SPI 句柄的指针
  GPIO_TypeDef* acc_cs_port_ = nullptr;   ///* 加速度计片选端口
  uint32_t acc_cs_pin_ = GPIO_PIN_0;      ///* 加速度计片选引脚
  GPIO_TypeDef* gyro_cs_port_ = nullptr;  ///* 陀螺仪片选端口
  uint32_t gyro_cs_pin_ = GPIO_PIN_0;     ///* 陀螺仪片选引脚

  BMI088Config config_;
  float rot_mat_flatten_[9] = {1.0f, 0.0f, 0.0f,
                               0.0f, 1.0f, 0.0f,
                               0.0f, 0.0f, 1.0f};
  arm_matrix_instance_f32 rot_mat_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace imu
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_IMU_BMI088_HPP_ */
