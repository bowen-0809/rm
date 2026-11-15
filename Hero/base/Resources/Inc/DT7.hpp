/**
 *******************************************************************************
 * @file      : DT7.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2023-12-05      Caikunzhen      1. 完成测试
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REMOTE_CONTROL_DT7_HPP_
#define HW_COMPONENTS_DEVICES_REMOTE_CONTROL_DT7_HPP_

/* Includes ------------------------------------------------------------------*/
#include "system_user.hpp"

namespace hello_world
{
namespace devices
{
namespace remote_control
{
/* Exported macro ------------------------------------------------------------*/

enum SwitchState : uint8_t {
  kSwitchStateUp = 1u,
  kSwitchStateMid = 3u,
  kSwitchStateDown = 2u,
};
/* Exported constants --------------------------------------------------------*/

static const uint8_t kRcRxDataLen = 18u;
/* Exported types ------------------------------------------------------------*/

class DT7 
{
 public:
  DT7(){};
  virtual ~DT7() {}

  void decode(const uint8_t rx_data[kRcRxDataLen]);

  float rc_lv(void) const { return rc_lv_; }
  float rc_lh(void) const { return rc_lh_; }
  float rc_rv(void) const { return rc_rv_; }
  float rc_rh(void) const { return rc_rh_; }
  float rc_wheel(void) const { return rc_wheel_; }

  SwitchState rc_l_switch(void) const { return rc_l_switch_; }
  SwitchState rc_r_switch(void) const { return rc_r_switch_; }

  int16_t mouse_x(void) const { return mouse_x_; }
  int16_t mouse_y(void) const { return mouse_y_; }
  int16_t mouse_z(void) const { return mouse_z_; }

  bool mouse_l_btn(bool reset)
  {
    bool tmp = mouse_l_btn_;
    if (reset) {
      mouse_l_btn_ = false;
    }
    return tmp;
  }
  bool mouse_r_btn(bool reset)
  {
    bool tmp = mouse_r_btn_;
    if (reset) {
      mouse_r_btn_ = false;
    }
    return tmp;
  }
  bool key_W(bool reset = false)
  {
    bool tmp = key_.W;
    if (reset) {
      key_.W = false;
    }
    return tmp;
  }
  bool key_S(bool reset = false)
  {
    bool tmp = key_.S;
    if (reset) {
      key_.S = false;
    }
    return tmp;
  }
  bool key_A(bool reset = false)
  {
    bool tmp = key_.A;
    if (reset) {
      key_.A = false;
    }
    return tmp;
  }
  bool key_D(bool reset = false)
  {
    bool tmp = key_.D;
    if (reset) {
      key_.D = false;
    }
    return tmp;
  }
  bool key_SHIFT(bool reset = false)
  {
    bool tmp = key_.SHIFT;
    if (reset) {
      key_.SHIFT = false;
    }
    return tmp;
  }
  bool key_CTRL(bool reset = false)
  {
    bool tmp = key_.CTRL;
    if (reset) {
      key_.CTRL = false;
    }
    return tmp;
  }
  bool key_Q(bool reset = false)
  {
    bool tmp = key_.Q;
    if (reset) {
      key_.Q = false;
    }
    return tmp;
  }
  bool key_E(bool reset = false)
  {
    bool tmp = key_.E;
    if (reset) {
      key_.E = false;
    }
    return tmp;
  }
  bool key_R(bool reset = false)
  {
    bool tmp = key_.R;
    if (reset) {
      key_.R = false;
    }
    return tmp;
  }
  bool key_F(bool reset = false)
  {
    bool tmp = key_.F;
    if (reset) {
      key_.F = false;
    }
    return tmp;
  }
  bool key_G(bool reset = false)
  {
    bool tmp = key_.G;
    if (reset) {
      key_.G = false;
    }
    return tmp;
  }
  bool key_Z(bool reset = false)
  {
    bool tmp = key_.Z;
    if (reset) {
      key_.Z = false;
    }
    return tmp;
  }
  bool key_X(bool reset = false)
  {
    bool tmp = key_.X;
    if (reset) {
      key_.X = false;
    }
    return tmp;
  }
  bool key_C(bool reset = false)
  {
    bool tmp = key_.C;
    if (reset) {
      key_.C = false;
    }
    return tmp;
  }
  bool key_V(bool reset = false)
  {
    bool tmp = key_.V;
    if (reset) {
      key_.V = false;
    }
    return tmp;
  }
  bool key_B(bool reset = false)
  {
    bool tmp = key_.B;
    if (reset) {
      key_.B = false;
    }
    return tmp;
  }

 private:
  float rc_lv_;     ///* 遥控器左摇杆竖直值，[-1, 1]
  float rc_lh_;     ///* 遥控器左摇杆水平值，[-1, 1]
  float rc_rv_;     ///* 遥控器右摇杆竖直值，[-1, 1]
  float rc_rh_;     ///* 遥控器右摇杆水平值，[-1, 1]
  float rc_wheel_;  ///* 遥控器拨轮值，[-1, 1]

  SwitchState rc_l_switch_;  ///* 遥控器左拨杆值
  SwitchState rc_r_switch_;  ///* 遥控器右拨杆值

  bool mouse_l_btn_;  ///* 鼠标左键是否按下
  bool mouse_r_btn_;  ///* 鼠标右键是否按下
  int16_t mouse_x_;   ///* 鼠标x轴数值
  int16_t mouse_y_;   ///* 鼠标y轴数值
  int16_t mouse_z_;   ///* 鼠标z轴数值

  union {
    uint16_t data;
    struct {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    };
  } key_;  ///* 键盘按键
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace remote_control
}  // namespace devices
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REMOTE_CONTROL_DT7_HPP_ */
