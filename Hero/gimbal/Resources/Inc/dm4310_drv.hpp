#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__
#include "system_user.hpp"

#define MIT_MODE 0x000
#define POS_MODE 0x100
#define SPEED_MODE 0x200

// 达妙 DM4310 默认 MIT 映射范围：PMAX=12.5 rad，VMAX=30 rad/s，TMAX=10 Nm
// 如果调试助手里改过这三个参数，需要同步修改。
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

struct motor_fbpara_t {
  uint16_t id;
  uint16_t state;
  int p_int;
  int v_int;
  int t_int;

  float pos;
  float vel;
  float tor;

  float Tmos;
  float Tcoil;
};

struct Joint_Motor_t {
  uint16_t mode;
  motor_fbpara_t para;
};

extern void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,
                          uint32_t data_len);

extern void enable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                              uint16_t mode_id);
extern void disable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                               uint16_t mode_id);

// 关节电机
extern void mit_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos,
                     float vel, float kp, float kd, float torq);
extern void pos_speed_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                           float pos, float vel);
extern void speed_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float _vel);

extern void joint_motor_init(Joint_Motor_t *motor, uint16_t id, uint16_t mode);

extern float Hex_To_Float(uint32_t *Byte, int num); // 十六进制到浮点数
extern uint32_t FloatTohex(float HEX);              // 浮点数到十六进制转换

extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x_float, float x_min, float x_max, int bits);

#endif /* __DM4310_DRV_H__ */
