/*
 * Tasks/Task.cpp
 * GM6020 正弦速度/位置跟随  + IWDG 看门狗保护
 *
 * 控制结构 :
 *   速度正弦:  vel_ref(rpm) = A_v*sin(2π f_v t)  --[速度环]--> 电流指令
 *   位置正弦:  theta_ref = θ0 + A_p*sin(2π f_p t)
 *                          --[位置环]--> 目标转速(rpm) --[速度环]--> 电流指令
 *
 * 切换模式: 改全局 g_mode (Ozone 里可直接改)
 *   3 = 位置正弦跟随 (串级)
 *   2 = 速度正弦跟随
 *   1 = 开环, 固定电流 openloop_curr (调试/验证转向用)
 */

#include "main.h"
#include "can.h"
#include "iwdg.h"
#include "tim.h"
#include "Gm6020.h"

#include <cmath>

#define TWO_PI      6.283185307179586f
#define DEG_PER_RAD (180.0f / 3.14159265358979f)

#ifndef GM6020_ID
#define GM6020_ID   1       // 拨码 ID (1~7)
#endif

// 电流控制帧标识符: ID 1~4 → 0x1FE, ID 5~7 → 0x2FE
#define GM6020_TX_ID    ((GM6020_ID <= 4) ? 0x1FEu : 0x2FEu)

#define CURR_MAX        16384.0f   // 电流给定硬限幅 (协议上限, 对应 ±3A)
#define CURR_LIMIT       8800.0f   // 速度环电流限幅 ≈ 额定电流 1.62A (1.62/3*16384≈8847)

struct Pid {
    float kp, ki, kd;
    float out_max;      // 输出限幅
    float i_max;        // 积分累加器限幅
    float integ;        // 积分累加器
    float last_meas;    // 上一拍测量值 (用于对测量值微分)
    uint8_t started;
};

static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static float Pid_Step(Pid *p, float err, float meas, float dt)
{
    if (!p->started) { p->last_meas = meas; p->started = 1u; }

    p->integ += err * dt;                              // 积分累加
    if (p->integ >  p->i_max) p->integ =  p->i_max;    // ★ 积分限幅
    if (p->integ < -p->i_max) p->integ = -p->i_max;

    float dmeas = (meas - p->last_meas) / dt;          // ★ 对测量值微分
    p->last_meas = meas;

    float out = p->kp * err + p->ki * p->integ - p->kd * dmeas;

    if (out >  p->out_max) out =  p->out_max;          // 输出限幅
    if (out < -p->out_max) out = -p->out_max;
    return out;
}

Gm6020 motor(GM6020_ID);

Pid speed_pid = { 20.0f, 2.6f, 0.065f, CURR_LIMIT, 200.0f, 0.0f, 0.0f, 0u };
Pid angle_pid = {  3.3f, 0.0f, 0.0f,          80.0f,  20.0f, 0.0f, 0.0f, 0u };

// 在线调参
volatile uint8_t g_mode = 0;      // 0=位置正弦  1=速度正弦  2=开环(固定电流)
float vel_amp_rpm = 150.0f;        // 速度正弦幅值 (rpm)
float vel_freq_hz = 0.5f;         // 速度正弦频率 (Hz)
float pos_amp_deg = 120.0f;        // 位置正弦幅值 (度)
float pos_freq_hz = 0.5f;         // 位置正弦频率 (Hz)
float openloop_curr = 1000.0f;    // 固定电流指令, 测试用

// 运行状态（出bug时监测）
struct MotorState {
    float target;      // 当前模式参考 (速度模式 rpm; 位置模式 deg)
    float meas;        // 当前模式反馈
    float angle_deg;   // 多圈角度 (度)
    float rpm;         // 转速 (rpm)
    float target_rpm;  // 速度环给定 (rpm)
    float current;     // 电流指令输出 (counts, ±16384 对应 ±3A)
    float temp;        // 电机温度
};
MotorState g_motor = {0};

//以下变量用于开机测试，其余情况不会用到
volatile uint32_t g_tick   = 0;   // 1ms 计数器, 每秒应涨 1000
volatile uint32_t rx_count = 0;   // 收到的反馈帧数
volatile uint32_t tx_drop  = 0;   // CAN 发送失败次数 (邮箱满)

// 轨迹记录: {ref, meas, out}, 每 10ms 一点, 共 200 点 (~2 秒)
struct TracePoint { float ref; float meas; float out; };
TracePoint g_trace[200];
volatile uint16_t g_trace_i = 0;

// 位置正弦: 上电锁位状态
static float   theta0_deg  = 0.0f;   // ★ 上电锁定的起始角度 (度)
static uint8_t hold_inited = 0u;
static uint8_t last_mode   = 0xFFu;

// CAN 收发缓冲
static uint8_t can1_tx_data[8] = {0};

// 配置 CAN1 FIFO0 过滤器, 放行 0x200~0x20F (覆盖 GM6020 反馈帧 0x205~0x20B)
extern "C" void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filt = {0};
    filt.FilterBank           = 0;
    filt.FilterMode           = CAN_FILTERMODE_IDMASK;
    filt.FilterScale          = CAN_FILTERSCALE_32BIT;
    filt.FilterIdHigh         = 0x40A0;   // 0x205 << 5
    filt.FilterIdLow          = 0x0000;
    filt.FilterMaskIdHigh     = 0xFE00;   // mask 0x7F0 << 5
    filt.FilterMaskIdLow      = 0x0000;
    filt.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filt.FilterActivation     = ENABLE;
    filt.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(hcan, &filt);
}

// 开启自动总线恢复 (Bus-Off 后自动回到 Active)
extern "C" void Can_EnableAutoRecovery(CAN_HandleTypeDef *hcan)
{
    hcan->Init.AutoBusOff = ENABLE;
    HAL_CAN_Init(hcan);
}

// 发送标准数据帧, 邮箱满时丢弃并计数
extern "C" void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg,
                             uint32_t id, uint8_t len)
{
    CAN_TxHeaderTypeDef txh = {0};
    uint32_t mailbox;
    txh.StdId = id;
    txh.IDE   = CAN_ID_STD;
    txh.RTR   = CAN_RTR_DATA;
    txh.DLC   = len;
    if (HAL_CAN_AddTxMessage(hcan, &txh, msg, &mailbox) != HAL_OK) {
        tx_drop = tx_drop + 1u;
    }
}

extern "C" void MainInit(void)
{
    CanFilter_Init(&hcan1);
    Can_EnableAutoRecovery(&hcan1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_TIM_Base_Start_IT(&htim6);

    // 调试时冻结 IWDG: 内核暂停(core halted)时看门狗停走,
    // Ozone 打断点/暂停不再复位; 运行时靠 1ms 回调喂狗。
    __HAL_DBGMCU_FREEZE_IWDG();
}

extern "C" void MainTask(void)
{
    g_tick = g_tick + 1u;
    HAL_IWDG_Refresh(&hiwdg);                 // ★ 看门狗: 每 1ms 喂一次

    float t   = (float)g_tick * 0.001f;
    float rpm = (float)motor.velRpm();
    float deg = motor.angle() * DEG_PER_RAD;

    // 切换模式时: 重新锁位 + 清 PID 积分/微分状态
    if (g_mode != last_mode) {
        last_mode   = g_mode;
        hold_inited = 0u;
        speed_pid.integ = 0.0f; speed_pid.started = 0u;
        angle_pid.integ = 0.0f; angle_pid.started = 0u;
    }

    // 下面的voltage是我自己搞错了，实际应该是电流指令，不要被误导
    if (g_mode == 1u) {
        // ---- 开环: 固定小电流 (调试/验证转向用, 忽略反馈) ----
        motor.setVoltage((int16_t)clampf(openloop_curr, -CURR_MAX, CURR_MAX));
        g_motor.target = 0.0f; g_motor.meas = rpm; g_motor.target_rpm = 0.0f;
    }
    else if (rx_count == 0u) {
        // 还没收到反馈帧: 输出 0, 避免上电乱动
        motor.setVoltage(0);
        g_motor.target = 0.0f; g_motor.meas = 0.0f; g_motor.target_rpm = 0.0f;
    }
    else if (g_mode == 2u) {
        // 速度正弦
        float wref = vel_amp_rpm * sinf(TWO_PI * vel_freq_hz * t);
        float v    = Pid_Step(&speed_pid, wref - rpm, rpm, 0.001f);
        motor.setVoltage((int16_t)clampf(v, -CURR_MAX, CURR_MAX));
        g_motor.target = wref; g_motor.meas = rpm; g_motor.target_rpm = wref;
    }
    else if (g_mode == 3u) {
        // 位置正弦
        if (!hold_inited) { theta0_deg = deg; hold_inited = 1u; }   // 上电只锁一次

        float dref = theta0_deg + pos_amp_deg * sinf(TWO_PI * pos_freq_hz * t);
        float wref = Pid_Step(&angle_pid, dref - deg, 0.0f, 0.001f); // 角度环 D=0 -> meas 传 0
        // 速度前馈: 位置参考的导数 (deg/s -> rpm: *60/360), 减小正弦跟踪的相位滞后
        wref += pos_amp_deg * TWO_PI * pos_freq_hz
                * cosf(TWO_PI * pos_freq_hz * t) * (60.0f / 360.0f);
        wref = clampf(wref, -angle_pid.out_max, angle_pid.out_max);

        float v = Pid_Step(&speed_pid, wref - rpm, rpm, 0.001f);
        motor.setVoltage((int16_t)clampf(v, -CURR_MAX, CURR_MAX));
        g_motor.target = dref; g_motor.meas = deg; g_motor.target_rpm = wref;
    }

    g_motor.angle_deg = deg;
    g_motor.rpm       = rpm;
    g_motor.current   = (float)motor.voltage();   // 实际发出的电流指令 (counts)
    g_motor.temp      = motor.temp();

    // 组帧发送 (每 1ms 一帧), 电流控制帧 0x1FE/0x2FE
    motor.encode(can1_tx_data);
    CAN_Send_Msg(&hcan1, can1_tx_data, GM6020_TX_ID, 8);

    // 每 10ms 记一次轨迹 (Ozone 看超调/跟随)
    static uint8_t div = 0;
    if (++div >= 10u) {
        div = 0u;
        g_trace[g_trace_i].ref  = g_motor.target;
        g_trace[g_trace_i].meas = g_motor.meas;
        g_trace[g_trace_i].out  = g_motor.current;
        g_trace_i = (uint16_t)((g_trace_i + 1u) % 200u);
    }
}

// HAL 回调: TIM6 1ms 中断

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        MainTask();
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) {
        CAN_RxHeaderTypeDef rxh;
        uint8_t data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxh, data) == HAL_OK) {
            if (rxh.IDE == CAN_ID_STD && rxh.StdId == motor.rxId()) {
                motor.decode(data);
                rx_count = rx_count + 1u;
            }
        }
    }
}
