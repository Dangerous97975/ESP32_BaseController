#ifndef __BASE_CONFIG_H__
#define __BASE_CONFIG_H__

//* STEPPER MOTOR CONSTANT
// 步進馬達參數 (通常用於轉向機構 Steering)
#define STEP_ANGLE 1.8          // 步進角 (度/步)
#define MICRO_STEP 16           // 驅動器細分設定 (Micro-stepping)
#define STEP_GEAR_RATIO 10      // 轉向機構減速比
// 計算每脈衝對應的角度 (Degrees per Pulse)
// 邏輯: 360度 / (馬達一圈原生步數 * 細分 * 減速比)
#define DEG_TO_PUL 360 / ((360 / STEP_ANGLE) * MICRO_STEP * STEP_GEAR_RATIO)

//* STEPPER MOTOR PINS
// 右側轉向馬達腳位 (方向 / 步進脈衝)
#define DIR_R 12
#define STEP_R 14

// 左側轉向馬達腳位 (方向 / 步進脈衝)
#define DIR_L 25
#define STEP_L 33

//* BLDC MOTOR PINS
// 無刷馬達 (BLDC) 參數 (通常用於驅動 Traction)
// 左/右輪 PWM 速度控制腳位
#define BLDC_L_PWM 26
#define BLDC_R_PWM 27

// 左/右輪 反轉訊號腳位 (Reverse Signal)
#define BLDC_L_REV 16
#define BLDC_R_REV 4

#define BLDC_GEAR_RATIO 4       // 驅動輪減速比

//* PWM channels
// ESP32 PWM 通道設定
#define L_PWM_CHANNEL 0
#define R_PWM_CHANNEL 1
#define PWM_RESOLUTION 10       // PWM 解析度 (10-bit, 0-1023)
#define PWM_OFFSET 550          // PWM 起步補償值 (Deadband Compensation) - 克服靜摩擦力用

//* Robot significant
// 車輛幾何與運動學參數 (Vehicle Kinematics)
#define TRACK 1120              // 輪距 (Track Width): 左右輪中心距離 (mm)
#define WHEELBASE 600           // 軸距 (Wheelbase): 前後軸中心距離 (mm)
#define WHEEL_DIAMETER 13       // 輪胎直徑 (單位: 英吋)
#define MAX_BLDC_vel 1.5        // 無刷馬達最大設計線速度 (m/s)

// 遙控手把 MAC 位址
#define XBOX_MAC_ADDR "3c:fa:06:08:2d:08"

// ROS2 通訊啟用旗標
#define ENABLE_MICRO_ROS false
#endif