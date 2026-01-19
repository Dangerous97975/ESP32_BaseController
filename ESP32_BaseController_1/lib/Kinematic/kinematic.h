/**
 * @file kinematic.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-04-08
 *
 * @copyright Copyright (c) 2024
 *
 * *負責將 ROS2 輸入進來的線速度與角速度最終轉換成 PWM 與 Pulse
 */

#ifndef __KINEMATIC_H__
#define __KINEMATIC_H__

#include <Arduino.h>
#include "base_config.h"

class Kinematics
{
public:
    // 定義底盤運動學模型類型
    enum base
    {
        ACKERMAN,          // 0: 阿克曼轉向 (類汽車結構，前輪轉向後輪驅動)
        SELF_ROTATION,     // 1: 原地旋轉模式 (可能指特殊的獨立轉向輪模式)
        DIFFERENTIAL_DRIVE,// 差速驅動 (如掃地機器人，靠左右輪速差轉向)
        ROBOT_2WD2WS,      // 雙輪驅動 / 雙輪轉向 (2-Wheel Drive, 2-Wheel Steering)
    };

    enum base eBasePlatform;

    // 定義上層下達的運動指令結構 (Input)
    typedef struct MotionCommand
    {
        float linear_x;              // 線速度 (m/s)
        float center_rotation_rad;   // 轉向弧度或旋轉半徑 (依具體實作邏輯而定)
        float center_rotation_angle; // 轉向角度 (Degrees)
        short int turning_mode;      // 轉向模式旗標 (例如切換 平行移動/阿克曼/原地轉)
    } MCommand;

    MCommand mCommand;

    // 定義中間層的物理控制量 (Intermediate) - 尚未轉為硬體訊號
    typedef struct MotionControlData
    {
        struct
        {
            int BLDC_R; // 右輪目標 RPM
            int BLDC_L; // 左輪目標 RPM
        } rpm;
        struct
        {
            float STEPPER_R; // 右輪轉向目標角度 (Degrees)
            float STEPPER_L; // 左輪轉向目標角度 (Degrees)
        } angle;
    } MControl;

    // 定義底層硬體驅動參數 (Output) - 最終寫入暫存器的值
    typedef struct ConverterParameters
    {
        struct
        {
            int BLDC_R; // 右輪 PWM 值 (0 ~ 1023)
            int BLDC_L; // 左輪 PWM 值 (0 ~ 1023)
        } pwm;
        struct
        {
            long STEPPER_R; // 右輪步進馬達脈衝數 (Steps)
            long STEPPER_L; // 左輪步進馬達脈衝數 (Steps)
        } pulse;
    } CP;

    // 建構子：初始化車輛幾何參數
    Kinematics(base robot_base, float motor_max_vel, int wheelDiameter, float wheelBase, float track);
    
    // 逆運動學解算主程式 (Inverse Kinematics Solver)
    // 輸入：目標速度、轉角 -> 輸出：硬體 PWM 與 Pulse
    CP inverseKinematics(float linear_x, float center_rotation_rad, int center_rotation_angle, short int turning_mode);

private:
    float _iMaxRPM;             // 馬達最大轉速 (RPM)
    float _fWheelBase;          // 軸距 (mm)
    float _fTrack;              // 輪距 (mm)
    float _degree2pulse;        // 角度轉脈衝係數 (Steps per Degree)
    int _max_pwm = pow(2, PWM_RESOLUTION) - 1; // PWM 最大值 (飽和限制用)
    double _fWheelCircumference;// 輪胎周長 (用於 m/s 轉 RPM)

    // TODO:把傳出也改成用傳址的方式 (優化建議：改用 Pointer 或 Reference 傳遞以節省記憶體複製)
    
    // 各種運動學模型的具體實作
    MControl ackerman(MCommand *mCommand);     // 阿克曼幾何解算
    MControl selfRotation(MCommand *mCommand); // 原地旋轉解算
    
    // 單位轉換器：將物理量 (RPM, Angle) 轉換為電子訊號 (PWM, Pulse)
    CP baseOrderConvert(MControl *mControl);
};

#endif