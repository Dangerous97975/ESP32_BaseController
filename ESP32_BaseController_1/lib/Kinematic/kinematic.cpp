#include "Arduino.h"
#include "kinematic.h"

// 建構子：初始化運動學參數
Kinematics::Kinematics(base robot_base, float motor_max_vel,
                       int wheelDiameter, float wheelBase,
                       float track) : eBasePlatform(robot_base),
                                      // 計算輪胎周長 (單位: 公尺) = PI * 直徑(吋) * 0.0254(吋轉公尺)
                                      _fWheelCircumference(PI * wheelDiameter * 0.0254),
                                      // 計算馬達最大轉速 (RPM)
                                      // 邏輯: 線速度(m/s) 轉為 輪子轉速(RPM) 再乘上 減速比
                                      _iMaxRPM(motor_max_vel * (60 / (PI * wheelDiameter * 0.0254)) * BLDC_GEAR_RATIO),
                                      // 設定軸距 (若是差速驅動則設為 0，避免除以零錯誤)
                                      _fWheelBase(eBasePlatform == DIFFERENTIAL_DRIVE ? 0 : wheelBase),
                                      _fTrack(track),
                                      // 角度轉脈衝係數 (由 Config 定義)
                                      _degree2pulse(DEG_TO_PUL) {};

//% 逆向運動學解算 (Inverse Kinematics)
// 根據輸入的線速度與轉向需求，計算各輪的目標狀態
Kinematics::CP Kinematics::inverseKinematics(
    float linear_x,
    float center_rotation_rad,
    int center_rotation_angle,
    short int turning_mode)
{
    MCommand mCommand;
    MControl mControl;
    CP converterParameters;

    // 填充指令結構
    mCommand.linear_x = linear_x;
    mCommand.center_rotation_rad = center_rotation_rad;
    mCommand.center_rotation_angle = center_rotation_angle;
    mCommand.turning_mode = turning_mode;

    // 根據模式選擇演算法
    if (mCommand.turning_mode == ACKERMAN)
        mControl = ackerman(&mCommand);      // 阿克曼轉向模式
    else if (mCommand.turning_mode == SELF_ROTATION)
        mControl = selfRotation(&mCommand);  // 原地旋轉模式

    // 將物理量 (RPM/Deg) 轉換為 電子訊號 (PWM/Pulse)
    converterParameters = baseOrderConvert(&mControl);

    return converterParameters;
}

//% 阿克曼轉向運動學 (Ackermann Steering Geometry)
// 定義：右轉為正 (Right turn is positive)
Kinematics::MControl Kinematics::ackerman(MCommand *mCommand)
{
    MControl mControl;

    float center_rotation_rad; // 旋轉半徑 (mm)
    float rotation_rad;        // 用於計算的半徑中間變數
    float theta_center;        // 虛擬中心轉向角 (Virtual center steering angle)
    float angle_R, angle_L;    // 左右輪轉向角
    float vel_R, vel_L;        // 左右輪線速度

    float linear_x_mm = mCommand->linear_x * 1000; // 單位轉換: m/s -> mm/s

    //@ Ackerman parameter calc (阿克曼參數計算)
    
    //* Case 1: 輸入為旋轉半徑 (Rotation Radius Input)
    if (mCommand->center_rotation_rad != 0 && mCommand->center_rotation_angle == 0)
    {
        center_rotation_rad = mCommand->center_rotation_rad * 1000;
        // 反推中心轉向角度 (幾何關係: sin(theta) = 輪距 / 2R)
        theta_center = asin(_fTrack / (2 * center_rotation_rad));
    }
    //* Case 2: 輸入為中心轉向角度 (Center Rotation Angle Input)
    else if (mCommand->center_rotation_rad == 0 && mCommand->center_rotation_angle != 0)
    {
        theta_center = mCommand->center_rotation_angle * DEG_TO_RAD;
        // 計算對應的旋轉半徑 R = (輪距 / 2) / sin(theta)
        center_rotation_rad = _fTrack / (2 * sin(theta_center));
        // 計算阿克曼幾何所需的投影半徑
        rotation_rad = _fTrack / (2 * tan(theta_center));
    }
    //* Case 3: 直線行駛 (Forward/Backward)
    else
    {
        // 直線行駛時，左右輪轉速相同，轉向角歸零
        mControl.rpm.BLDC_L = (linear_x_mm / 1000) * (60 / _fWheelCircumference) * BLDC_GEAR_RATIO;
        mControl.rpm.BLDC_R = (linear_x_mm / 1000) * (60 / _fWheelCircumference) * BLDC_GEAR_RATIO;
        // mControl.rpm.BLDC_R = 0; // 除錯用註解
        // mControl.rpm.BLDC_L = 0;
        mControl.angle.STEPPER_L = 0;
        mControl.angle.STEPPER_R = 0;
        return mControl;
    }

    // (註解掉的舊算法備份)
    // float rotation_rad = center_rotation_rad * cos(theta_center);
    // float theta_ack = atan(_fWheelBase / rotation_rad) * RAD_TO_DEG;

    // 取絕對值以處理幾何計算
    float abs_center_rotation_rad = abs(center_rotation_rad);
    rotation_rad = abs(rotation_rad);

    // 計算內外輪的旋轉半徑
    // R_short: 內側輪半徑 (轉彎半徑較小)
    // R_long:  外側輪半徑 (轉彎半徑較大)
    float R_short = rotation_rad - _fTrack / 2;
    float R_long = rotation_rad + _fTrack / 2;

    //@ Angle & Velocity Calculation (角度與速度差速運算)
    
    //* 右轉 (Right Turn)
    // 這裡使用 800mm 作為判定閥值，避免直線行駛時的數值抖動
    if (center_rotation_rad > 800)
    {
        // 阿克曼角公式: atan(軸距 / 輪半徑)
        angle_L = atan(_fWheelBase / R_long);   // 左輪 (外輪) 角度較小
        angle_R = atan(_fWheelBase / R_short);  // 右輪 (內輪) 角度較大

        // 差速計算: 外輪(L)路徑長，速度需較快；內輪(R)路徑短，速度需較慢
        vel_L = (R_long / cos(angle_L)) / abs_center_rotation_rad * linear_x_mm;
        vel_R = (R_short / cos(angle_R)) / abs_center_rotation_rad * linear_x_mm;
    } 
    //* 左轉 (Left Turn)
    else if (center_rotation_rad < -800)
    {
        // 左轉時，右輪為外輪，左輪為內輪 (角度取負值代表反向)
        angle_R = -atan(_fWheelBase / R_long);
        angle_L = -atan(_fWheelBase / R_short);

        // 差速計算
        vel_R = (R_long / cos(angle_R)) / abs_center_rotation_rad * linear_x_mm;
        vel_L = (R_short / cos(angle_L)) / abs_center_rotation_rad * linear_x_mm;
    }

    // 將計算出的弧度轉換為度數 (Rad -> Deg)
    mControl.angle.STEPPER_R = angle_R * RAD_TO_DEG;
    mControl.angle.STEPPER_L = angle_L * RAD_TO_DEG;

    // 將線速度 (mm/s) 轉換為 馬達轉速 (RPM)
    // 公式: (速度 m/s) * (60 / 輪周長) * 減速比
    mControl.rpm.BLDC_L = (vel_L / 1000) * (60 / _fWheelCircumference) * BLDC_GEAR_RATIO;
    mControl.rpm.BLDC_R = (vel_R / 1000) * (60 / _fWheelCircumference) * BLDC_GEAR_RATIO;

    return mControl;
}

//% 自轉運動學 (In-Place Rotation / Spin Turn)
// 適用於具備獨立轉向輪的載具 (2WD2WS)
Kinematics::MControl Kinematics::selfRotation(MCommand *mCommand)
{
    MControl mControl;
    float rotation_speed = mCommand->center_rotation_angle; // 這裡借用 angle 變數來存旋轉速度指令

    // 計算車輪需轉到的切線角度，形成圓形軌跡
    // atan(軸距 / 半輪距)
    mControl.angle.STEPPER_L = atan(_fWheelBase / (_fTrack / 2)) * RAD_TO_DEG;
    mControl.angle.STEPPER_R = -atan(_fWheelBase / (_fTrack / 2)) * RAD_TO_DEG;

    //* 逆時針旋轉 (Counter-Clockwise)
    if (rotation_speed > 10)
    {
        // 設定左右輪反向轉動 (數值可能經過實驗調校)
        mControl.rpm.BLDC_R = 1.0 * (60 / _fWheelCircumference) * BLDC_GEAR_RATIO;
        mControl.rpm.BLDC_L = -2 * (60 / _fWheelCircumference) * BLDC_GEAR_RATIO;

    } //* 順時針旋轉 (Clockwise)
    else if (rotation_speed < -10)
    {
        // 設定左右輪反向轉動
        mControl.rpm.BLDC_R =  -2 * (60 / _fWheelCircumference) * BLDC_GEAR_RATIO;
        mControl.rpm.BLDC_L = 1.0 * (60 / _fWheelCircumference) * BLDC_GEAR_RATIO;
    }
    // 停止 (Deadzone)
    else
    {
        mControl.rpm.BLDC_R = 0;
        mControl.rpm.BLDC_L = 0;
    }

    return mControl;
}

//% 硬體訊號轉換 (Signal Conversion)
// 將物理目標值 (RPM, Degree) 映射到硬體暫存器數值 (PWM, Pulse)
Kinematics::CP Kinematics::baseOrderConvert(MControl *mControl)
{
    CP converterParameters;

    // 角度轉換脈衝: 目標角度 / 每脈衝對應角度
    converterParameters.pulse.STEPPER_R = mControl->angle.STEPPER_R / _degree2pulse;
    converterParameters.pulse.STEPPER_L = mControl->angle.STEPPER_L / _degree2pulse;

    // RPM 轉換 PWM: 目標RPM * (最大PWM / 最大RPM) -> 線性映射
    converterParameters.pwm.BLDC_R = mControl->rpm.BLDC_R * (_max_pwm / _iMaxRPM);
    converterParameters.pwm.BLDC_L = mControl->rpm.BLDC_L * (_max_pwm / _iMaxRPM);

    return converterParameters;
}