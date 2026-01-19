/**
 * @file Base_controller.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-04-08
 *
 * *連接端口  processPWM(int pwm)
 * *負責定義 BLDC 馬達物件與底層硬體控制介面
 */

#ifndef __BASE_CONTROLLER_H__
#define __BASE_CONTROLLER_H__

#include <Arduino.h>
#include "base_config.h"
#include "motor_interface.h"

// BLDC (無刷馬達) 控制類別
// 繼承自 MotorInterface (通用馬達介面)，實作具體的控制邏輯
class BLDC : public MotorInterface
{
public:
    //* 物件建構子多載 (Constructor)
    // 參數定義：反轉旗標、PWM腳位、PWM通道(ESP32用)、方向腳位、死區補償值、PWM解析度
    BLDC(bool invert, short int pwm_pin, short int pwm_channel, short int rev_pin, int pwm_offset, int pwm_resolution);

    // 覆寫 (Override) 父類別的煞車函式
    void brake() override;

    // 處理 PWM 數值 (核心邏輯：加上 Offset 補償死區)
    int processPWM(int pwm);

private:
    // 硬體腳位與參數設定 (Hardware Abstraction)
    short int _pwm_pin;      // PWM 輸出腳位
    short int _pwm_channel;  // ESP32 LEDC PWM 通道
    short int _rev_pin;      // 方向控制腳位 (Reverse Pin)
    int _pwm_offset;         // PWM 起步補償值 (Deadband Compensation) - 克服靜摩擦力用
    int _pwm_resolution;     // PWM 解析度 (例如 10-bit = 1024 階)

protected:
    // 覆寫父類別的正反轉函式 (僅限內部或繼承類別呼叫)
    void forward(int pwm) override; // 正轉控制
    void reverse(int pwm) override; // 反轉控制
};

#endif