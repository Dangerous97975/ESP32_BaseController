#include "Base_controller.h"
#include <math.h>

// 建構子：初始化 BLDC 馬達物件、GPIO 與 PWM 通道
// @param invert: 是否反轉馬達方向
// @param pwm_pin: PWM 輸出腳位
// @param pwm_channel: ESP32 LEDC 通道 (0-15)
// @param rev_pin: 方向控制腳位 (Reverse Pin)
// @param pwm_offset: 死區/起步補償值
// @param pwm_resolution: PWM 解析度 (bits)
BLDC::BLDC(bool invert, short int pwm_pin, short int pwm_channel,
           short int rev_pin, int pwm_offset, int pwm_resolution) : MotorInterface(invert),
                                                                    _pwm_pin(pwm_pin),
                                                                    _pwm_channel(pwm_channel),
                                                                    _rev_pin(rev_pin),
                                                                    _pwm_offset(pwm_offset),
                                                                    _pwm_resolution(pwm_resolution)
{
    // 設定 GPIO 模式為輸出
    pinMode(_pwm_pin, OUTPUT);
    pinMode(_rev_pin, OUTPUT);

    // 設定 ESP32 LEDC PWM 參數
    // 頻率 5000Hz：適合大多數中低階 BLDC 驅動器 (過高可能導致開關損耗，過低可能有噪音)
    ledcSetup(_pwm_channel, 5000, _pwm_resolution);
    
    // 將 PWM 通道綁定到實體腳位
    ledcAttachPin(_pwm_pin, _pwm_channel);
    
    // 初始化輸出為 0 (安全起見，防止上電爆衝)
    ledcWrite(_pwm_channel, 0);
}

// 煞車/停止函式
void BLDC::brake()
{
    // 將 PWM 佔空比 (Duty Cycle) 設為 0
    // 注意：視驅動器邏輯而定，這可能是自由滑行 (Coasting) 或被動煞車
    ledcWrite(_pwm_channel, 0);
}

// 正轉控制
void BLDC::forward(int pwm)
{
    // 計算包含補償值的 PWM 並輸出，同時將方向腳位拉低 (LOW)
    ledcWrite(_pwm_channel, processPWM(pwm));
    digitalWrite(_rev_pin, LOW);
}

// 反轉控制
void BLDC::reverse(int pwm)
{
    // 計算包含補償值的 PWM 並輸出，同時將方向腳位拉高 (HIGH)
    ledcWrite(_pwm_channel, processPWM(pwm));
    digitalWrite(_rev_pin, HIGH);
}

// PWM 訊號後處理：死區補償與限幅 (Saturation)
int BLDC::processPWM(int pwm)
{
    // 若輸入值小於 0 (防呆或特殊控制邏輯)
    if (pwm < 0)
    {
        // 取絕對值並加上偏移量 (Offset) 以克服靜摩擦力 (Static Friction)
        pwm = abs(pwm) + _pwm_offset;
    }
    // 若輸入值大於 0 (正常驅動情況)
    else if (pwm > 0)
    {
        // 加上偏移量 (Deadband Compensation)
        // 這是為了讓馬達一收到指令就能越過死區開始轉動，改善低速響應
        pwm += _pwm_offset;
    }
    // 若 pwm == 0，則不加 Offset，保持 0 輸出 (避免靜止時馬達發熱或蠕動)

    // 飽和限制 (Saturation/Clamping)
    // 防止計算後的 PWM 值超過硬體解析度上限 (例如 10-bit 為 1023)
    if (pwm > (pow(2, _pwm_resolution) - 1))
        pwm = (pow(2, _pwm_resolution) - 1);

        return pwm;
}