#ifndef __MOTOR_INTERFACE_H__
#define __MOTOR_INTERFACE_H__

// 馬達控制介面 (抽象基底類別 Abstract Base Class)
// 目的：定義所有種類馬達 (BLDC, DC, Stepper) 的統一操作標準，實現多型 (Polymorphism)
class MotorInterface
{
    bool invert_; // 馬達旋轉方向反轉旗標

protected:
    // 純虛擬函式 (Pure Virtual Function)
    // 定義介面但不實作，強制繼承的子類別 (如 BLDC) 必須自行定義硬體如何 "正轉" 與 "反轉"
    virtual void forward(int pwm) = 0;
    virtual void reverse(int pwm) = 0;

public:
    // 建構子：初始化方向設定
    MotorInterface(int invert) : invert_(invert)
    {
    }

    // 純虛擬函式：強制子類別實作煞車邏輯
    virtual void brake() = 0;

    // 馬達轉動控制主函式 (對外統一接口)
    // 包含方向處理與死區過濾
    void spin(int pwm)
    {
        // 若硬體安裝方向相反，則在此處將控制訊號變號
        if (invert_)
            pwm *= -1;

        // 死區 (Deadzone) 邏輯與方向判斷
        // 閥值 100：避免 PWM 過低時馬達無法轉動只產生噪音 (Humming)
        // 注意：此數值應配合實際硬體特性調整
        if (pwm > 100)
            forward(pwm);       // 正轉
        else if (pwm < -100)
            reverse(pwm);       // 反轉
        else
            brake();            // 進入死區範圍則執行煞車
    }
};

#endif