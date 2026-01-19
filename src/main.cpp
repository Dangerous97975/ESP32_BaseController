/*
 * @file main.cpp
 * @Jhih-Bin Huang (F112102103@nkust.edu.tw)
 * @brief ESP32 控制器 - 藍牙/MicroROS 二合一版本
 * @version 1.0
 * @date 2024-03-31
 *
 * @copyright Copyright (c) 2024
 *
 * 使用方法：
 * 要使用藍牙控制版本，將 MODE_SELECTION 設置為 0
 * 要使用 MicroROS 控制版本，將 MODE_SELECTION 設置為 1
 */

// ======= 選擇操作模式 (Compile-time Switch) =======
//這是編譯時期的開關 (Compile-time switch)。#define 為 1 時，
//只會編譯 MicroROS 相關程式碼，節省記憶體並避免藍牙干擾。部署到正式機器人上時，請務必確認此設定。
// 0 = 藍牙控制模式 (Standalone Bluetooth Control) - 用於除錯或手動遙控
// 1 = MicroROS 控制模式 (ROS 2 Serial Control) - 用於導航與自動駕駛
#define MODE_SELECTION 1
// ===============================================

#include <Arduino.h>
#include <FastAccelStepper.h>

#include "kinematic.h"
#include "base_config.h"
#include "Base_controller.h"

#define LED_PIN 2

// 通用變數
// 步進馬達控制器 (用於轉向 Steering)
FastAccelStepperEngine stepperEngine;
FastAccelStepper *R_Stepper = nullptr;
FastAccelStepper *L_Stepper = nullptr;

// 無刷馬達控制器 (用於驅動 Traction)
// 初始化時帶入腳位定義與 PWM 參數
BLDC bldcL_controller(false, BLDC_L_PWM, L_PWM_CHANNEL, BLDC_L_REV, PWM_OFFSET, PWM_RESOLUTION);
BLDC bldcR_controller(false, BLDC_R_PWM, R_PWM_CHANNEL, BLDC_R_REV, PWM_OFFSET, PWM_RESOLUTION);

// 運動學解算物件 (2WD + 2WS 架構)
Kinematics kinematics(
    Kinematics::ROBOT_2WD2WS,
    MAX_BLDC_vel,
    WHEEL_DIAMETER,
    WHEELBASE,
    TRACK);

// 通用函數：硬體初始化
void initialize_motors()
{
  // 初始化步進引擎
  stepperEngine.init();
  // 綁定脈衝腳位 (Step Pin)
  R_Stepper = stepperEngine.stepperConnectToPin(STEP_R);
  L_Stepper = stepperEngine.stepperConnectToPin(STEP_L);

  if (R_Stepper && L_Stepper)
  {
    // 設定方向腳位與賦能
    R_Stepper->setDirectionPin(DIR_R);
    R_Stepper->setAutoEnable(true);     // 移動時自動 Enable，停止時 Disable 以省電/降溫
    R_Stepper->setAcceleration(10000);  // 設置加速度 (steps/s^2)

    L_Stepper->setDirectionPin(DIR_L);
    L_Stepper->setAutoEnable(true);
    L_Stepper->setAcceleration(10000);  // 設置加速度
  }
}

// 核心運動控制函式
// 流程：輸入指令 -> 運動學解算 -> BLDC轉動 -> 步進馬達轉向
void moveBase(float linear_x, float center_rotation_rad, float center_rotation_angle, int turning_mode)
{
  int32_t right_steps;
  int32_t left_steps;

  // 1. 逆運動學解算：將速度與轉角需求轉換為 PWM 與 脈衝數
  Kinematics::CP req_convertPara = kinematics.inverseKinematics(
      linear_x,
      center_rotation_rad,
      center_rotation_angle,
      turning_mode);

  // 2. 執行驅動輪速度控制
  bldcL_controller.spin(req_convertPara.pwm.BLDC_L);
  bldcR_controller.spin(req_convertPara.pwm.BLDC_R);

  // 3. 執行轉向輪角度控制 (包含同步邏輯)
  if (R_Stepper && L_Stepper)
  {
    // 獲取當前步進馬達位置
    int32_t current_right_pos = R_Stepper->getCurrentPosition();
    int32_t current_left_pos = L_Stepper->getCurrentPosition();

    // 絕對位置計算：目標位置 - 當前位置 = 需移動步數
    if (current_right_pos == 0 && current_left_pos == 0)
    {
      // 若為初始狀態，直接設為目標
      right_steps = req_convertPara.pulse.STEPPER_R;
      left_steps = req_convertPara.pulse.STEPPER_L;
    }
    else
    {
      right_steps = req_convertPara.pulse.STEPPER_R - current_right_pos;
      left_steps = req_convertPara.pulse.STEPPER_L - current_left_pos;
    }

    // 步進馬達同步邏輯 (Synchronized Movement)
    // 為了讓左右輪同時到達目標角度，移動距離較短的一邊需要降速
    int32_t max_steps = max(abs(right_steps), abs(left_steps));
    float right_speed_ratio = (max_steps > 0) ? abs(right_steps) / (float)max_steps : 0;
    float left_speed_ratio = (max_steps > 0) ? abs(left_steps) / (float)max_steps : 0;

    uint32_t base_speed = 6000; // 基準最大速度（步/秒 Hz）
    
    // 依比例動態調整速度
    R_Stepper->setSpeedInHz(base_speed * right_speed_ratio);
    L_Stepper->setSpeedInHz(base_speed * left_speed_ratio);

    // 發送移動指令
    R_Stepper->moveTo(req_convertPara.pulse.STEPPER_R);
    L_Stepper->moveTo(req_convertPara.pulse.STEPPER_L);
  }
}

// =============== 藍牙控制模式 (Bluetooth Mode) ===============
#if MODE_SELECTION == 0

#include "XboxSeriesXControllerESP32_asukiaaa.hpp"

// 控制來源標誌
volatile uint8_t control_source = 0;

// 藍牙命令結構
typedef struct
{
  float linear_x;
  float center_rotate_angle;
  int8_t turning_mode;
} MotionCommand;

MotionCommand bluetooth_command = {0, 0, 0};

// Xbox 手把物件實例化
XboxSeriesXControllerESP32_asukiaaa::Core
    xboxController(XBOX_MAC_ADDR);

int8_t turning_mode = 0;
boolean gear_flag = true; // 換檔旗標 (防抖動/狀態鎖存用)
boolean vel_direction = true;

// 數值映射函式 (Map float)
float linear_mapping(float value, float in_min, float in_max, float out_min, float out_max)
{
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 藍牙邏輯主程式
void bluetooth_remote_control()
{
  // 若藍牙控制器有連線且有輸入，設定控制來源為藍牙
  if (xboxController.isConnected())
  {
    control_source = 1;
  }

  // 檢查按鍵A是否被按下 (使用按鍵A切換轉向模式：阿克曼/原地轉/平行)
  if (xboxController.xboxNotif.btnA == 1 && gear_flag == true)
  {
    turning_mode = !(turning_mode); // 切換模式 (0 <-> 1)
    gear_flag = false;              // 鎖定旗標，防止連點
  }
  else if (xboxController.xboxNotif.btnA == 0)
  {
    gear_flag = true;               // 放開按鍵後解鎖旗標
  }

  // 檢查按鍵B是否被按下 (作為緊急停止按鈕 E-Stop)
  if (xboxController.xboxNotif.btnB == 1)
  {
    // 緊急停止 - 將線速度和角度都設為0
    bluetooth_command.linear_x = 0;
    bluetooth_command.center_rotate_angle = 0;

    // 立即調用 moveBase 停止馬達
    moveBase(0, 0, 0, turning_mode);

    // 輸出停止信息
    Serial.println("緊急停止！");

    // 視覺提示：快速閃爍 LED
    for (int i = 0; i < 5; i++)
    {
      digitalWrite(LED_PIN, HIGH);
      delay(50);
      digitalWrite(LED_PIN, LOW);
      delay(50);
    }

    // 等待按鈕釋放以避免重複觸發 (Blocking Wait)
    while (xboxController.xboxNotif.btnB == 1)
    {
      xboxController.onLoop();
      delay(10);
    }

    return; // 跳過其餘的控制邏輯，維持停止狀態
  }

  // 處理油門 (RT) -> 線速度
  float_t linear_x = linear_mapping((float)xboxController.xboxNotif.trigRT, 0, 1023, 0, 1.1);

  // 處理換檔按鍵 (LB/RB) 切換前進/後退方向
  // 只有在停車時 (linear_x == 0) 才允許換向，保護齒輪箱
  if (xboxController.xboxNotif.btnLB == 1 && linear_x == 0)
  {
    vel_direction = false; // 後退
  }
  else if (xboxController.xboxNotif.btnRB == 1 && linear_x == 0)
  {
    vel_direction = true;  // 前進
  }

  if (vel_direction == true)
  {
    bluetooth_command.linear_x = linear_x;
  }
  else
  {
    bluetooth_command.linear_x = -linear_x;
  }

  // 處理左搖桿 (LHori) -> 轉向角度
  bluetooth_command.center_rotate_angle = linear_mapping((float_t)xboxController.xboxNotif.joyLHori, 0, 65535, -40, 40);
  bluetooth_command.turning_mode = turning_mode;

  // 僅輸出調試信息 (過濾掉零值雜訊)
  if (linear_x > 0.1 || abs(bluetooth_command.center_rotate_angle) > 5)
  {
    Serial.print("BT: ");
    Serial.print(bluetooth_command.linear_x);
    Serial.print(", ");
    Serial.println(bluetooth_command.center_rotate_angle);
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // 初始化馬達
  initialize_motors();

  // 初始化藍牙控制器
  Serial.println("初始化藍牙...");
  xboxController.begin();

  // 設定控制來源為藍牙
  control_source = 1;

  Serial.println("藍牙控制系統啟動完成");
  Serial.println("提示: 按 A 切換轉向模式，按 B 緊急停止");

  // 啟動時指示燈閃爍
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
}

void loop()
{
  // 當前時間
  unsigned long current_time = millis();

  // 處理藍牙控制器監聽
  xboxController.onLoop();
  bluetooth_remote_control();

  // 使用藍牙命令控制底盤
  moveBase(bluetooth_command.linear_x, 0, bluetooth_command.center_rotate_angle, bluetooth_command.turning_mode);

  // LED 指示燈心跳 (1Hz 閃爍)
  digitalWrite(LED_PIN, (current_time / 1000) % 2);

  // 定期顯示系統狀態 (每 5秒)
  static unsigned long last_status_time = 0;
  if (current_time - last_status_time >= 5000)
  {
    Serial.printf("系統狀態: 控制源=%d, 自由堆=%d\n",
                  control_source,
                  ESP.getFreeHeap());
    last_status_time = current_time;
  }

  // 避免 CPU 負載過高，讓出時間片給 IDLE task
  delay(10);
}

// =============== MicroROS 控制模式 (MicroROS Mode) ===============
#elif MODE_SELECTION == 1

#include <micro_ros_platformio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <customize_interface/msg/joy_motion_command.h>

// 定義 MicroROS 使用的 UART 腳位
#define RX1_PIN 9
#define TX1_PIN 10

// MicroROS 相關全域變數
rclc_support_t support;
rclc_executor_t executor;
rcl_node_t node;
rcl_allocator_t allocator;
rcl_subscription_t joy_motion_subscriber; // 訂閱者
customize_interface__msg__JoyMotionCommand joy_command; // 訊息容器

// 記錄變數 (用於超時保護)
unsigned long last_ros_cmd_time = 0;
unsigned long ros_message_count = 0;

// 返回值檢查巨集 (Error Handling)
#define RCCHECK(fn)                    \
  {                                    \
    rcl_ret_t temp_rc = fn;            \
    if ((temp_rc != RCL_RET_OK))       \
    {                                  \
      Serial.println("Error in " #fn); \
    }                                  \
  }
#define RCSOFTCHECK(fn)                  \
  {                                      \
    rcl_ret_t temp_rc = fn;              \
    if ((temp_rc != RCL_RET_OK))         \
    {                                    \
      Serial.println("Warning in " #fn); \
    }                                    \
  }

// 安全的消息回調函式 (Callback Function)
// 當收到 ROS 2 的 /joy_command 主題時觸發
void joy_motion_callback(const void *msgin)
{
  try
  {
    // 強制轉型為自定義訊息格式
    const customize_interface__msg__JoyMotionCommand *msg = (const customize_interface__msg__JoyMotionCommand *)msgin;

    // 執行移動命令
    moveBase(msg->linear_x, 0, msg->center_rotate_angle, msg->turning_mode);

    // 更新看門狗計時器 (Watchdog Reset)
    unsigned long current_time = millis();
    last_ros_cmd_time = current_time;
    ros_message_count++;

    // 僅在有動作時輸出 Debug 資訊
    if (msg->linear_x != 0 || abs(msg->center_rotate_angle) != 0)
    {
      Serial.print("MR: ");
      Serial.print(msg->linear_x);
      Serial.print(", ");
      Serial.print(msg->center_rotate_angle);
      Serial.print(", ");
      Serial.println(msg->turning_mode);
    }
  }
  catch (...)
  {
    Serial.println("錯誤: 消息處理異常");
  }
}

void setup()
{
  // 初始化 USB 除錯串口和 LED
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Serial.println("初始化 MicroROS 控制系統...");

  // 禁用軟體看門狗以避免 MicroROS 初始化過久導致重置
  disableCore0WDT();
  disableLoopWDT();

  // 初始化馬達
  initialize_motors();
  // Serial.println("馬達已初始化");

  // 設置 MicroROS 傳輸層 (使用 Serial2)
  Serial2.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);
  set_microros_serial_transports(Serial2);
  delay(2000); // 等待串口穩定

  // 初始化 ROS 2 資源分配器
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // 創建 ROS 2 節點 "Base_controller"
  RCCHECK(rclc_node_init_default(&node, "Base_controller", "", &support));

  // 創建訂閱者，監聽 "/joy_command" 話題
  RCCHECK(rclc_subscription_init_default(
      &joy_motion_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(customize_interface, msg, JoyMotionCommand),
      "/joy_command"));

  // 創建執行器 (Executor) 並添加訂閱者
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &joy_motion_subscriber,
      &joy_command,
      &joy_motion_callback,
      ON_NEW_DATA)); // 設定為收到新資料時觸發

  last_ros_cmd_time = millis();
}

void loop()
{
  // 處理 ROS 消息 (Spin)
  // 檢查是否有新訊息並執行 callback，超時設為 10ms
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  
  unsigned long current_time = millis();
  unsigned long time_since_last_cmd = current_time - last_ros_cmd_time;

  // 安全保護機制 (Safety Timeout)
  // 如果超過 2000ms 沒有收到 ROS 命令 (例如上位機當機或斷線)，強制停止
  if (time_since_last_cmd > 2000)
  {
    moveBase(0, 0, 0, 0); // 停車
  }
  else
  {
    digitalWrite(LED_PIN, HIGH); // 常亮表示連線正常且有收到命令
  }

  // 短暫延遲，讓 ESP32 背景任務執行 (WiFi/Watchdog)
  yield();
}

#endif

