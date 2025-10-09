//Note: esp32 library v2.0.2
#include "ros_interface.h"
#include "motor_encoders.h"
// #include "motor_controller.h"


// ===== KHAI BÁO CHÂN ===== //
#define ENA 12
#define IN1 13
#define IN2 14
#define IN3 25
#define IN4 26
#define ENB 27

#define S1L 22
#define S2L 23
// ===== END ===== //

// ===== KHAI BÁO HẰNG SỐ ===== //
// ref: JGA25-370 280RPM 
const double GEAR_RATIO = 20.4 / 1; // hệ số bánh răng của động cơ
const int DECODE_FACTOR = 1;        // hệ số giải mã xung A/B (x1, x2, x4), trong file motor_encoder đang là x1 (đếm RISING của S1)
const int MOTOR_CPR = 11;           // số xung trên 1 vòng encoder (COUNTS_PER_REVOLUTION) 
const double WHEEL_CPR = GEAR_RATIO * MOTOR_CPR * DECODE_FACTOR; // số xung trên 1 vòng trục (COUNTS_PER_REVOLUTION)
const int WHEEL_DIAMETER = 65;       // đường kính bánh xe (mm)
// ===== END ===== //
 
MotorEncoders encoders(S1L, S2L, 255, 255, WHEEL_CPR); 
// MotorController controller(ENA, IN1, IN2, IN3, IN4, ENB);

void setup()
{
  Serial.begin(921600); 
  encoders.init();
  // controller.init();
  // controller.movePWM(255,255);

  xTaskCreatePinnedToCore(ros_task, "ros", 8192, NULL, 2, NULL, 0);
}

void ros_task(void*) {
  ros_init();
  for(;;){ ros_spin_some(); vTaskDelay(pdMS_TO_TICKS(1)); }
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000)); 

  // FOR TESTING
  
  // Serial.print("count: ");
  // Serial.print(encoders.getLeftCount());
  // Serial.print(", pos: ");
  // Serial.print(encoders.getLeftAngle());
  // Serial.print(", vel: ");
  // Serial.print(encoders.getLeftAngularVelocity());
  // Serial.println();
}