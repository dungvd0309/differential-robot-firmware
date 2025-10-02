//TODO: tách file quản lý ros, freertos
#include "ros_interface.h"
#include "motor_controller.h"
#include "motor_encoders.h"
#include "rtos_tasks.h"

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
 
MotorEncoders encoders(S1L, S2L);
MotorController controller(ENA, IN1, IN2, IN3, IN4, ENB);

void setup()
{
  Serial.begin(115200);
  encoders.init();
  controller.init();
  controller.movePWM(255,255);

  start_tasks();
}

int temp = 0;

void loop()
{
  // if (Serial.available() > 0) {
  //   int v = Serial.parseInt(); 
  //   if (v != 0 || Serial.peek() == '0') { // phân biệt được số 0 hợp lệ
  //     controller.movePWM(v, v);
  //     Serial.print("Da nhan: ");
  //     Serial.println(v);
  //   }
  //   while (Serial.available() && Serial.read() != '\n') { /* flush line */ }
  // }
  
  temp = encoders.getLeftCount();
  // Serial.print(temp);
  // Serial.print(" | ");
  // Serial.println(temp / WHEEL_CPR);
  micro_ros_data_publish(temp);
  
}