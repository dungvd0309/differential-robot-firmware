//Note: esp32 library v2.0.2
#include <stdio.h>
#include "ros_interface.h"
#include "motors.h"
#include "motor_encoders.h"
#include "motor_controller.h"
#include "robot_config.h"

CONFIG cfg;

String inputString = "";    // Biến lưu chuỗi serial nhận
bool stringComplete = false;

void setup()
{
  Serial.begin(115200); 
  setupEncoders();
  setupMotors();
  // Đặt tốc độ mục tiêu ban đầu cho PID, ví dụ 100 RPM
  leftMotor.setTargetRPM(100);
  // FreeRTOS
  // xTaskCreatePinnedToCore(ros_task, "ros", 8192, NULL, 2, NULL, 0);
  
}

void ros_task(void*) {
  ros_init(); 
  for(;;){ ros_spin_some(); vTaskDelay(pdMS_TO_TICKS(1)); }
}

void loop()
{
  leftMotor.update();
  // rightMotor.update();
  Serial.print(leftMotor.getEncoderValue());
  Serial.print(' ');
  Serial.print(leftMotor.getCurrentRPM());
  Serial.print("| ");
  Serial.print("Kp = ");
  Serial.print(leftMotor.getPIDKp(), 5);
  Serial.print(" Ki = ");
  Serial.print(leftMotor.getPIDKi(), 5);
  Serial.print(" Kd = ");
  Serial.println(leftMotor.getPIDKd(), 5);


  // // Gửi dữ liệu cho Serial Plotter
  Serial.print("0:0 ");
  Serial.print("Input:"); Serial.print(leftMotor.getCurrentRPM(), 2); Serial.print(" ");
  Serial.print("Setpoint:"); Serial.print(leftMotor.getTargetRPM(), 2); Serial.print(" ");
  Serial.print("Output:"); Serial.print(leftMotor.getCurrentPWM(), 2); Serial.print(" ");
  Serial.println();
  // // Kiểm tra có dữ liệu Serial mới nhập
  if (stringComplete) {
    parseInput(inputString);
    inputString = "";
    stringComplete = false;
  }

  // vTaskDelay(pdMS_TO_TICKS(1000)); 
  delay(10);
  
}

// Hàm phân tích chuỗi serial dạng "<setpoint> <kp> <ki> <kd>"
void parseInput(String s) {
  // Tách chuỗi thành các phần tử
  float sp = 0, p = 0, i = 0, d = 0;
  int n = sscanf(s.c_str(), "%f %f %f %f", &sp, &p, &i, &d);
  
  if (n == 4) {
    float Setpoint = sp;
    float Kp = p;
    float Ki = i;
    float Kd = d;

    leftMotor.setTargetRPM(Setpoint);
    leftMotor.setPIDConfig(Kp, Ki, Kd);


    Serial.print("Updated - Setpoint: ");
    Serial.print(Setpoint);
    Serial.print(" Kp: ");
    Serial.print(Kp);
    Serial.print(" Ki: ");
    Serial.print(Ki);
    Serial.print(" Kd: ");
    Serial.println(Kd);
  } else {
    Serial.println("Invalid input format. Use:<setpoint> <Kp> <Ki> <Kd>");
  }
}