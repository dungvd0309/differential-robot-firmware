//Note: esp32 library v2.0.2
#include <stdio.h>
#include "ros_interface.h"
#include "motors.h"
#include "motor_encoders.h"
#include "motor_controller.h"
#include "robot_config.h"
#include <PID_v1.h>

CONFIG cfg;

String inputString = "";    // Biến lưu chuỗi serial nhận
bool stringComplete = false;

void setup()
{
  Serial.begin(115200); 
  setupEncoders();
  setupMotors();
  leftMotor.setPWM(255);
  rightMotor.setPWM(-255);

  // FreeRTOS
  // xTaskCreatePinnedToCore(ros_task, "ros", 8192, NULL, 2, NULL, 0);
  
}

void ros_task(void*) {
  ros_init(); 
  for(;;){ ros_spin_some(); vTaskDelay(pdMS_TO_TICKS(1)); }
}

void loop()
{
  Serial.print(leftMotor.getEncoderValue());
  Serial.print(' ');
  Serial.println(rightMotor.getEncoderValue());
  delay(20);

  // if (abs(Setpoint - Input) > 20) {
  //   myPID.SetTunings(Kp, Ki, Kd);
  // } else {
  //   myPID.SetTunings(Kp, Ki, 0);
  // }

  // // Tính PID
  // myPID.Compute();

  // if(abs(Setpoint) < 15) {
  //   Output = 0;
  // }

  // // Ánh xạ lại giá trị Output để loại bỏ vùng chết của động cơ
  // int motorPWM = (Output == 0) ? 0 : map(abs(Output), 0, 255, cfg.motor_driver_min_pwm, 255);

  // // Gửi tín hiệu PWM
  // // controller.movePWM((int)Output,(int)Output);
  // digitalWrite(IN1, Output < 0 ? HIGH : LOW);
  // digitalWrite(IN2, Output < 0 ? LOW : HIGH);
  // ledcWrite(channel, motorPWM);

  // // Gửi dữ liệu cho Serial Plotter
  // Serial.print("0:0 ");
  // Serial.print("Input:"); Serial.print(Input, 2); Serial.print(" ");
  // Serial.print("Setpoint:"); Serial.print(Setpoint, 2); Serial.print(" ");
  // Serial.print("Output:"); Serial.print(Output, 2); Serial.print(" ");
  // Serial.println();
  // // Kiểm tra có dữ liệu Serial mới nhập
  // if (stringComplete) {
  //   parseInput(inputString);
  //   inputString = "";
  //   stringComplete = false;
  // }
  // delay(20);

  // vTaskDelay(pdMS_TO_TICKS(1000)); 

  
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    // Nếu kết thúc dòng
    if (inChar == '\n') {
      stringComplete = true;
    }
    else {
      inputString += inChar;
    }
  }
}

// Hàm phân tích chuỗi serial dạng "<setpoint> <kp> <ki> <kd>"
void parseInput(String s) {
  // Tách chuỗi thành các phần tử
  float sp = 0, p = 0, i = 0, d = 0;
  int n = sscanf(s.c_str(), "%f %f %f %f", &sp, &p, &i, &d);
  
  // if (n == 4) {
    // Setpoint = sp;
    // Kp = p;
    // Ki = i;
    // Kd = d;


    // Serial.print("Updated - Setpoint: ");
    // Serial.print(Setpoint);
    // Serial.print(" Kp: ");
    // Serial.print(Kp);
    // Serial.print(" Ki: ");
    // Serial.print(Ki);
    // Serial.print(" Kd: ");
    // Serial.println(Kd);
  // } else {
    // Serial.println("Invalid input format. Use:<setpoint> <Kp> <Ki> <Kd>");
  // }
}