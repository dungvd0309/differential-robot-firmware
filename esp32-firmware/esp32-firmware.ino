//Note: esp32 library v2.0.2
#include <stdio.h>
#include "ros_interface.h"
#include "motors.h"
#include "motor_controller.h"
#include "robot_config.h"

CONFIG cfg;

String inputString = "";    // Biến lưu chuỗi serial nhận
bool stringComplete = false;

void setup()
{
  Serial.begin(115200); 
  Serial.println("Setting up robot...");
  setupEncoders();
  setupMotors();

  // FreeRTOS
  xTaskCreatePinnedToCore(motor_task, "motor", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(ros_task, "ros", 10240, NULL, 2, NULL, 1); // Increased stack for ROS
  
}

void ros_task(void*) {
  ros_init(); 
  for(;;){ ros_spin_some(); vTaskDelay(pdMS_TO_TICKS(10)); } // Small delay to yield CPU
}

void motor_task(void*) {
  for(;;){ updateMotors(); vTaskDelay(pdMS_TO_TICKS(10)); } // Control loop at 100Hz
}

void loop()
{
}

// // Hàm phân tích chuỗi serial dạng "<setpoint> <kp> <ki> <kd>"
// void parseInput(String s) {
//   // Tách chuỗi thành các phần tử
//   float sp = 0, p = 0, i = 0, d = 0;
//   int n = sscanf(s.c_str(), "%f %f %f %f", &sp, &p, &i, &d);
  
//   if (n == 4) {
//     float Setpoint = sp;
//     float Kp = p;
//     float Ki = i;
//     float Kd = d;

//     // leftMotor.setTargetRPM(Setpoint);
//     // rightMotor.setTargetRPM(Setpoint);
//     setMotorsRPM(Setpoint, Setpoint);
//     // leftMotor.setPIDConfig(Kp, Ki, Kd);


//     Serial.print("Updated - Setpoint: ");
//     Serial.print(Setpoint);
//     Serial.print(" Kp: ");
//     Serial.print(Kp);
//     Serial.print(" Ki: ");
//     Serial.print(Ki);
//     Serial.print(" Kd: ");
//     Serial.println(Kd);
//   } else {
//     Serial.println("Invalid input format. Use:<setpoint> <Kp> <Ki> <Kd>");
//   }
// }