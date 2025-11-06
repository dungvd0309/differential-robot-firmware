//Note: esp32 library v2.0.2
#include <stdio.h>
#include "ros_interface.h"
#include "motor_encoders.h"
#include "motor_controller.h"
#include "robot_config.h"
#include <PID_v1.h>

CONFIG cfg;

// 32 33 25 26 27 14 
// ===== KHAI BÁO CHÂN ===== //
#define ENA 14
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33
#define ENB 32

#define S1L 16
#define S2L 17
#define S1R 5
#define S2R 18
// ===== END ===== //

// ===== KHAI BÁO HẰNG SỐ ===== //
const int channel = 0;     // kênh PWM (0-15)
// ===== END ===== //

MotorController controller(ENA, IN1, IN2, IN3, IN4, ENB);
MotorEncoders encoders(cfg.mot_left_enc_gpio_a_fg, cfg.mot_left_enc_gpio_b, cfg.mot_right_enc_gpio_a_fg, cfg.mot_right_enc_gpio_b, cfg.WHEEL_CPR, cfg.WHEEL_RADIUS); 

double Setpoint = 0, Input = 0, Output = 0;
double Kp = 0.6, Ki = 1.0, Kd = 0.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

String inputString = "";    // Biến lưu chuỗi serial nhận
bool stringComplete = false;

void setup()
{
  Serial.begin(115200); 
  encoders.init();
  controller.init();
  controller.movePWM(0,0);

  // Đặt tần số PWM cao để giảm tiếng động cơ
  ledcDetachPin(ENA);
  ledcSetup(channel, cfg.PWM_FREQUENCY, cfg.PWM_RESOLUTION);  // thiết lập kênh PWM với tần số và độ phân giải
  ledcAttachPin(ENA, channel);      // gán chân PWM cho kênh

  // FreeRTOS
  // xTaskCreatePinnedToCore(ros_task, "ros", 8192, NULL, 2, NULL, 0);
  
  // Serial.println("Angle(rad),Setpoint(rad),Output(PWM)");
  // inputString.reserve(50);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
}

void ros_task(void*) {
  ros_init(); 
  for(;;){ ros_spin_some(); vTaskDelay(pdMS_TO_TICKS(1)); }
}

void loop()
{
  // Đọc encoder
  encoders.updateVelocities();
  Input = encoders.getLeftRpm();

  if (abs(Setpoint - Input) > 20) {
    myPID.SetTunings(Kp, Ki, Kd);
  } else {
    myPID.SetTunings(Kp, Ki, 0);
  }

  // Tính PID
  myPID.Compute();

  if(abs(Setpoint) < 15) {
    Output = 0;
  }

  // Ánh xạ lại giá trị Output để loại bỏ vùng chết của động cơ
  int motorPWM = (Output == 0) ? 0 : map(abs(Output), 0, 255, cfg.motor_driver_min_pwm, 255);

  // Gửi tín hiệu PWM
  // controller.movePWM((int)Output,(int)Output);
  digitalWrite(IN1, Output < 0 ? HIGH : LOW);
  digitalWrite(IN2, Output < 0 ? LOW : HIGH);
  ledcWrite(channel, motorPWM);

  // Gửi dữ liệu cho Serial Plotter
  Serial.print("0:0 ");
  Serial.print("Input:"); Serial.print(Input, 2); Serial.print(" ");
  Serial.print("Setpoint:"); Serial.print(Setpoint, 2); Serial.print(" ");
  Serial.print("Output:"); Serial.print(Output, 2); Serial.print(" ");
  Serial.println();
  // Kiểm tra có dữ liệu Serial mới nhập
  if (stringComplete) {
    parseInput(inputString);
    inputString = "";
    stringComplete = false;
  }
  delay(20);

  // vTaskDelay(pdMS_TO_TICKS(1000)); 

  // FOR TESTING

  // encoders.updateVelocities();
  // Serial.print("count: ");
  // Serial.print(encoders.getLeftCount());
  // Serial.print("\t pos: ");
  // Serial.print(encoders.getLeftAngle());
  // Serial.print("\t rad/s: ");
  // Serial.print(encoders.getLeftAngularVelocity());
  // Serial.print("\t rpm: ");
  // Serial.print(encoders.getLeftRpm());
  // Serial.print("\t m/s: ");
  // Serial.print(encoders.getLeftLinearVelocity());
  // Serial.println();
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
  
  if (n == 4) {
    Setpoint = sp;
    Kp = p;
    Ki = i;
    Kd = d;

    myPID.SetTunings(Kp, Ki, Kd);

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