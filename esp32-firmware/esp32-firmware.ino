//Note: esp32 library v2.0.2
#include "ros_interface.h"
#include "motor_encoders.h"
#include "motor_controller.h"
#include <PID_v1.h>

// 32 33 25 26 27 14 
// ===== KHAI BÁO CHÂN ===== //
#define ENA 32
#define IN1 33
#define IN2 25
#define IN3 26
#define IN4 27
#define ENB 14

#define S1L 18
#define S2L 5
#define S1R 17
#define S2R 16
// ===== END ===== //

// ===== KHAI BÁO HẰNG SỐ ===== //
const int freq = 20000;    // tần số PWM 20 kHz
const int channel = 0;     // kênh PWM (0-15)
const int resolution = 8;  // độ phân giải bit (8 bit cho giá trị từ 0-255)

// ref: JGA25-370 280RPM 
const double GEAR_RATIO = 20.4 / 1; // hệ số bánh răng của động cơ
const int DECODE_FACTOR = 1;        // hệ số giải mã xung A/B (x1, x2, x4), trong file motor_encoder đang là x1 (đếm RISING của S1)
const int MOTOR_CPR = 11;           // số xung trên 1 vòng encoder (COUNTS_PER_REVOLUTION) 
const double WHEEL_CPR = GEAR_RATIO * MOTOR_CPR * DECODE_FACTOR; // số xung trên 1 vòng trục (COUNTS_PER_REVOLUTION)
const int WHEEL_DIAMETER = 65;       // đường kính bánh xe (mm)
// ===== END ===== //

MotorController controller(ENA, IN1, IN2, IN3, IN4, ENB);
MotorEncoders encoders(S1L, S2L, S1R, S2R, WHEEL_CPR); 

double Setpoint = 0, Input = 0, Output = 0;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

String inputString = "";    // Biến lưu chuỗi serial nhận
bool stringComplete = false;

void setup()
{
  Serial.begin(115200); 
  encoders.init();
  controller.init();
  controller.movePWM(255,255);

  // Đặt tần số PWM cao để giảm tiếng động cơ
  // ledcDetachPin(ENA);
  ledcSetup(channel, freq, resolution);  // thiết lập kênh PWM với tần số và độ phân giải
  ledcAttachPin(ENA, channel);      // gán chân PWM cho kênh
  // ledcAttachChannel(ENA, freq, resolution, channel);

  // FreeRTOS
  xTaskCreatePinnedToCore(ros_task, "ros", 8192, NULL, 2, NULL, 0);
  
  // Serial.println("Angle(rad),Setpoint(rad),Output(PWM)");
  // inputString.reserve(50);

  // myPID.SetMode(AUTOMATIC);
  // myPID.SetOutputLimits(-255, 255);
}

void ros_task(void*) {
  ros_init();
  for(;;){ ros_spin_some(); vTaskDelay(pdMS_TO_TICKS(1)); }
}

void loop()
{
  // // Đọc encoder
  // Input = encoders.getLeftAngle();

  // // Tính PID
  // myPID.Compute();

  // // Gửi tín hiệu PWM
  // // controller.movePWM((int)Output,(int)Output);
  // digitalWrite(IN1, Output < 0 ? HIGH : LOW);
  // digitalWrite(IN2, Output < 0 ? LOW : HIGH);
  // ledcWrite(channel, abs((int)Output));

  // // Gửi dữ liệu cho Serial Plotter
  // Serial.print(Input, 2); Serial.print(",");
  // Serial.print(Setpoint, 2); Serial.print(",");
  // Serial.print(Output, 2); Serial.print(",");
  // Serial.println();
  // // Kiểm tra có dữ liệu Serial mới nhập
  // if (stringComplete) {
  //   parseInput(inputString);
  //   inputString = "";
  //   stringComplete = false;
  // }
  delay(20);

  // vTaskDelay(pdMS_TO_TICKS(1000)); 

  // FOR TESTING
  
  // Serial.print("count: ");
  // Serial.print(encoders.getLeftCount());
  // Serial.print(", pos: ");
  // Serial.print(encoders.getLeftAngle());
  // Serial.print(", vel: ");
  // Serial.print(encoders.getLeftAngularVelocity());
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