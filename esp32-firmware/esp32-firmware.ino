#include "motor_controller.h"
#include "motor_encoders.h"

#define S1L 4
#define S2L 5

MotorEncoders encoders(4, 5);

void setup()
{
  Serial.begin(115200);
  encoders.init();
}

int temp = 0;

void loop()
{
  
  temp = encoders.getLeftCount();
  Serial.println(temp);
  delay(50);
}