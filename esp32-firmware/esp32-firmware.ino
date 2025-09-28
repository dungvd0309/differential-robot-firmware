#include "motor_controller.h"
#include "motor_encoders.h"

#define S1L 4
#define S2L 5

// ref: JGA25-370 280RPM datasheet: https://precisionminidrives.com/product/25mm-gear-motor-with-encoder-48mm-type-model-nfp-jga25-370-en
const double GEAR_RATIO = 21.3 / 1; 

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