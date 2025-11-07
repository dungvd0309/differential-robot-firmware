#pragma once
#include "esp32-hal-gpio.h"
#include <PID_Timed.h>

class MotorController {
  public:
    typedef void (*SetPWMCallback)(MotorController*, float);
    

  protected: 
    volatile long int encoder;
    
    PID pid;
    float kp, ki, kd;
    double pidPWM; // pid output (-1 to 1)
    double targetRPM; // pid target
    double measuredRPM; // pid input
    float maxRPM;
    SetPWMCallback pwm_callback; 
    
    float encoderPPR;
    float ticksPerMicroSecToRPM;

    unsigned int updatePeriodUs;
    long int encPrev;
    unsigned long tickSampleTimePrev;

  public:
    // MotorController();
    void init(float encoderPPR, float kp, float ki, float kd, float maxRPM);
    void setPWMCallback(SetPWMCallback callback);
    void setPWM(float pwm);
    void update();
    void resetEncoder();
    void setPIDConfig(float kp, float ki, float kd);
    void setPIDkp(float kp);
    void setPIDki(float ki);
    void setPIDkd(float kd);
    bool setTargetRPM(float rpm);
    float getCurrentRPM();
    float getTargetRPM();
    float getCurrentPWM();
    float getPIDKp();
    float getPIDKi();
    float getPIDKd();
    long int getEncoderValue() const;

  public:
    void tickEncoder(bool increment) {
      if (increment) {
        encoder++;
      } else {
        encoder--;
      }
    }
    
};  