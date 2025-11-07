#include "PID_v1.h"
#include <Arduino.h>
#include "motor_controller.h"

MotorController::MotorController() :
    encoder(0),
    pid(&measuredRPM, &pidPWM, &targetRPM, 0, 0, 0, DIRECT),
    kp(0), ki(0), kd(0),
    pidPWM(0),
    targetRPM(0),
    measuredRPM(0),
    maxRPM(0),
    encoderPPR(0),
    encPrev(0)
{
}

void MotorController::init(float encoderPPR, float kp, float ki, float kd, float maxRPM) {
    this->encoderPPR = encoderPPR;
    this->maxRPM = maxRPM;
    this->ticksPerMicroSecToRPM = 1e6 * 60.0 / this->encoderPPR;
    tickSampleTimePrev = 0;

    float pidPeriod = 0.03;
    updatePeriodUs = (unsigned int) round(pidPeriod * 1e6);

    setPIDConfig(kp, ki, kd);
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-1, 1);
}

void MotorController::setPWMCallback(SetPWMCallback set_pwm_callback) {
  this->pwm_callback = set_pwm_callback;
}

void MotorController::setPWM(float pwm) {
  if (pwm_callback) {
    pwm_callback(this, pwm);
  }
}

void MotorController::resetEncoder() {
    encoder = 0;
}

void MotorController::setPIDConfig(float kp, float ki, float kd) {
    this->kp = kp; this->ki = ki; this->kd = kd;
    pid.SetTunings(kp, ki, kd);
}

void MotorController::setPIDkp(float kp) {
    this->kp = kp;
    pid.SetTunings(this->kp, this->ki, this->kd);
}

void MotorController::setPIDki(float ki) {
    this->ki = ki;
    pid.SetTunings(this->kp, this->ki, this->kd);
}

void MotorController::setPIDkd(float kd) {
    this->kd = kd;
    pid.SetTunings(this->kp, this->ki, this->kd);
}


bool MotorController::setTargetRPM(float rpm) {
    if (targetRPM == rpm)
    return false;

    bool within_limit = (abs(rpm) <= maxRPM);
    rpm = within_limit ? rpm : (rpm >= 0 ? maxRPM : -maxRPM);

    targetRPM = rpm;
    return within_limit;
}

float MotorController::getCurrentPWM() {
    return pidPWM;
}

float MotorController::getCurrentRPM() {
    return measuredRPM;
}

float MotorController::getTargetRPM() {
    return targetRPM;
}

long int MotorController::getEncoderValue() const {
    return encoder;
}

void MotorController::update() {
    unsigned long tickTime = micros();
    unsigned long tickTimeDelta = tickTime - tickSampleTimePrev;
    if(tickTimeDelta < updatePeriodUs)
        return;
    tickSampleTimePrev = tickTime;

    long int encNow = getEncoderValue();
    long int encDelta = encNow - encPrev;
    encPrev = encNow;

    float ticksPerMicroSec = ((float) encDelta) / ((float) tickTimeDelta);
    measuredRPM = ticksPerMicroSec * ticksPerMicroSecToRPM;
    
    pid.Compute();
    setPWM(pidPWM);
}