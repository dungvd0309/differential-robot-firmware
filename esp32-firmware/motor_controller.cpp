#include <PID_Timed.h>
#include <Arduino.h>
#include "motor_controller.h"

void MotorController::init(float encoderPPR, float kp, float ki, float kd, float maxRPM) {
    this->encoderPPR = encoderPPR;
    this->maxRPM = maxRPM;
    this->ticksPerMicroSecToRPM = 1e6 * 60.0 / this->encoderPPR;

    encoder = 0;
    targetRPM = 0;
    measuredRPM = 0;
    pidPWM = 0;
    encPrev = 0;
    tickSampleTimePrev = 0;

    float pidPeriod = 0.03;
    updatePeriodUs = (unsigned int) round(pidPeriod * 1e6);
    pid.Init(&measuredRPM, &pidPWM, &targetRPM, kp, ki, kd, pidPeriod, 0);

    pid.SetOutputLimits(-1, 1);
    pid.enable(true);
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
    pid.SetTunings(kp, ki, kd, 0);
}

void MotorController::setPIDkp(float kp) {
    this->kp = kp;
    setPIDConfig(this->kp, this->ki, this->kd);
}

void MotorController::setPIDki(float ki) {
    this->ki = ki;
    setPIDConfig(this->kp, this->ki, this->kd);
}

void MotorController::setPIDkd(float kd) {
    this->kd = kd;
    setPIDConfig(this->kp, this->ki, this->kd);
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

float MotorController::getPIDKp() {
  return pid.GetKpe();
}

float MotorController::getPIDKi() {
  return pid.GetKi();
}

float MotorController::getPIDKd() {
  return pid.GetKd();
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
    
    if (targetRPM == 0 && measuredRPM == 0) {
      // Prevent wheels from twitching or slowly turning after stop
      pid.clearErrorIntegral();
    }

    float sampleTime = tickTimeDelta * 1e-6;
    pid.Compute(sampleTime);
    setPWM(pidPWM);
}