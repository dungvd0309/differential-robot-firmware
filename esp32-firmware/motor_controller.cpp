#include <Arduino.h>
#include "motor_controller.h"

MotorController::MotorController(uint8_t enL, uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4, uint8_t enR)
{
    _enL = enL;
    _in1 = in1;
    _in2 = in2;
    _in3 = in3;
    _in4 = in4;
    _enR = enR;
}

void MotorController::init()
{
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_in3, OUTPUT);
    pinMode(_in4, OUTPUT);

    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
    digitalWrite(_in3, LOW);
    digitalWrite(_in4, LOW);
}

void MotorController::movePWM(int leftPWM, int rightPWM)
{
    digitalWrite(_in1, leftPWM < 0 ? HIGH : LOW);
    digitalWrite(_in2, leftPWM < 0 ? LOW : HIGH);
    digitalWrite(_in3, rightPWM < 0 ? HIGH : LOW);
    digitalWrite(_in4, rightPWM < 0 ? LOW : HIGH);
    analogWrite(_enL, abs(leftPWM));
    analogWrite(_enR, abs(rightPWM));
}

void MotorController::movePWM(int leftPWM, int rightPWM, int minPWM)
{
    // Loại bỏ xung khiến động cơ không chạy được
    int realLeftPWM = (leftPWM == 0) ? 0 : map(abs(leftPWM), 0, 255, minPWM, 255);
    int realRightPWM = (rightPWM == 0) ? 0 : map(abs(rightPWM), 0, 255, minPWM, 255);
    digitalWrite(_in1, leftPWM < 0 ? HIGH : LOW);
    digitalWrite(_in2, leftPWM < 0 ? LOW : HIGH);
    digitalWrite(_in3, rightPWM < 0 ? HIGH : LOW);
    digitalWrite(_in4, rightPWM < 0 ? LOW : HIGH);
    analogWrite(_enL, realLeftPWM);
    analogWrite(_enR, realRightPWM);
}