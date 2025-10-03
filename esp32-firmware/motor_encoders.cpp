#include "motor_encoders.h"

MotorEncoders* MotorEncoders::_instance = nullptr;

MotorEncoders::MotorEncoders(uint8_t s1l, uint8_t s2l, uint8_t s1r, uint8_t s2r)
    : _s1l(s1l), _s2l(s2l), _s1r(s1r), _s2r(s2r)
{
    _instance = this; 
}

void MotorEncoders::init()
{
    // Khởi tạo ngắt ngoài cho việc đếm xung (-1 cho việc dùng 1 động cơ để test)
    if (_s1l != 255 && _s2l != 255)
    {
        pinMode(_s1l, INPUT_PULLUP);
        pinMode(_s2l, INPUT_PULLUP);
        attachInterrupt(_s1l, isrLeft, RISING);
    }
    if (_s1r != 255 && _s2r != 255)
    {
        pinMode(_s1r, INPUT_PULLUP);
        pinMode(_s2r, INPUT_PULLUP);
        attachInterrupt(_s1r, isrRight, RISING);
    }
}

void IRAM_ATTR MotorEncoders::updateLeft()
{
    if (digitalRead(_s2l) == HIGH) _left_count++; else _left_count--;
}

void IRAM_ATTR MotorEncoders::updateRight()
{
    if (digitalRead(_s2r) == HIGH) _right_count--; else _right_count++;
}

void IRAM_ATTR MotorEncoders::isrLeft()  { if (_instance) _instance->updateLeft(); }
void IRAM_ATTR MotorEncoders::isrRight() { if (_instance) _instance->updateRight(); }

long MotorEncoders::getLeftCount() const { return _left_count; }
long MotorEncoders::getRightCount() const{ return _right_count; }