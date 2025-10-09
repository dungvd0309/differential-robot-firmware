#pragma once
#include <esp32-hal-gpio.h>
#include <Arduino.h>

class MotorEncoders {

  private:
    static MotorEncoders* _instance;  // Lưu con trỏ của instance cho hàm ngắt ngoài static

    uint8_t _s1l, _s2l, _s1r, _s2r;   // Các chân sensor encoder của động cơ trái và phải (Left, Right)
    volatile long _left_count = 0;    // Biến đếm xung encoder trái
    volatile long _right_count = 0;   // Biến đếm xung encoder phải


    // Cac bien dung cho tinh toan van toc
    double _wheel_cpr;
    long _prev_left_count = 0;      // Biến đếm xung của lần tính vận tốc trước
    long _prev_right_count = 0;     // Biến đếm xung của lần tính vận tốc trước
    unsigned long _prev_update_time = 0; // Thời điểm lần tính vận tốc trước
    volatile double _left_angular_velocity = 0;  // rad/s
    volatile double _right_angular_velocity = 0; // rad/s
    
  public:
    MotorEncoders(uint8_t s1l = 255, uint8_t s2l = 255, uint8_t s1r = 255, uint8_t s2r = 255
                , double wheel_cpr = 11);

    // Hàm khai báo các chân encoder
    void init();

    // Hàm đếm xung 
    inline void IRAM_ATTR updateLeft();
    inline void IRAM_ATTR updateRight();

    // Hàm phục vụ ngắt ngoài
    static void IRAM_ATTR isrLeft();
    static void IRAM_ATTR isrRight();

    // Hàm getter hanh trinh dong co
    long getLeftCount() const { return _left_count; }
    long getRightCount() const { return _right_count; }
    double getLeftAngle() const { return (double)_left_count / _wheel_cpr * 2 * PI; } // rad
    double getRightAngle() const { return (double)_right_count / _wheel_cpr * 2 * PI; } // rad

    // Ham getter van toc dong co
    void updateVelocities();
    double getLeftAngularVelocity() const { return _left_angular_velocity; } // rad/s
    double getRightAngularVelocity() const { return _right_angular_velocity; } // rad/s
};
