#pragma once
#include "esp32-hal-gpio.h"

class MotorEncoders {

  private:
    uint8_t _s1l, _s2l, _s1r, _s2r;   // Các chân sensor encoder của động cơ trái và phải (Left, Right)
    volatile long leftCount = 0;      // Biến đếm xung encoder trái
    volatile long rightCount = 0;     // Biến đếm xung encoder phải

    static MotorEncoders* _instance;  // Lưu con trỏ của instance cho hàm ngắt ngoài static

  public:
    MotorEncoders(uint8_t s1l = 255, uint8_t s2l = 255, uint8_t s1r = 255, uint8_t s2r = 255);

    // Hàm khai báo các chân encoder
    void init();

    // Hàm đếm xung 
    inline void IRAM_ATTR updateLeft();
    inline void IRAM_ATTR updateRight();

    // Hàm phục vụ ngắt ngoài
    static void IRAM_ATTR isrLeft();
    static void IRAM_ATTR isrRight();

    // Hàm getter
    long getLeftCount() const;
    long getRightCount() const;
};

