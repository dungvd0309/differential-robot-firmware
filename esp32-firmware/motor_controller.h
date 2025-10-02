#pragma once
#include "esp32-hal-gpio.h"

class MotorController {

  private: 
    uint8_t _enL, _in1, _in2, _in3, _in4, _enR; // Các chân điều khiển động cơ của L298N

  public:
    MotorController(uint8_t enL, uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4, uint8_t enR);
    
    // Phương thức khai báo các chân động cơ
    void init();

    // Các phương thức di chuyển
    void movePWM(int leftPWM, int rightPWM);
    void movePWM(int leftPWM, int rightPWM, int minPWM);
    
};