#pragma once

class CONFIG
{
public:
    static constexpr float WHEEL_DIAMETER = 0.065;             // m
    static constexpr float WHEEL_RADIUS = WHEEL_DIAMETER / 2;  // m
    // static constexpr float WHEEL_SEPARATION = 0.13;            // m
    static constexpr float GEAR_RATIO = 20.4; 
    static constexpr int ENCODER_CPR = 11;  // số xung trên 1 vòng encoder (COUNTS_PER_REVOLUTION) 
    static constexpr int DECODE_FACTOR = 1; // hệ số giải mã xung A/B (x1, x2, x4), trong file motor_encoder đang là x1 (đếm RISING của S1)
    static constexpr float WHEEL_CPR = GEAR_RATIO * ENCODER_CPR * DECODE_FACTOR; // số xung trên 1 vòng trục (COUNTS_PER_REVOLUTION)
    
    static constexpr int PWM_FREQUENCY = 20000; // Hz
    static constexpr uint8_t PWM_RESOLUTION = 8; // Bits

public:
    static constexpr char * SSID = "dungvd";
    static constexpr char * PASSWORD = "44448888";
    static constexpr char * DESTINATION_IP = "10.42.0.1";
    static constexpr int DESTINATION_PORT = 8888;

public:
    int motor_driver_max_pwm = 255;
    int motor_driver_min_pwm = 135;
    float motor_driver_pid_kp = 0.001f;
    float motor_driver_pid_ki = 0.003f;
    float motor_driver_pid_kd = 0;

    // motor driver pins 32 33 25 26 27 14 
    uint8_t mot_left_drv_gpio_pwm = 32;
    uint8_t mot_left_drv_gpio_in1 = 33;
    uint8_t mot_left_drv_gpio_in2 = 25;
    uint8_t mot_right_drv_gpio_pwm = 14;
    uint8_t mot_right_drv_gpio_in1 = 25;
    uint8_t mot_right_drv_gpio_in2 = 33;

    // encoder pins
    uint8_t mot_left_enc_gpio_a_fg = 255;
    uint8_t mot_left_enc_gpio_b = 255;
    uint8_t mot_right_enc_gpio_a_fg = 255;
    uint8_t mot_right_enc_gpio_b = 255;
};