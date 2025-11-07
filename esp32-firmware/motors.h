#pragma once

#include <Arduino.h>
#include "robot_config.h"
#include "motor_controller.h"

extern CONFIG cfg;

MotorController leftMotor, rightMotor;

void IRAM_ATTR isrLeft() {
  leftMotor.tickEncoder(digitalRead(cfg.mot_left_enc_gpio_b) == HIGH);
}
void IRAM_ATTR isrRight() {
  rightMotor.tickEncoder(digitalRead(cfg.mot_right_enc_gpio_b) == HIGH);
}

void setMotorPWM(MotorController * motor, float pwm) {
  bool is_right = motor == &rightMotor;

  uint8_t pwm_channel = is_right ? cfg.MOT_PWM_RIGHT_CHANNEL : cfg.MOT_PWM_LEFT_CHANNEL;
  uint8_t pwm_pin = is_right ? cfg.mot_right_drv_gpio_pwm : cfg.mot_left_drv_gpio_pwm;
  uint8_t in1_pin = is_right ? cfg.mot_right_drv_gpio_in1 : cfg.mot_left_drv_gpio_in1;
  uint8_t in2_pin = is_right ? cfg.mot_right_drv_gpio_in2 : cfg.mot_left_drv_gpio_in2;

  int max_pwm = cfg.motor_driver_max_pwm;
  // int min_pwm = cfg.motor_driver_min_pwm;
  int pwm_magnitude = round(abs(pwm) * max_pwm);

  if (pwm == 0) {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, HIGH);
    return;
  }
  else if (pwm > 0) {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
  }
  else {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
  }
  ledcAttachPin(pwm_pin, pwm_channel);
  ledcWrite(pwm_channel, pwm_magnitude);
}

void setupEncoders() {
  pinMode(cfg.mot_left_enc_gpio_a_fg, INPUT_PULLUP);
  pinMode(cfg.mot_left_enc_gpio_b, INPUT_PULLUP);
  attachInterrupt(cfg.mot_left_enc_gpio_a_fg, isrLeft, RISING);

  pinMode(cfg.mot_right_enc_gpio_a_fg, INPUT_PULLUP);
  pinMode(cfg.mot_right_enc_gpio_b, INPUT_PULLUP);
  attachInterrupt(cfg.mot_right_enc_gpio_a_fg, isrRight, RISING);
}

void setupMotors() {
  // in 1,2,3,4 pins setup
  pinMode(cfg.mot_left_drv_gpio_in1, OUTPUT);
  pinMode(cfg.mot_left_drv_gpio_in2, OUTPUT);
  pinMode(cfg.mot_right_drv_gpio_in1, OUTPUT);
  pinMode(cfg.mot_right_drv_gpio_in2, OUTPUT);

  // PWM pins setup
  pinMode(cfg.mot_left_drv_gpio_pwm, OUTPUT);
  pinMode(cfg.mot_right_drv_gpio_pwm, OUTPUT);
  ledcSetup(cfg.MOT_PWM_LEFT_CHANNEL, cfg.PWM_FREQUENCY, cfg.PWM_RESOLUTION);
  ledcSetup(cfg.MOT_PWM_RIGHT_CHANNEL, cfg.PWM_FREQUENCY, cfg.PWM_RESOLUTION);
  // ledcAttachPin(cfg.mot_left_drv_gpio_pwm, cfg.MOT_PWM_LEFT_CHANNEL);
  // ledcAttachPin(cfg.mot_right_drv_gpio_pwm, cfg.MOT_PWM_RIGHT_CHANNEL);

  leftMotor.setPWMCallback(setMotorPWM);
  rightMotor.setPWMCallback(setMotorPWM);

  leftMotor.init(cfg.WHEEL_CPR, cfg.motor_driver_pid_kp, cfg.motor_driver_pid_ki, cfg.motor_driver_pid_kd, cfg.motor_driver_max_pwm);
  rightMotor.init(cfg.WHEEL_CPR, cfg.motor_driver_pid_kp, cfg.motor_driver_pid_ki, cfg.motor_driver_pid_kd, cfg.motor_driver_max_pwm);
  
}