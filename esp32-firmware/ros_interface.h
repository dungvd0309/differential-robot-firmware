#pragma once

#define LED_PIN 2

void error_loop();

void ros_init();

void ros_spin_some(int delay = 5);
