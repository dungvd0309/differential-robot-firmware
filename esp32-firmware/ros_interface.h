#pragma once

void error_loop();

void ros_init(bool wifi_mode = true);

void ros_spin_some(int delay = 5);
