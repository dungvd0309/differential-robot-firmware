#pragma once

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>

#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/joint_state.h>

#define LED_PIN 2

// Macro for checking return codes of rcl functions
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop();

void ros_msg_update(float pose);

void ros_init();

void ros_spin_some(int delay = 5);
