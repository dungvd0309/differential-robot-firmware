#pragma once

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

// Global variable declarations
extern rcl_publisher_t publisher;
extern std_msgs__msg__Float32 msg;
extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;
extern rcl_timer_t timer;

#define LED_PIN 2

// Function declarations
void error_loop();

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time);

void micro_ros_data_publish(float pose);

void ros_init();

void ros_spin_some(int delay = 5);
