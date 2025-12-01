#include "esp32-hal.h"
#include <sys/_types.h>
#pragma once

#include "robot_config.h"
#include "motors.h"
#include "motor_controller.h"
#include "imu_interface.h"

#include <micro_ros_arduino.h>
#include "rmw_microros/time_sync.h"

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/twist.h>

static rcl_publisher_t encoders_publisher; 
static sensor_msgs__msg__JointState joint_state_pub_msg;
static rcl_publisher_t imu_publisher; 
static sensor_msgs__msg__Imu imu_pub_msg;
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist twist_sub_msg;

static rclc_executor_t executor;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rcl_timer_t timer;

extern CONFIG cfg;
extern MotorController leftMotor, rightMotor;

unsigned long last_sync_time;

#define LED_PIN 2

// Macro for checking return codes of rcl functions
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
    while(1) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
}

// publishing timer cb
static void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // Encoder to msg
        joint_state_pub_msg.position.data[0] = leftMotor.getEncoderRadValue();
        joint_state_pub_msg.velocity.data[0] = cfg.rpm_to_speed(leftMotor.getCurrentRPM());
        joint_state_pub_msg.position.data[1] = rightMotor.getEncoderRadValue();
        joint_state_pub_msg.velocity.data[1] = cfg.rpm_to_speed(rightMotor.getCurrentRPM());

        // IMU to msg
        imuToMsg(imu_pub_msg);

        // Bind timestamp to msg 
        int64_t time_ns = rmw_uros_epoch_nanos();
        joint_state_pub_msg.header.stamp.sec = time_ns / 1000000000;
        joint_state_pub_msg.header.stamp.nanosec = time_ns % 1000000000;
        imu_pub_msg.header.stamp.sec = joint_state_pub_msg.header.stamp.sec;
        imu_pub_msg.header.stamp.nanosec = joint_state_pub_msg.header.stamp.nanosec;
        
        // Publish msg
        RCSOFTCHECK(rcl_publish(&encoders_publisher, &joint_state_pub_msg, NULL));
        RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_pub_msg, NULL));
    }
}

// initialize ROS message 
static void ros_msg_init()
{
    const char * joint_names[] = {"left_wheel_joint", "right_wheel_joint"};
    sensor_msgs__msg__JointState__init(&joint_state_pub_msg);
    rosidl_runtime_c__String__Sequence__init(&joint_state_pub_msg.name, 2);
    rosidl_runtime_c__String__assign(&joint_state_pub_msg.name.data[0], joint_names[0]);
    rosidl_runtime_c__String__assign(&joint_state_pub_msg.name.data[1], joint_names[1]);

    joint_state_pub_msg.position.data = (double*)malloc(sizeof(double) * 2);
    joint_state_pub_msg.position.size = 2;
    joint_state_pub_msg.position.capacity = 2;
    joint_state_pub_msg.position.data[0] = 0.0;
    joint_state_pub_msg.position.data[1] = 0.0;

    joint_state_pub_msg.velocity.data = (double*)malloc(sizeof(double) * 2);
    joint_state_pub_msg.velocity.size = 2;
    joint_state_pub_msg.velocity.capacity = 2;
    joint_state_pub_msg.velocity.data[0] = 0.0;
    joint_state_pub_msg.velocity.data[1] = 0.0;

    // joint_state_pub_msg.header.frame_id.data = NULL;
    // joint_state_pub_msg.header.frame_id.size = 0;
    // joint_state_pub_msg.header.frame_id.capacity = 0;
}

// /cmd_vel topic callback
void subscription_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    float target_speed_lin_x = msg->linear.x;
    float target_speed_ang_z = msg->angular.z;

    // Serial.print("linear.x ");
    // Serial.print(msg->linear.x);
    // Serial.print(", angular.z ");
    // Serial.println(msg->angular.z);

    float twist_target_speed_right = 0;
    float twist_target_speed_left = 0;

    cfg.twistToWheelSpeeds(target_speed_lin_x, target_speed_ang_z,
    &twist_target_speed_right, &twist_target_speed_left);

    // Serial.print("twist_target_speed_right ");
    // Serial.print(twist_target_speed_right);
    // Serial.print(", twist_target_speed_left ");
    // Serial.println(twist_target_speed_left);

    float twist_target_rpm_right = cfg.speed_to_rpm(twist_target_speed_right);
    float twist_target_rpm_left = cfg.speed_to_rpm(twist_target_speed_left);
    
    setMotorsRPM(twist_target_rpm_left, twist_target_rpm_right);
}

void ros_init(bool wifi_mode = true)
{   
    // set transmission
    if(wifi_mode) {
        Serial.print("Connecting to "); Serial.print(cfg.SSID); Serial.println("...");
        set_microros_wifi_transports(cfg.SSID, cfg.PASSWORD, cfg.DESTINATION_IP, cfg.DESTINATION_PORT); // wifi
        Serial.println("Wifi connected!");
    }
    else {
        set_microros_transports(); // serial
    }

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); 
    
    delay(2000);

    allocator = rcl_get_default_allocator();
    Serial.println("Created allocator. Connecting to micro_ros_agent...");

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    Serial.println("Created init options");

    // time sync
    RCCHECK(rmw_uros_sync_session(1000));
    last_sync_time = millis();
    Serial.println("Time synced");

    // create node
    RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));
    Serial.println("Created node");

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &encoders_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), 
        "/encoders/data_raw"));
    Serial.println("Created /encoders/data_raw publisher");

    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), 
        "/imu/data_raw"));
    Serial.println("Created /imu/data_raw publisher");
    
    // create timer
    const unsigned int timer_timeout = 20; // 50hz
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));
    Serial.println("Created timer");

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));
    Serial.println("Created /cmd_vel subcriber");
    
    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &twist_sub_msg, &subscription_callback, ON_NEW_DATA));
    Serial.println("Created executor");

    // initialize ROS message
    ros_msg_init();
    digitalWrite(LED_PIN, LOW);
    
    Serial.println("micro-ROS initialised!");
}      

void ros_time_sync() {
    if (millis() - last_sync_time > 60000) {
      rmw_uros_sync_session(10); 
      last_sync_time = millis();
  }
}

void ros_spin_some(int delay = 5)
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(delay));
    ros_time_sync();
}

