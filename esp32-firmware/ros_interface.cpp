#include "ros_interface.h"
#include "motors.h"
#include "robot_config.h"

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>
#include <geometry_msgs/msg/twist.h>

static rcl_publisher_t publisher; 
static sensor_msgs__msg__JointState pub_msg;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist sub_msg;
static rclc_executor_t executor;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rcl_timer_t timer;

extern CONFIG cfg;

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
        // pub_msg.position.data[0] = encoders.getLeftAngle();
        // pub_msg.velocity.data[0] = encoders.getLeftAngularVelocity();
        // pub_msg.position.data[1] = encoders.getRightAngle();
        // pub_msg.velocity.data[1] = encoders.getRightAngularVelocity();
        
        // pub_msg.position.data[0] = 0;
        pub_msg.velocity.data[0] = cfg.rpm_to_speed(leftMotor.getCurrentRPM());
        // pub_msg.position.data[1] = 0;
        pub_msg.velocity.data[1] = cfg.rpm_to_speed(rightMotor.getCurrentRPM());

        RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
    }
}

// initialize ROS message 
static void ros_msg_init()
{
    const char * joint_names[] = {"left_wheel_joint", "right_wheel_joint"};
    sensor_msgs__msg__JointState__init(&pub_msg);
    rosidl_runtime_c__String__Sequence__init(&pub_msg.name, 2);
    rosidl_runtime_c__String__assign(&pub_msg.name.data[0], joint_names[0]);
    rosidl_runtime_c__String__assign(&pub_msg.name.data[1], joint_names[1]);


    // pub_msg.position.data = (double*)malloc(sizeof(double) * 2);
    // pub_msg.position.size = 2;
    // pub_msg.position.capacity = 2;
    // pub_msg.position.data[0] = 0;
    // pub_msg.position.data[1] = 0;

    pub_msg.velocity.data = (double*)malloc(sizeof(double) * 2);
    pub_msg.velocity.size = 2;
    pub_msg.velocity.capacity = 2;
    pub_msg.velocity.data[0] = 0;
    pub_msg.velocity.data[1] = 0;
}

//twist message cb
void subscription_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    float target_speed_lin_x = msg->linear.x;
    float target_speed_ang_z = msg->angular.z;
    Serial.print("linear.x ");
    Serial.print(msg->linear.x);
    Serial.print(", angular.z ");
    Serial.println(msg->angular.z);

    float twist_target_speed_right = 0;
    float twist_target_speed_left = 0;

    Serial.print("twist_target_speed_right ");
    Serial.print(twist_target_speed_right);
    Serial.print(", twist_target_speed_left ");
    Serial.println(twist_target_speed_left);

    cfg.twistToWheelSpeeds(target_speed_lin_x, target_speed_ang_z,
    &twist_target_speed_right, &twist_target_speed_left);
    float twist_target_rpm_right = cfg.speed_to_rpm(twist_target_speed_right);
    float twist_target_rpm_left = cfg.speed_to_rpm(twist_target_speed_left);
    
    setMotorsRPM(twist_target_rpm_left, twist_target_rpm_right);
}

void ros_init(bool wifi_mode)
{   
    if(wifi_mode)
        set_microros_wifi_transports(cfg.SSID, cfg.PASSWORD, cfg.DESTINATION_IP, cfg.DESTINATION_PORT); // wifi
    else
        set_microros_transports(); // serial

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); 
    
    delay(2000);

    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "hardware_node", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), 
        "hardware_states"));
    
    // create timer
    const unsigned int timer_timeout = 10; // 50hz
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel1"));
    
    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA));

    // initialize ROS message
    ros_msg_init();
    digitalWrite(LED_PIN, LOW);
}      

void ros_spin_some(int delay)
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(delay));
}
