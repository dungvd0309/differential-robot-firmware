#include "ros_interface.h"
#include "motor_encoders.h"

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>

static rcl_publisher_t publisher; 
static sensor_msgs__msg__JointState pub_msg;
static rclc_executor_t executor;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rcl_timer_t timer;

extern MotorEncoders encoders; // Use the global encoders object from the .ino file

static char* ssid = "myap";
static char* password = "44448888";
static char* destination_ip = "10.25.48.58";
static int destination_port = 8888;

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

static void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // Update values directly before publishing
        encoders.updateVelocities();

        pub_msg.position.data[0] = encoders.getLeftAngle();
        pub_msg.velocity.data[0] = encoders.getLeftAngularVelocity();
        pub_msg.position.data[1] = encoders.getRightAngle();
        pub_msg.velocity.data[1] = encoders.getRightAngularVelocity();

        RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
        // msg.data++;
    }
}

// Initialize ROS message function
static void ros_msg_init()
{
    const char * joint_names[] = {"left_wheel_joint", "right_wheel_joint"};
    sensor_msgs__msg__JointState__init(&pub_msg);
    rosidl_runtime_c__String__Sequence__init(&pub_msg.name, 2);
    rosidl_runtime_c__String__assign(&pub_msg.name.data[0], joint_names[0]);
    rosidl_runtime_c__String__assign(&pub_msg.name.data[1], joint_names[1]);


    pub_msg.position.data = (double*)malloc(sizeof(double) * 2);
    pub_msg.position.size = 2;
    pub_msg.position.capacity = 2;
    pub_msg.position.data[0] = 0;
    pub_msg.position.data[1] = 0;

    pub_msg.velocity.data = (double*)malloc(sizeof(double) * 2);
    pub_msg.velocity.size = 2;
    pub_msg.velocity.capacity = 2;
    pub_msg.velocity.data[0] = 0;
    pub_msg.velocity.data[1] = 0;
}

void ros_init()
{
    // set_microros_transports(); // serial
    set_microros_wifi_transports(ssid, password, destination_ip, destination_port); // wifi

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
        "wheel_pose"));
    
    // create timer
    const unsigned int timer_timeout = 4; // 250hz
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));
    
    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    ros_msg_init();
    digitalWrite(LED_PIN, LOW);
}      

void ros_spin_some(int delay)
{
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(delay)));
}
