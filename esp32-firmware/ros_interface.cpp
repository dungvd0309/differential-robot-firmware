#include "ros_interface.h"

rcl_publisher_t publisher; 
std_msgs__msg__Float32 msg; 
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

void error_loop() {
    while(1) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        // msg.data++;
    }
}

void micro_ros_data_publish(float pose)
{
    msg.data = pose;
}

void ros_init()
{
    set_microros_transports();

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); 
    
    delay(2000);

    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "hardware_node", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), 
        "wheel_pose"));
    
    // create timer,
    const unsigned int timer_timeout = 4; // 250hz
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));
    
    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg.data = 0;
}      

void ros_spin_some(int delay)
{
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(delay)));
}
