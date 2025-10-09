#include "ros_interface.h"
#include "motor_encoders.h"

static rcl_publisher_t publisher; 
static sensor_msgs__msg__JointState pub_msg;
static rclc_executor_t executor;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rcl_timer_t timer;

extern MotorEncoders encoders; // Use the global encoders object from the .ino file

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
        RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
        // msg.data++;
    }
}

// Initialize ROS message function
static void ros_msg_init()
{
    const char * joint_names[] = {"wheel_joint"};
    sensor_msgs__msg__JointState__init(&pub_msg);
    rosidl_runtime_c__String__Sequence__init(&pub_msg.name, 1);
    rosidl_runtime_c__String__assign(&pub_msg.name.data[0], joint_names[0]);

    pub_msg.position.data = (double*)malloc(sizeof(double) * 1);
    pub_msg.position.size = 1;
    pub_msg.position.capacity = 1;
    pub_msg.position.data[0] = 0;

    pub_msg.velocity.data = (double*)malloc(sizeof(double) * 1);
    pub_msg.velocity.size = 1;
    pub_msg.velocity.capacity = 1;
    pub_msg.velocity.data[0] = 0;
}

void ros_init()
{
    set_microros_transports();

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
