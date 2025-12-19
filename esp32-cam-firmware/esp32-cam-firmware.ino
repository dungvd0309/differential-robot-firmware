#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/compressed_image.h>
#include "esp_camera.h"
#include "esp_heap_caps.h"

// Camera pins for AI-Thinker ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// micro-ROS objects
rcl_publisher_t publisher;
sensor_msgs__msg__CompressedImage msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Publishing frequency
#define PUBLISH_FREQUENCY 10  // 10 Hz
#define TIMER_TIMEOUT (1000 / PUBLISH_FREQUENCY)  // milliseconds

// Buffer for image data (allocated in regular RAM, not PSRAM)
#define MAX_IMAGE_SIZE 65536  // 64KB max
uint8_t* image_buffer = NULL;

// Error handling macro
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// LED pin for status indication
#define LED_PIN 33

void error_loop() {
  while(1) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

// Initialize camera
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;  // Use JPEG format for compressed image
  
  // Frame size and quality settings
  // Use fb_location to put frame buffers in PSRAM (if available)
  if(psramFound()) {
    config.frame_size = FRAMESIZE_VGA;  // 640x480
    config.jpeg_quality = 12;  // 0-63, lower means higher quality
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;  // Explicitly use PSRAM for frame buffers
    config.grab_mode = CAMERA_GRAB_LATEST;     // Always grab latest frame
    Serial.println("PSRAM found - using VGA resolution");
  } else {
    config.frame_size = FRAMESIZE_QVGA;  // 320x240
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    Serial.println("No PSRAM - using QVGA resolution");
  }
  
  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }
  
  // Additional sensor settings
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_brightness(s, 0);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect)
    s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);        // 0 to 4
    s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
    s->set_agc_gain(s, 0);       // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
    s->set_bpc(s, 0);            // 0 = disable , 1 = enable
    s->set_wpc(s, 1);            // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
    s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
    s->set_vflip(s, 0);          // 0 = disable , 1 = enable
    s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
  }
  
  Serial.println("Camera initialized successfully");
  return true;
}

// Validate JPEG buffer
bool isValidJPEG(uint8_t* buf, size_t len) {
  if (len < 2) return false;
  // Check for JPEG SOI (Start of Image) marker: 0xFFD8
  if (buf[0] != 0xFF || buf[1] != 0xD8) return false;
  // Check for JPEG EOI (End of Image) marker: 0xFFD9
  if (len < 2 || buf[len-2] != 0xFF || buf[len-1] != 0xD9) return false;
  return true;
}

// Timer callback to publish images
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Capture image
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      return;
    }
    
    // Validate JPEG format
    if (fb->format != PIXFORMAT_JPEG) {
      Serial.println("Invalid format - expected JPEG");
      esp_camera_fb_return(fb);
      return;
    }
    
    // Check image size
    if (fb->len > MAX_IMAGE_SIZE) {
      Serial.printf("Image too large: %d bytes (max %d)\n", fb->len, MAX_IMAGE_SIZE);
      esp_camera_fb_return(fb);
      return;
    }
    
    // Validate JPEG markers
    if (!isValidJPEG(fb->buf, fb->len)) {
      Serial.println("Invalid JPEG data - missing markers");
      esp_camera_fb_return(fb);
      return;
    }
    
    // Copy from PSRAM to regular RAM to avoid cache coherency issues
    memcpy(image_buffer, fb->buf, fb->len);
    size_t image_size = fb->len;
    
    // Return frame buffer immediately after copying
    esp_camera_fb_return(fb);
    
    // Prepare message with copied data
    msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
    msg.header.stamp.nanosec = (rmw_uros_epoch_millis() % 1000) * 1000000;
    
    // Set frame_id
    static char frame_id[] = "esp32_cam_link";
    msg.header.frame_id.data = frame_id;
    msg.header.frame_id.size = strlen(frame_id);
    msg.header.frame_id.capacity = strlen(frame_id) + 1;
    
    // Set format
    static char format[] = "jpeg";
    msg.format.data = format;
    msg.format.size = strlen(format);
    msg.format.capacity = strlen(format) + 1;
    
    // Set image data from copied buffer (not PSRAM)
    msg.data.data = image_buffer;
    msg.data.size = image_size;
    msg.data.capacity = image_size;
    
    // Publish
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    
    if (ret == RCL_RET_OK) {
      // Blink LED to indicate successful publish
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      Serial.printf("Published image: %d bytes\n", image_size);
    } else {
      Serial.printf("Failed to publish image (error code: %d)\n", ret);
    }
  }
}

void setup() {
  // Initialize serial
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("ESP32-CAM micro-ROS Image Publisher");
  Serial.println("====================================");
  
  // Print memory info
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  if (psramFound()) {
    Serial.printf("PSRAM found: %d bytes\n", ESP.getPsramSize());
    Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());
  }
  
  // Allocate image buffer in regular RAM (not PSRAM)
  // This is critical to avoid cache coherency issues
  image_buffer = (uint8_t*)heap_caps_malloc(MAX_IMAGE_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  if (image_buffer == NULL) {
    Serial.println("Failed to allocate image buffer!");
    Serial.println("Trying smaller buffer size...");
    // Try with smaller buffer
    image_buffer = (uint8_t*)malloc(32768);  // 32KB fallback
    if (image_buffer == NULL) {
      Serial.println("Critical: Cannot allocate buffer!");
      error_loop();
    }
  }
  Serial.printf("Allocated image buffer: %d bytes in RAM\n", MAX_IMAGE_SIZE);
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize camera
  if (!initCamera()) {
    Serial.println("Camera initialization failed!");
    error_loop();
  }
  
  // Configure WiFi transport
  Serial.println("Connecting to WiFi...");
  set_microros_wifi_transports("dungvd", "44448888", "10.42.0.1", 8888);
  Serial.println("WiFi connected..");
  
  delay(2000);
  
  // Initialize micro-ROS
  allocator = rcl_get_default_allocator();
  
  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Create node
  RCCHECK(rclc_node_init_default(&node, "esp32_cam_publisher", "", &support));
  
  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
    "esp32_cam/image/compressed"));
  
  // Create timer for 10 Hz publishing
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(TIMER_TIMEOUT),
    timer_callback));
  
  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  Serial.println("====================================");
  Serial.println("micro-ROS node initialized successfully");
  Serial.printf("Publishing at %d Hz to topic: esp32_cam/image/compressed\n", PUBLISH_FREQUENCY);
  Serial.println("====================================");
  
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Spin executor to handle callbacks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
}