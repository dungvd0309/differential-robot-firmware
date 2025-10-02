#include "rtos_tasks.h"
#include "ros_interface.h"

static void ros_task(void*) {
  ros_init();
  for(;;){ ros_spin_some(); vTaskDelay(pdMS_TO_TICKS(1)); }
}

void start_tasks() {
  xTaskCreatePinnedToCore(ros_task, "ros", 8192, NULL, 2, NULL, 0);
}