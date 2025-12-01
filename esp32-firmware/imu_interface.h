#include <sys/_intsup.h>
#include <sys/_types.h>
#include "Arduino.h"
#pragma once

#include <Wire.h>
#include <MPU6050.h>
#include <micro_ros_arduino.h>
#include <sensor_msgs/msg/imu.h>

MPU6050 mpu;

#define GYRO_SENS 131.0 // ±250°/s sensitivity
#define ACCEL_SENS 16384.0 // ±2g sensitivity
#define G_TO_MS2 9.80665;           // g to m/s^2

// unsigned long time_prev_us;
// long gyro_angle_z = 0;
// long angle_z = 0;

float linear_acceleration_x, linear_acceleration_y, linear_acceleration_z;
float angular_velocity_x, angular_velocity_y, angular_velocity_z;

void setupImu() {
  Wire.begin();
  mpu.initialize();
  Serial.println("Testing IMU connection...");
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  mpu.setXAccelOffset(-4454);
  mpu.setYAccelOffset(-856);
  mpu.setZAccelOffset(853);
  mpu.setXGyroOffset(113);
  mpu.setYGyroOffset(-10);
  mpu.setZGyroOffset(3);

  // time_prev_us = 0;
}

void updateImu() {
  // unsigned long time_now = micros();
  // int delta_time = time_now - time_prev_us;
  // time_prev_us = time_now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  linear_acceleration_x = (ax / ACCEL_SENS) * G_TO_MS2;
  linear_acceleration_y = (ay / ACCEL_SENS)* G_TO_MS2;
  linear_acceleration_z = (az / ACCEL_SENS)* G_TO_MS2;
  angular_velocity_x = (gx / GYRO_SENS) * DEG_TO_RAD;
  angular_velocity_y = (gy / GYRO_SENS) * DEG_TO_RAD;
  angular_velocity_z = (gz / GYRO_SENS) * DEG_TO_RAD;

  // gyro_angle_z = gyro_angle_z + gz/GYRO_SENS*delta_time/1e6;

  // Serial.print(gz/GYRO_SENS); Serial.print("\t");
  // Serial.print(gyro_angle_z);
  // Serial.println();
}

void imuToMsg(sensor_msgs__msg__Imu& imu_pub_msg) {
  imu_pub_msg.linear_acceleration.x = linear_acceleration_x;
  imu_pub_msg.linear_acceleration.y = linear_acceleration_y;
  imu_pub_msg.linear_acceleration.z = linear_acceleration_z;
  imu_pub_msg.angular_velocity.x = angular_velocity_x;
  imu_pub_msg.angular_velocity.y = angular_velocity_y;
  imu_pub_msg.angular_velocity.z = angular_velocity_z;
}