//
// Created by qingx on 2026/10/18.
//
#include "main.h"
#include "IMU.hpp"

extern "C" void IMU_update(void) {
  imu_.accel_calculate();
  imu_.gyro_calculate();
}