//
// Created by qingx on 2026/10/18.
//

#include "IMU.hpp"
#include "bmi088.hpp"

IMU imu_;

void IMU::accel_calculate() {
  bmi088::bmi088_accel_read_reg(0x41, &raw_range,1);
  bmi088::bmi088_accel_read_reg(0x12,rx_acc_data,6);
  int16_t accel_x = int16_t((uint16_t)rx_acc_data[1]<<8|rx_acc_data[0]);
  int16_t accel_y = int16_t((uint16_t)rx_acc_data[3]<<8|rx_acc_data[2]);
  int16_t accel_z = int16_t((uint16_t)rx_acc_data[5]<<8|rx_acc_data[4]);

  raw_data_.accel[0]=accel_x*(6.0f/32768.0f)*9.80665f;
  raw_data_.accel[1]=accel_y*(6.0f/32768.0f)*9.80665f;
  raw_data_.accel[2]=accel_z*(6.0f/32768.0f)*9.80665f;
}

void IMU::gyro_calculate() {
  bmi088::bmi088_gyro_read_reg(0x0F, &raw_range,1);
  bmi088::bmi088_gyro_read_reg(0x02,rx_gyro_data,6);
  int16_t gyro_x = int16_t((uint16_t)rx_gyro_data[1]<<8|rx_gyro_data[0]);
  int16_t gyro_y = int16_t((uint16_t)rx_gyro_data[3]<<8|rx_gyro_data[2]);
  int16_t gyro_z = int16_t((uint16_t)rx_gyro_data[5]<<8|rx_gyro_data[4]);

  raw_data_.gyro[0]=gyro_x*(2000.0f/32768.0f)*dps2rads;
  raw_data_.gyro[1]=gyro_y*(2000.0f/32768.0f)*dps2rads;
  raw_data_.gyro[2]=gyro_z*(2000.0f/32768.0f)*dps2rads;
}

void IMU::snesorDataTransform(void){
  for (uint8_t i = 0; i < 3; i++) {
    gyro_sensor_[i] = raw_data_.gyro[0] * R_imu_[i][0] +
                      raw_data_.gyro[1] * R_imu_[i][1] +
                      raw_data_.gyro[2] * R_imu_[i][2] - gyro_bias_[i];
    accel_sensor_[i] = accel_sen_ * (raw_data_.accel[0] * R_imu_[i][0] +
                                     raw_data_.accel[1] * R_imu_[i][1] +
                                     raw_data_.accel[2] * R_imu_[i][2]) -
                       accel_bias_[i];
  }
}