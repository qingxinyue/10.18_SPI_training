//
// Created by qingx on 2026/10/18.
//

#ifndef INC_1018_SPI_IMU_HPP
#define INC_1018_SPI_IMU_HPP
#include <stdint.h>

class IMU {
  public:
  void accel_calculate();
  void gyro_calculate();
  const float* accel_ms2() const{return rx_acc_ms2;}
  const float* gyro_ms2() const{return rx_gyro_rads;}

private:
  const float dps2rads=3.1415925f/180.0f;
  uint8_t raw_range;
  uint8_t rx_acc_data[6]={};
  uint8_t rx_gyro_data[6]={};
  float rx_acc_ms2[3]={};
  float rx_gyro_rads[3]={};
};

extern IMU imu_;
#endif // INC_1018_SPI_IMU_HPP
