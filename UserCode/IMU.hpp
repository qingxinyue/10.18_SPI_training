//
// Created by qingx on 2026/10/18.
//

#ifndef INC_1018_SPI_IMU_HPP
#define INC_1018_SPI_IMU_HPP
#include <stdint.h>

class IMU {
  struct raw_data{
    float accel[3];
    float gyro[3];
  };

  public:
  void accel_calculate();
  void gyro_calculate();
  const float* accel_ms2() const{return raw_data_.accel;}
  const float* gyro_ms2() const{return raw_data_.gyro;}
  void snesorDataTransform(void);

private:
  raw_data raw_data_;
  float R_imu_[3][3];
  float gyro_sensor_[3];
  float accel_sensor_[3];
  float gyro_bias_[3];
  float accel_bias_[3];
  float accel_sen_;

  const float dps2rads=3.1415925f/180.0f;
  const float R_imu[3][3] = {{-1.0f, 0.0f, 0.0f},
                         {0.0f, 1.0f, 0.0f},
                         {0.0f, 0.0f, -1.0f}};
  uint8_t raw_range;
  uint8_t rx_acc_data[6]={};
  uint8_t rx_gyro_data[6]={};
};

extern IMU imu_;
#endif // INC_1018_SPI_IMU_HPP
