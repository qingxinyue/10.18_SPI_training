//
// Created by qingx on 2026/10/18.
//

#ifndef INC_1018_SPI_BMI088_HPP
#define INC_1018_SPI_BMI088_HPP

#include <stdint.h>

namespace bmi088 {
  uint8_t bmi088_init(void);

  void bmi088_accel_read_reg(uint8_t reg, uint8_t* rx_data, uint8_t length);

  void bmi088_gyro_read_reg(uint8_t reg, uint8_t* rx_data, uint8_t length);

  void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data);

  void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t data);
}


#endif // INC_1018_SPI_BMI088_HPP
