//
// Created by qingx on 2026/10/18.
//

#include "IMU.hpp"
#include "main.h"
#include "spi.h"

IMU imu_;
static uint8_t acc_ID;
static uint8_t dum;
static uint8_t gyro_ID;

void bmi088_write_byte(uint8_t tx_data) {
  HAL_SPI_Transmit(&hspi1,&tx_data,1,1000);
}

void bmi088_read_byte(uint8_t *rx_data,uint8_t length) {
  HAL_SPI_Receive(&hspi1,rx_data,length,1000);
}

void bmi088_write_reg(uint8_t reg,uint8_t data) {
  HAL_SPI_Transmit(&hspi1,&reg,1,1000);
  HAL_SPI_Transmit(&hspi1,&data,1,1000);
}


//
void bmi088_ACCEL_NS_L(void) {
  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port,CS1_ACCEL_Pin,GPIO_PIN_RESET);
}

void bmi088_ACCEL_NS_H(void) {
  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port,CS1_ACCEL_Pin,GPIO_PIN_SET);
}

void bmi088_GYRO_NS_L(void) {
  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port,CS1_GYRO_Pin,GPIO_PIN_RESET);
}

void bmi088_GYRO_NS_H(void) {
  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port,CS1_GYRO_Pin,GPIO_PIN_SET);
}

void bmi088_accel_write_single_reg(uint8_t reg,uint8_t data) {
  bmi088_GYRO_NS_H();
  bmi088_ACCEL_NS_L();

  bmi088_write_byte(reg&0x7f);
  bmi088_write_byte(data);

  bmi088_ACCEL_NS_H();
}

void bmi088_accel_read_reg(uint8_t reg,uint8_t *rx_data,uint8_t length) {
  uint8_t tmp;
  bmi088_GYRO_NS_H();
  bmi088_ACCEL_NS_L();

  bmi088_write_byte(reg | 0x80);
  bmi088_read_byte(&tmp,1);
  bmi088_read_byte(rx_data,length);

  bmi088_ACCEL_NS_H();
}

void bmi088_gyro_read_reg(uint8_t reg,uint8_t *rx_data,uint8_t length) {
  bmi088_ACCEL_NS_H();
  bmi088_GYRO_NS_L();

  bmi088_write_byte(reg | 0x80);
  bmi088_read_byte(rx_data,length);

  bmi088_GYRO_NS_H();
}

void bmi088_gyro_write_single_reg(uint8_t reg,uint8_t data) {
  bmi088_ACCEL_NS_H();
  bmi088_GYRO_NS_L();

  bmi088_write_byte(reg&0x7f);
  bmi088_write_byte(data);

  bmi088_GYRO_NS_H();
}

uint8_t bmi088_init(void) {
  uint8_t dummy = 0;
  uint8_t acc_id = 0;
  uint8_t gyro_id = 0;
  uint8_t acc_power[2] = {0};
  uint8_t acc_config[2] = {0};
  uint8_t gyro_config[3] = {0};

  bmi088_ACCEL_NS_H();
  bmi088_GYRO_NS_H();
  HAL_Delay(1);

  // Switch ACCEL to SPI，第一次读到的值不用
  bmi088_accel_read_reg(0x00, &dummy, 1);
  HAL_Delay(1);

  // Soft Reset ACCEL
  bmi088_accel_write_single_reg(0x7E, 0xB6);
  HAL_Delay(1);

  // Soft Reset GYRO
  bmi088_gyro_write_single_reg(0x14, 0xB6);
  HAL_Delay(100);

  // ACCEL 软复位后再做一次虚拟读取
  bmi088_accel_read_reg(0x00, &dummy, 1);
	dum=dummy;
  HAL_Delay(1);

  // Check Chip ID
  bmi088_accel_read_reg(0x00, &acc_id, 1);
  acc_ID = acc_id;
  if (acc_id != 0x1E) {
    return 0;
  }

  bmi088_gyro_read_reg(0x00, &gyro_id, 1);
  gyro_ID=gyro_id;
  if (gyro_id != 0x0F) {
    return 0;
  }

  // Switch ACCEL to Active Mode
  bmi088_accel_write_single_reg(0x7C, 0x00); // ACC_PWR_CONF
  HAL_Delay(5);
  bmi088_accel_write_single_reg(0x7D, 0x04); // ACC_PWR_CTRL
  HAL_Delay(5);

  // ACCEL Range / ODR / BW
  bmi088_accel_write_single_reg(0x41, 0x01); // ACC_RANGE
  HAL_Delay(1);
  bmi088_accel_write_single_reg(0x40, 0xAB); // ACC_CONF
  HAL_Delay(40);

  // GYRO Power / Range / ODR / BW
  bmi088_gyro_write_single_reg(0x11, 0x00); // GYRO_LPM1
  HAL_Delay(30);
  bmi088_gyro_write_single_reg(0x0F, 0x00); // GYRO_RANGE
  HAL_Delay(1);
  bmi088_gyro_write_single_reg(0x10, 0x02); // GYRO_BANDWIDTH
  HAL_Delay(10);

  // Read back，连续读取相邻的配置寄存器
  bmi088_accel_read_reg(0x7C, acc_power, 2);
  if (acc_power[0] != 0x00 || acc_power[1] != 0x04) {
    return 0;
  }
  bmi088_accel_read_reg(0x40, acc_config, 2);
  if (acc_config[0] != 0xAB || (acc_config[1] & 0x03) != 0x01) {
    return 0;
  }
  bmi088_gyro_read_reg(0x0F, gyro_config, 3);
  if ((gyro_config[0] & 0x07) != 0x00 ||
      (gyro_config[1] & 0x07) != 0x02 || gyro_config[2] != 0x00) {
    return 0;
      }

  return 1;
}

void IMU::accel_calculate() {
  bmi088_accel_read_reg(0x41, &raw_range,1);
  bmi088_accel_read_reg(0x12,rx_acc_data,6);
  int16_t accel_x = int16_t((uint16_t)rx_acc_data[1]<<8|rx_acc_data[0]);
  int16_t accel_y = int16_t((uint16_t)rx_acc_data[3]<<8|rx_acc_data[2]);
  int16_t accel_z = int16_t((uint16_t)rx_acc_data[5]<<8|rx_acc_data[4]);

  rx_acc_ms2[0]=accel_x*(6.0f/32768.0f)*9.80665f;
  rx_acc_ms2[1]=accel_y*(6.0f/32768.0f)*9.80665f;
  rx_acc_ms2[2]=accel_z*(6.0f/32768.0f)*9.80665f;
}

void IMU::gyro_calculate() {
  bmi088_gyro_read_reg(0x0F, &raw_range,1);
  bmi088_gyro_read_reg(0x02,rx_gyro_data,6);
  int16_t gyro_x = int16_t((uint16_t)rx_gyro_data[1]<<8|rx_gyro_data[0]);
  int16_t gyro_y = int16_t((uint16_t)rx_gyro_data[3]<<8|rx_gyro_data[2]);
  int16_t gyro_z = int16_t((uint16_t)rx_gyro_data[5]<<8|rx_gyro_data[4]);

  rx_gyro_rads[0]=gyro_x*(2000.0f/32768.0f)*dps2rads;
  rx_gyro_rads[1]=gyro_y*(2000.0f/32768.0f)*dps2rads;
  rx_gyro_rads[2]=gyro_z*(2000.0f/32768.0f)*dps2rads;
}

extern "C" uint8_t BMI088_init(void) {
  return bmi088_init();
}