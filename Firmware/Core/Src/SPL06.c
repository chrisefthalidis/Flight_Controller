#include "SPL06.h"

void SPL06_Read_Register(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
  HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

void SPL06_Read_Registers(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t length)
{
  HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

void SPL06_Write_Register(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
  HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

void SPL06_Write_Registers(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t length)
{
  HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

void SPL06_Initialise(SPL06 *dev)
{
  dev->c00 = 0;
  dev->c10 = 0;

  dev->c20 = 0;
  dev->c30 = 0;
  dev->c01 = 0;
  dev->c11 = 0;
  dev->c21 = 0;

  dev->bar_bias = 0.f;
  dev->bar_alt_m = 0.f;

  uint8_t tx_data, rx_data[15];

  tx_data = 0x61; // Pressure measurement rate: 64, Pressure oversampling rate: 2
  SPL06_Write_Register(SPL06_ADDR, SPL06_REG_PRS_CFG, &tx_data);

  tx_data = 0xA1; // Temperature measurement rate: 4, Temperature oversampling rate: 2
  SPL06_Write_Register(SPL06_ADDR, SPL06_REG_TMP_CFG, &tx_data);

  tx_data = 0b0111; // Continuous pressure and temperature measurement
  SPL06_Write_Register(SPL06_ADDR, SPL06_REG_MEAS_CFG, &tx_data);

  // Read the Calibration Coefficients register
  SPL06_Read_Registers(SPL06_ADDR, SPL06_REG_COEF, rx_data, sizeof(rx_data));

  // Calculate the Calibration Coefficients
  dev->c00 = ((uint32_t)rx_data[0] << 12) | ((uint16_t)rx_data[1] << 4) | ((uint16_t)rx_data[2] >> 4);
  dev->c10 = ((uint32_t)(rx_data[2] & 0x0F)) << 16 | ((uint16_t)rx_data[3] << 8) | rx_data[4];
  dev->c01 = ((uint16_t)rx_data[5] << 8) | rx_data[6];
  dev->c11 = ((uint16_t)rx_data[7] << 8) | rx_data[8];
  dev->c20 = ((uint16_t)rx_data[9] << 8) | rx_data[10];
  dev->c21 = ((uint16_t)rx_data[11] << 8) | rx_data[12];
  dev->c30 = ((uint16_t)rx_data[13] << 8) | rx_data[14];

  if (dev->c00 & (1 << 19))
    dev->c00 = dev->c00 | 0xFFF00000;

  if (dev->c10 & (1 << 19))
    dev->c10 = dev->c10 | 0xFFF00000;
}

void SPL06_Calibrate(SPL06 *dev)
{
  float avg_bar_bias = 0.f;

  // Take 2000 measurements and average them to get the bias
  for (uint16_t i = 0; i < 2010; i++)
  {
    SPL06_Read(dev);

    if (i >= 10)
      avg_bar_bias += dev->bar_alt_m;

    HAL_Delay(10);
  }

  dev->bar_bias = avg_bar_bias / 2000;
}

void SPL06_Read(SPL06 *dev)
{
  uint8_t data[6];
  int32_t press_raw = 0, temp_raw = 0;
  float press_raw_scaled = 0.f, temp_raw_scaled = 0.f, press_hPa = 0.f;

  // Read all the data registers
  SPL06_Read_Registers(SPL06_ADDR, SPL06_REG_PSR_B2, data, sizeof(data));

  // Calculate raw pressure and temperature values
  press_raw = ((uint32_t)data[0] << 16) | ((uint16_t)data[1] << 8) | data[2];
  temp_raw = ((uint32_t)data[3] << 16) | ((uint16_t)data[4] << 8) | data[5];

  if (press_raw & (1 << 23))
    press_raw = press_raw | 0xFF000000;

  if (temp_raw & (1 << 23))
    temp_raw = temp_raw | 0xFF000000;

  press_raw_scaled = (float)press_raw / 1572864.f;
  temp_raw_scaled = (float)temp_raw / 1572864.f;

  // Convert the raw values into pressure in hPa
  press_hPa = ((float)dev->c00 + press_raw_scaled * ((float)dev->c10 + press_raw_scaled * ((float)dev->c20 + press_raw_scaled * (float)dev->c30)) +
               temp_raw_scaled * (float)dev->c01 + temp_raw_scaled * press_raw_scaled * ((float)dev->c11 + press_raw_scaled * (float)dev->c21)) /
              100.f;

  // Convert the pressure in hPa into altitude in meters
  dev->bar_alt_m = 44330.f * (1.f - powf(press_hPa / 1013.25f, 1.f / 5.255f));

  // Subtract the bias
  dev->bar_alt_m -= dev->bar_bias;
}
