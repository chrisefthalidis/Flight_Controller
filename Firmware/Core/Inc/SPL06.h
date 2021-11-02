/*
Title: SPL06-007 I2C Driver

Authors: Chris Efthalidis, Vangelis Epanomitis
*/

#ifndef SPL06_H
#define SPL06_H

#include "i2c.h"
#include "math.h"

// I2C address
#define SPL06_ADDR 0x77 << 1

// Registers
#define SPL06_REG_PSR_B2 0x00
#define SPL06_REG_PSR_B1 0x01
#define SPL06_REG_PSR_B0 0x02
#define SPL06_REG_TMP_B2 0x03
#define SPL06_REG_TMP_B1 0x04
#define SPL06_REG_TMP_B0 0x05
#define SPL06_REG_PRS_CFG 0x06
#define SPL06_REG_TMP_CFG 0x07
#define SPL06_REG_MEAS_CFG 0x08
#define SPL06_REG_CFG_REG 0x09
#define SPL06_REG_INT_STS 0x0A
#define SPL06_REG_FIFO_STS 0x0B
#define SPL06_REG_RESET 0x0C
#define SPL06_REG_ID 0x0D
#define SPL06_REG_COEF 0x13

typedef struct
{

    int32_t c00, c10;
    int16_t c01, c11, c20, c21, c30;
    float bar_bias, bar_alt_m;

} SPL06;

// Low-level functions
void SPL06_Read_Register(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
void SPL06_Read_Registers(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t length);
void SPL06_Write_Register(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
void SPL06_Write_Registers(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t length);

// Initialisation function
void SPL06_Initialise(SPL06 *dev);

// Calibration function
void SPL06_Calibrate(SPL06 *dev);

// Data acquisition function
void SPL06_Read(SPL06 *dev);

#endif
