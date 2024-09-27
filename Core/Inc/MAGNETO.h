#ifndef MAGNETO_H
#define MAGNETO_H

#include "stm32f4xx_hal.h"
extern I2C_HandleTypeDef hi2c1;

// MAGNETO I2C address
#define MAGNETO_I2C_ADDR      (0x1E)  // 7-bit address shifted for STM32 HAL

// MAGNETO Register Addresses
#define MAGNETO_CONFIG_A      0x00  // Configuration Register A
#define MAGNETO_CONFIG_B      0x01  // Configuration Register B
#define MAGNETO_MODE          0x02  // Mode Register
#define MAGNETO_DATAX0        0x03  // Data Output X MSB Register
#define MAGNETO_DATAX1        0x04  // Data Output X LSB Register
#define MAGNETO_DATAZ0        0x05  // Data Output Z MSB Register
#define MAGNETO_DATAZ1        0x06  // Data Output Z LSB Register
#define MAGNETO_DATAY0        0x07  // Data Output Y MSB Register
#define MAGNETO_DATAY1        0x08  // Data Output Y LSB Register
#define MAGNETO_STATUS        0x09  // Status Register
#define MAGNETO_IDENT_A       0x0A  // Identification Register A
#define MAGNETO_IDENT_B       0x0B  // Identification Register B
#define MAGNETO_IDENT_C       0x0C  // Identification Register C
extern int16_t mx;
extern int16_t my;
extern int16_t mz;
// Function Prototypes
void MAGNETO_Init(void);
HAL_StatusTypeDef MAGNETO_ReadAxis(int16_t *mx, int16_t *my, int16_t *mz);

#endif // MAGNETO_H
