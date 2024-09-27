#ifndef __ADXL345_H
#define __ADXL345_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define ADXL345_I2C_ADDR 0x53
#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37

extern I2C_HandleTypeDef hi2c1;

extern int16_t x;
extern int16_t y;
extern int16_t z;

void ADXL345_Init(void);
HAL_StatusTypeDef ADXL345_ReadAxis(int16_t *x, int16_t *y, int16_t *z);

#ifdef __cplusplus
}
#endif

#endif /* __SR04_H */
