#include "adxl345.h"

int16_t x;
int16_t y;
int16_t z;

void ADXL345_Init(void) {
    uint8_t data = 0;
    uint8_t address = 0;
    volatile check = 0;
// 30 magneto 83 acc gyro 104
    for (uint16_t i = 0 ;i <157; i++) {
		HAL_StatusTypeDef status3 = HAL_I2C_Mem_Read(&hi2c1, (i << 1) , 0x00, 1, &data, 1, HAL_MAX_DELAY);
		if (status3 == HAL_OK) {
			check = 1;
		}
    }

    // Set data format to full resolution with +/- 2g range
    data = 0x08; // Full resolution, +/- 2g
    HAL_StatusTypeDef status1 = HAL_I2C_Mem_Write(&hi2c1, ADXL345_I2C_ADDR, ADXL345_DATA_FORMAT, 21, &data, 1, HAL_MAX_DELAY);

    // Set the power control to measurement mode
    data = 0x08; // Measurement mode
    HAL_StatusTypeDef status2 = HAL_I2C_Mem_Write(&hi2c1, ADXL345_I2C_ADDR, ADXL345_POWER_CTL, 1, &data, 1, HAL_MAX_DELAY);

}

HAL_StatusTypeDef ADXL345_ReadAxis(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t data[6];

    // Read 6 bytes of acceleration data
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&hi2c1, ADXL345_I2C_ADDR, ADXL345_DATAX0, 1, data, 6, HAL_MAX_DELAY);

    // Convert the data to 16-bit integers
    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *z = (int16_t)((data[5] << 8) | data[4]);
    return status;
}
