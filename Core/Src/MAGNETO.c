#include "MAGNETO.h" // Replace with your actual header file for MAGNETO

int16_t mx, my, mz;

void MAGNETO_Init(void) {
    uint8_t data;

    // Set the configuration register A for 8-average, 15 Hz default, normal measurement
    data = 0x70;
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(&hi2c1, MAGNETO_I2C_ADDR, MAGNETO_CONFIG_A, 1, &data, 1, HAL_MAX_DELAY);

    // Set the configuration register B for gain = 1090 (default)
    data = 0xA0;
    status = HAL_I2C_Mem_Write(&hi2c1, MAGNETO_I2C_ADDR, MAGNETO_CONFIG_B, 1, &data, 1, HAL_MAX_DELAY);

    // Set the mode register to continuous measurement mode
    data = 0x00;
    status = HAL_I2C_Mem_Write(&hi2c1, MAGNETO_I2C_ADDR, MAGNETO_MODE, 1, &data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MAGNETO_ReadAxis(int16_t *mx, int16_t *my, int16_t *mz) {
    uint8_t data[6];

    // Read 6 bytes of magnetometer data
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&hi2c1, MAGNETO_I2C_ADDR, MAGNETO_DATAX0, 1, data, 6, HAL_MAX_DELAY);

    // Convert the data to 16-bit integers
    *mx = (int16_t)((data[0] << 8) | data[1]);
    *my = (int16_t)((data[4] << 8) | data[5]);
    *mz = (int16_t)((data[2] << 8) | data[3]);

    return status;
}
