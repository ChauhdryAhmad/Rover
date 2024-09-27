// variables for receiving DATA
uint8_t DataX[2];
uint16_t Xaxis = 0;
uint8_t DataY[2];
uint16_t Yaxis = 0;
uint8_t DataZ[2];
uint16_t Zaxis = 0;

// adresses
// HMC5883l - ADDRESS
#define HMC5883l_ADDRESS  0x1E//(0x1E << 1)
// CONTROL REG A
#define HMC5883l_Enable_A (0x78)
// CONTROL REG B
#define HMC5883l_Enable_B (0xA0)
// MODE REGISTER
#define HMC5883l_MR (0x00)
// HMC5883l - MSB / LSB ADDRESSES
#define HMC5883l_ADD_DATAX_MSB (0x03)
#define HMC5883l_ADD_DATAX_LSB (0x04)
#define HMC5883l_ADD_DATAZ_MSB (0x05)
#define HMC5883l_ADD_DATAZ_LSB (0x06)
#define HMC5883l_ADD_DATAY_MSB (0x07)
#define HMC5883l_ADD_DATAY_LSB (0x08)
// SUM (MSB + LSB) DEFINE
#define HMC5883l_ADD_DATAX_MSB_MULTI (HMC5883l_ADD_DATAX_MSB | 0x80)
#define HMC5883l_ADD_DATAY_MSB_MULTI (HMC5883l_ADD_DATAY_MSB | 0x80)
#define HMC5883l_ADD_DATAZ_MSB_MULTI (HMC5883l_ADD_DATAZ_MSB | 0x80)
