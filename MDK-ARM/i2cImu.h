#ifndef __I2CIMU_H__
#define __I2CIMU_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define NN 6.103515625e-5
#define F 1/(1+10/200)

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
HAL_StatusTypeDef i2cWrite(uint8_t mode, uint16_t reg, uint8_t data);
uint8_t i2cRead(uint8_t mode, uint16_t reg);
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */