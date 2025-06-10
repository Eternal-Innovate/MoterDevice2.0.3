#include "i2cImu.h"

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

HAL_StatusTypeDef i2cWrite(uint8_t mode, uint16_t reg, uint8_t data)
{
	uint8_t txdata[] = {data};
	I2C_HandleTypeDef *tmp;
	
	if(mode==0xd0) tmp = &hi2c1;
	else if(mode==0x3C) tmp = &hi2c2;
	
	return HAL_I2C_Mem_Write(
    tmp,                         
    mode,                           
    reg,                            
    I2C_MEMADD_SIZE_8BIT,           
    txdata,                         
    sizeof(txdata),                 
    100                              
	);
}
uint8_t i2cRead(uint8_t mode, uint16_t reg)
{
	uint8_t rxData[1];
	I2C_HandleTypeDef *tmp;
	
	if(mode==0xd0) tmp = &hi2c1;
	else if(mode==0x3C) tmp = &hi2c2;
	
	HAL_StatusTypeDef status2 = HAL_I2C_Mem_Read(
    tmp,    
		mode,
    reg,                            
    I2C_MEMADD_SIZE_8BIT,           
    rxData,                         
    sizeof(rxData),                 
    100                             
	);	
	if(status2==0x01)
	{
		return -1;
	}
	return rxData[0];
		
}



void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}