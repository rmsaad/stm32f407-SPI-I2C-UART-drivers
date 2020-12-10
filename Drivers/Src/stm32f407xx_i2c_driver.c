/*
 * i2c_driver.c
 *
 *  Created on: Dec 1, 2020
 *      Author: Rami
 */

#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f407xx_i2c_driver.h"

/*private function prototypes*/


/******************************************************************************/
/************************** Peripheral Clock Setup ****************************/

void I2C_PCLKControl(I2C_RegDef_t *pI2Cx, uint8_t en_di){
	if(en_di == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}

	}else{
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}
}

/******************************************************************************/
/********************** GPIO Initiation De-initiation *************************/

void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t tempreg = 0;

	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;

}

void I2C_DeInit(I2C_RegDef_t *pI2Cx){

}

/******************************************************************************/
/**************************** Data Send/Receive *******************************/


/******************************************************************************/
/******************* IRQ Configuration and ISR Handling ***********************/

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t en_di){

}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

}

/******************************************************************************/
/*********************** Other Peripheral Control API *************************/

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){

}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t en_di){

}

void I2C_SSIConfig(I2C_RegDef_t *pI2Cx, uint8_t en_di){

}
