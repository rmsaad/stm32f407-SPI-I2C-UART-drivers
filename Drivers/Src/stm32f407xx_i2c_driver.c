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

}

void I2C_DeInit(I2C_RegDef_t *pI2Cx){

}

