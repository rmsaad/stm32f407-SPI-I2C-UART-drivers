/*
 * i2c_driver.h
 *
 *  Created on: Dec 1, 2020
 *      Author: Rami
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include <stdint.h>
#include "stm32f407xx.h"

/******************************************************************************/
/****************************** I2C Macros ************************************/

/*SCL SPEED*/
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM4K	400000
#define I2C_SCL_SPEED_FM2K	200000

/*ACK Control*/
#define I2C_ACK_EN			1
#define I2C_ACK_DI			0

/*Duty Cycle*/
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/******************************************************************************/
/***************** I2C Configuration & Handle Structures **********************/

/*I2C Configuration Structure*/
typedef struct{
	uint32_t 	I2C_SCLSpeed;
	uint8_t 	I2C_DeviceAddress;
	uint8_t 	I2C_ACKControl;
	uint16_t	I2C_FMDutyCycle;
}I2C_Config_t;

/*I2C Handle Structure*/
typedef struct{
	I2C_RegDef_t *pI2Cx;	// base address of I2Cx peripheral
	I2C_Config_t I2C_Config;
}I2C_Handle_t;

/******************************************************************************/
/*************************** Function ProtoTypes ******************************/

/*Peripheral Clock setup*/
void I2C_PCLKControl(I2C_RegDef_t *pI2Cx, uint8_t en_di);

/*Initiation De-initiation*/
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*Data Send/Receive*/


/*IRQ Configuration and ISR Handling*/

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t en_di);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


/*Application Callback*/
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

/*Other Peripheral Control API*/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t en_di);
void I2C_SSIConfig(I2C_RegDef_t *pI2Cx, uint8_t en_di);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
