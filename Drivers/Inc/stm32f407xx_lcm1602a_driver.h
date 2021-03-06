/*
 * stm32f407_lcm1602a_driver.h
 *
 *      Author: Rami
 */

#ifndef INC_STM32F407XX_LCM1602A_DRIVER_H_
#define INC_STM32F407XX_LCM1602A_DRIVER_H_

#include <stdint.h>
#include "stm32f407xx.h"

/******************************************************************************/
/**************************** Generic Macros **********************************/

#define DATA_8					8
#define DATA_4					4
#define CONTROL_PIN_COUNT		3

/******************************************************************************/
/************************* Function prototypes ********************************/

/*Initiation De-initiation*/
void LCM1602a_Data8_GPIOInit(GPIO_RegDef_t *pDataPorts[8], uint8_t dataPins[8], GPIO_RegDef_t *pControlPorts[3], uint8_t controlPins[3]);
void LCM1602a_Data8_GPIODeInit();

/*Read/write Data*/
void LCM1602a_Write8_Data(uint8_t dataValues, uint8_t RS, uint8_t RW);
void LCM1602a_Write8_Message(char *Message);

void LCM1602a_Write4_Data(uint8_t dataValues, uint8_t RS, uint8_t RW);
void LCM1602a_Write4_Message(char *Message);

#endif /* INC_STM32F407XX_LCM1602A_DRIVER_H_ */
