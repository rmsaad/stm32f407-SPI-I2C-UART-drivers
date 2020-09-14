/*
 * stm32f407_lcm1602a_driver.c
 *
 *  Created on: Sep 11, 2020
 *      Author: Rami
 */

#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_lcm1602a_driver.h"

static GPIO_Handle_t *pGpioData[8];
static GPIO_Handle_t *pGpioControl[3];

/*private function prototypes*/
void LCM1602a_Hang_Busy_Flag();


void LCM1602a_Data8_GPIOInit(GPIO_RegDef_t *pDataPorts[8], uint8_t dataPins[8], GPIO_RegDef_t *pControlPorts[3], uint8_t controlPins[3], GPIO_Handle_t gpioData[8], GPIO_Handle_t gpioControl[3]){
	/*initialize Data Pins*/
	for(int i = 0; i < DATA_8; i++){
		pGpioData[i] = &gpioData[i];					// copying into static pointer
		memset(&gpioData[i], 0, sizeof(gpioData[i])); 	//set member elements to zero
		gpioData[i].pGPIOx = pDataPorts[i];
		gpioData[i].GPIO_PinConfig.GPIO_PinNumber = dataPins[i];
		gpioData[i].GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		gpioData[i].GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
		gpioData[i].GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
		gpioData[i].GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
		GPIO_Init(&gpioData[i]);
	}

	/*Initialize Control Pins*/
	for(int i = 0; i < CONTROL_PIN_COUNT; i++){
		pGpioControl[i] =  &gpioControl[i];						// copying into static pointer
		memset(&gpioControl[i], 0, sizeof(gpioControl[i])); 	//set member elements to zero
		gpioControl[i].pGPIOx = pControlPorts[i];
		gpioControl[i].GPIO_PinConfig.GPIO_PinNumber = controlPins[i];
		gpioControl[i].GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		gpioControl[i].GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
		gpioControl[i].GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
		gpioControl[i].GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
		GPIO_Init(&gpioControl[i]);
	}
}


void LCM1602a_Write8_Data(uint8_t dataValues, uint8_t RS, uint8_t RW){

	LCM1602a_Hang_Busy_Flag();

	// write to data lines
	for(int i = 0; i < DATA_8; i++){
		GPIO_WriteToOutputPin(pGpioData[i]->pGPIOx, pGpioData[i]->GPIO_PinConfig.GPIO_PinNumber, ((dataValues >> i) & 1));
	}

	// write to control lines RS, RW
	GPIO_WriteToOutputPin(pGpioControl[0]->pGPIOx, pGpioControl[0]->GPIO_PinConfig.GPIO_PinNumber, RS);
	GPIO_WriteToOutputPin(pGpioControl[1]->pGPIOx, pGpioControl[1]->GPIO_PinConfig.GPIO_PinNumber, RW);

	// set E to High
	GPIO_WriteToOutputPin(pGpioControl[2]->pGPIOx, pGpioControl[2]->GPIO_PinConfig.GPIO_PinNumber, HIGH);

	// reset all control pins
	for(int i = CONTROL_PIN_COUNT - 1; i >= 0; i--){
		GPIO_WriteToOutputPin(pGpioControl[i]->pGPIOx, pGpioControl[i]->GPIO_PinConfig.GPIO_PinNumber, LOW);
	}

}

void LCM1602a_Write8_Message(char *Message){

	// find length of the message
	uint16_t Len = (uint16_t)strlen(Message);

	//write message to display
	for(int i = 0; i < Len; i++){
		LCM1602a_Write8_Data((int)*Message, 1, 0);
		Message++;
	}
}

void LCM1602a_Hang_Busy_Flag(){

	/* temporary storage*/
	uint32_t temp = 0;

	// first set D7 to input
	temp = (GPIO_MODE_IN << (2 * pGpioData[7]->GPIO_PinConfig.GPIO_PinNumber));
	pGpioData[7]->pGPIOx->MODER &= ~(0x3 << (2 * pGpioData[7]->GPIO_PinConfig.GPIO_PinNumber));
	pGpioData[7]->pGPIOx->MODER |= temp;

	// hang till Busy flag is Low
	while(1){
		// Set RW and E
		GPIO_WriteToOutputPin(pGpioControl[1]->pGPIOx, pGpioControl[1]->GPIO_PinConfig.GPIO_PinNumber, HIGH);
		GPIO_WriteToOutputPin(pGpioControl[2]->pGPIOx, pGpioControl[2]->GPIO_PinConfig.GPIO_PinNumber, HIGH);

		// read Data 7 pin, if 0 set E back to LOW and stop hanging
		if(GPIO_ReadFromInputPin(pGpioData[7]->pGPIOx, pGpioData[7]->GPIO_PinConfig.GPIO_PinNumber) == 0){
			GPIO_WriteToOutputPin(pGpioControl[2]->pGPIOx, pGpioControl[2]->GPIO_PinConfig.GPIO_PinNumber, LOW);
			break;
		}
	}

	// set D7 back to ouput
	temp = (GPIO_MODE_OUT << (2 * pGpioData[7]->GPIO_PinConfig.GPIO_PinNumber));
	pGpioData[7]->pGPIOx->MODER &= ~(0x3 << (2 * pGpioData[7]->GPIO_PinConfig.GPIO_PinNumber));
	pGpioData[7]->pGPIOx->MODER |= temp;

}
