/*
 * lcm1602a_test.c
 *
 *      Author: Rami
 */

#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f407xx_lcm1602a_driver.h"

void delay(uint16_t kilo_cycles){
	uint16_t c, d;
   for (c = 1; c <= kilo_cycles; c++)
	   for (d = 1; d <= 1000; d++)
	   {}
}

int main(void)
{

/*	GPIO_Handle_t gpioBTN;
	gpioBTN.pGPIOx = GPIOA;
	gpioBTN.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	gpioBTN.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioBTN.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	gpioBTN.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&gpioBTN);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRIO_15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, EN);*/

	/*Data Pins 0-7*/
	GPIO_RegDef_t *pDataPorts[8] = {GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE};
	uint8_t	dataPins[8] = {GPIO_PIN_7,  GPIO_PIN_8,  GPIO_PIN_9,  GPIO_PIN_10,
						   GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14};

	/*Control Pins ~ RS RW E*/
	GPIO_RegDef_t *pControlPorts[3] = {GPIOC, GPIOC, GPIOC};
	uint8_t controlPins[3] = {GPIO_PIN_1, GPIO_PIN_5, GPIO_PIN_4};

	/*enable all the GPIO ports for 8 bit transmission*/
	LCM1602a_Data8_GPIOInit(pDataPorts, dataPins, pControlPorts, controlPins);

	/*Initialize the display 8 bit mode*/
	//LCM1602a_Write8_Data(0b00110000, 0, 0);
	//LCM1602a_Write8_Data(0b00001110, 0, 0);
	//LCM1602a_Write8_Data(0b00000110, 0, 0);

	/*clear the display*/
	//LCM1602a_Write8_Data(0b00000001, 0, 0);

	/*Initialize the display 4 bit mode*/
	LCM1602a_Write4_Data(0b00100000, 0, 0);
	LCM1602a_Write4_Data(0b00100000, 0, 0);
	LCM1602a_Write4_Data(0b00001110, 0, 0);
	LCM1602a_Write4_Data(0b00000110, 0, 0);

	/*clear the display*/
	LCM1602a_Write4_Data(0b00000001, 0, 0);

	/*Write Message to Display*/

	while(1){
	LCM1602a_Write4_Data(0b00000010, 0, 0);
	LCM1602a_Write4_Message((char*)"just work       ");
	delay(1000);
	LCM1602a_Write4_Data(0b00000010, 0, 0);
	LCM1602a_Write4_Message((char*)"just wor         ");
	delay(1000);
	}
	/*hang forever*/
	while(1);

	return 0;
}

/*void EXTI0_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_0);
	LCM1602a_Write4_Data(0b00000001, 0, 0);
}*/
