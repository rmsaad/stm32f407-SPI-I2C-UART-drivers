/*
 * lcm1602a_test.c
 *
 *      Author: Rami
 */

#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f407xx_lcm1602a_driver.h"
#include <string.h>

char * LargeText = "        3030.mp3        ";
int iCursor = 0;

// Speaker
uint8_t Speaker[8] = {
	0b00001,
	0b00011,
	0b00111,
	0b11111,
	0b11111,
	0b00111,
	0b00011,
	0b00001
};

// Sound 1
uint8_t Sound_1[8] = {
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b11000,
	0b11000
};

// Sound 2
uint8_t Sound_2[8] = {
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00011,
	0b00011,
	0b11011,
	0b11011
};

// Sound 3
uint8_t Sound_3[8] = {
	0b00000,
	0b00000,
	0b11000,
	0b11000,
	0b11000,
	0b11000,
	0b11000,
	0b11000
};

// Sound 4
uint8_t Sound_4[8] = {
	0b00011,
	0b00011,
	0b11011,
	0b11011,
	0b11011,
	0b11011,
	0b11011,
	0b11011
};

void delay(uint16_t kilo_cycles){
	uint16_t c, d;
   for (c = 1; c <= kilo_cycles; c++)
	   for (d = 1; d <= 1000; d++)
	   {}
}

void create_char(uint8_t location, uint8_t charmap[]){
	location &= 0x7; // we only have 8 locations 0-7
	LCM1602a_Write8_Data(0x40 | (location << 3), 0, 0);

	for (int i=0; i<8; i++) {
		LCM1602a_Write8_Data(charmap[i], 1, 0);
	}
}

void textwrap2(char* text){

}

void textwrap(char* text){

	int iLenOfLargeText = strlen(LargeText);
	if (iCursor == (iLenOfLargeText - 1) ){                                             // Reset variable for rollover effect.
		iCursor = 0;
	}

	LCM1602a_Write8_Data(0b00000010, 0, 0);
	//LCM1602a_Write8_Data(0b11000000, 0, 0);
	if(iCursor < iLenOfLargeText - 16){                                                 // This loop exicuted for normal 16 characters.
		for (int iChar = iCursor; iChar < iCursor + 16 ; iChar++){
			LCM1602a_Write8_Data((int)LargeText[iChar], 1, 0);
		}
	}

	else{
		for (int iChar = iCursor; iChar < (iLenOfLargeText - 1) ; iChar++){               //  This code takes care of printing charecters of current string.
		LCM1602a_Write8_Data((int)LargeText[iChar], 1, 0);
		}

		for (int iChar = 0; iChar <= 16 - (iLenOfLargeText - iCursor); iChar++){           //  Reamining charecter will be printed by this loop.
		LCM1602a_Write8_Data((int)LargeText[iChar], 1, 0);
		}
	}

	iCursor++;
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

	create_char(0, Speaker);
	create_char(1, Sound_2);
	create_char(2, Sound_4);
	/*Initialize the display 8 bit mode*/
	//LCM1602a_Write8_Data(0b00110000, 0, 0);	// 1 line
	LCM1602a_Write8_Data(0b00111000, 0, 0);	// 2 line
	LCM1602a_Write8_Data(0b00001110, 0, 0);
	LCM1602a_Write8_Data(0b00000110, 0, 0);

	/*clear the display*/
	LCM1602a_Write8_Data(0b00000001, 0, 0);



	//LCM1602a_Write8_Data(0b00000001, 1, 0);
	//LCM1602a_Write8_Data(0b00000010, 1, 0);

	/*Initialize the display 4 bit mode*/
	/*LCM1602a_Write4_Data(0b00100000, 0, 0);
	LCM1602a_Write4_Data(0b00100000, 0, 0);
	LCM1602a_Write4_Data(0b00001110, 0, 0);
	LCM1602a_Write4_Data(0b00000110, 0, 0);

	clear the display
	LCM1602a_Write4_Data(0b00000001, 0, 0);
*/

	while(1){
		textwrap("3030.mp3");
		delay(2000);
	}

//	while(1){
//	LCM1602a_Write8_Data(0b00000010, 0, 0);
//	LCM1602a_Write8_Message((char*)"Hello mll        ");
//	delay(2000);
//	LCM1602a_Write8_Data(0b00000010, 0, 0);
//	LCM1602a_Write8_Message((char*)"Hello all        ");
//	delay(2000);
//	}
	/*hang forever*/
	while(1);

	return 0;
}

/*void EXTI0_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_0);
	LCM1602a_Write4_Data(0b00000001, 0, 0);
}*/
