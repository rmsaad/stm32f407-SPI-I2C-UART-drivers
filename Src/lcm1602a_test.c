/*
 * lcm1602a_test.c
 *
 *      Author: Rami
 */

#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f407xx_lcm1602a_driver.h"

int main(void)
{
	/*Data Pins 0-7*/
	GPIO_RegDef_t *pDataPorts[8] = {GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE};
	uint8_t	dataPins[8] = {GPIO_PIN_7,  GPIO_PIN_8,  GPIO_PIN_9,  GPIO_PIN_10,
						   GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14};

	/*Control Pins ~ RS RW E*/
	GPIO_RegDef_t *pControlPorts[3] = {GPIOC, GPIOC, GPIOC};
	uint8_t controlPins[3] = {GPIO_PIN_1, GPIO_PIN_5, GPIO_PIN_4};

	/*enable all the GPIO ports for 8 bit transmission*/
	LCM1602a_Data8_GPIOInit(pDataPorts, dataPins, pControlPorts, controlPins);

	/*Initialize the display*/
	LCM1602a_Write8_Data(0b00110000, 0, 0);
	LCM1602a_Write8_Data(0b00001110, 0, 0);
	LCM1602a_Write8_Data(0b00000110, 0, 0);

	/*clear the display*/
	LCM1602a_Write8_Data(0b00000001, 0, 0);

	/*Write Message to Display*/
	LCM1602a_Write8_Message((char*)"Hello Test 2");

	/*hang forever */
	while(1);

	return 0;
}
