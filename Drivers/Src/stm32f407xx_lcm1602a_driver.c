/*
 * stm32f407_lcm1602a_driver.c
 *
 *      Author: Rami
 */

#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_lcm1602a_driver.h"

/*static handle structures for the Data and Control pins*/
static GPIO_Handle_t gpioData[8];														/*8 pin GPIO data handle structure*/
static GPIO_Handle_t gpioControl[3];													/*3 pin GPIO control handle structure*/
static GPIO_Handle_t *pgpioD7Busy;														/*pointer to the Data 7 pin GPIO handle structure*/

/*private function prototypes*/
static void LCM1602a_Hang_Busy_Flag();


/**
 * @fn			: LCM1602a_Data8_GPIOInit
 *
 * @brief		: initialize the GPIO pins for 8 lines of data communication and 3 lines for control by initializing their respective GPIO_Handle_t structures.
 *
 * @param[in]	: ordered array of the ports for data communication (from 0 - 7)
 * @param[in]	: ordered array GPIOx data pin numbers """
 * @param[in]	: ordered array of the ports for control information (RS, RW, and E)
 * @param[in]	: ordered array GPIOx control pin numbers """
 *
 * @return		: none
 */
void LCM1602a_Data8_GPIOInit(GPIO_RegDef_t *pDataPorts[8], uint8_t dataPins[8], GPIO_RegDef_t *pControlPorts[3], uint8_t controlPins[3]){

	/*initialize Data Pins*/
	for(int i = 0; i < DATA_8; i++){
		memset(&gpioData[i], 0, sizeof(gpioData[i])); 									/*set member elements to zero*/
		gpioData[i].pGPIOx = pDataPorts[i];												/*set data port*/
		gpioData[i].GPIO_PinConfig.GPIO_PinNumber = dataPins[i];						/*set data pin*/
		gpioData[i].GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;						/*set pin mode*/
		gpioData[i].GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;						/*set pin speed*/
		gpioData[i].GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;					/*set pin type*/
		gpioData[i].GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;					/*set pull up, pull down control*/
		GPIO_Init(&gpioData[i]);														/*initialize the GPIO handle structue*/
	}

	pgpioD7Busy = &gpioData[7];															/*pointer to Data 7 pin handle to simplify reading of busy flag*/

	/*Initialize Control Pins*/
	for(int i = 0; i < CONTROL_PIN_COUNT; i++){
		memset(&gpioControl[i], 0, sizeof(gpioControl[i])); 							/*set member elements to zero*/
		gpioControl[i].pGPIOx = pControlPorts[i];										/*set control port*/
		gpioControl[i].GPIO_PinConfig.GPIO_PinNumber = controlPins[i];					/*set control pin*/
		gpioControl[i].GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;						/*set pin mode*/
		gpioControl[i].GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;					/*set pin speed*/
		gpioControl[i].GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;					/*set pin type*/
		gpioControl[i].GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;				/*set pull up, pull down control*/
		GPIO_Init(&gpioControl[i]);														/*initialize the GPIO handle structue*/
	}
}

/**
 * @fn			: LCM1602a_Data8_GPIODeInit
 *
 * @brief		: de-initialize the 8 pin data and 3 pin control GPIO ports by setting the respective GPIO reset registers
 *
 *
 * @return		: none
 */
void LCM1602a_Data8_GPIODeInit(){

	for(int i = 0; i < DATA_8; i++){
		GPIO_DeInit(gpioData[i].pGPIOx);												/*reset GPIO data handles' ports*/
	}

	for(int i = 0; i < CONTROL_PIN_COUNT; i++){
		GPIO_DeInit(gpioControl[i].pGPIOx);												/*reset GPIO control handles' ports*/
	}
}

/**
 * @fn			: LCM1602a_Write8_Data
 *
 * @brief		: sets data pins, followed by RS and RW control pins with instruction code being sent to the LCD when the E pin is toggled
 *
 * @param[in]	: data values to be written to data pins with Data7 being MSB and Data0 being LSB
 * @param[in]	: RS control pin value
 * @param[in]	: RW control pin value
 *
 * @return		: none
 */
void LCM1602a_Write8_Data(uint8_t dataValues, uint8_t RS, uint8_t RW){

	LCM1602a_Hang_Busy_Flag();															/*hang until busy flag is reset*/

	for(int i = 0; i < DATA_8; i++){													/*write to data lines*/
		GPIO_WriteToOutputPin(gpioData[i].pGPIOx, gpioData[i].GPIO_PinConfig.GPIO_PinNumber, ((dataValues >> i) & 1));
	}

																						/*write to control lines RS, RW*/
	GPIO_WriteToOutputPin(gpioControl[0].pGPIOx, gpioControl[0].GPIO_PinConfig.GPIO_PinNumber, RS);
	GPIO_WriteToOutputPin(gpioControl[1].pGPIOx, gpioControl[1].GPIO_PinConfig.GPIO_PinNumber, RW);

																						/*set E to High*/
	GPIO_WriteToOutputPin(gpioControl[2].pGPIOx, gpioControl[2].GPIO_PinConfig.GPIO_PinNumber, HIGH);

	for(int i = CONTROL_PIN_COUNT - 1; i >= 0; i--){ 									/*reset all control pins*/
		GPIO_WriteToOutputPin(gpioControl[i].pGPIOx, gpioControl[i].GPIO_PinConfig.GPIO_PinNumber, LOW);
	}

}

/**
 * @fn			: LCM1602a_Write8_Message
 *
 * @brief		: Writes a char array to the LCD screen
 *
 * @param[in]	: char pointer to the ascii message to be displayed on the LCD screen
 *
 * @return		: none
 */
void LCM1602a_Write8_Message(char *Message){

	uint16_t Len = (uint16_t)strlen(Message);											/*find length of the message*/

	for(int i = 0; i < Len; i++){														/*write message to display*/
		LCM1602a_Write8_Data((int)*Message, 1, 0);
		Message++;
	}
}

/**
 * @fn			: LCM1602a_Hang_Busy_Flag
 *
 * @brief		: Hangs until the Data 7 pin returns the LOW value (Busy Flag)
 *
 *
 * @return		: none
 */
void LCM1602a_Hang_Busy_Flag(){

	uint32_t temp = 0;																	/* temporary variable to hold register value*/

	temp = (GPIO_MODE_IN << (2 * pgpioD7Busy->GPIO_PinConfig.GPIO_PinNumber));			/*first set D7 to input*/
	pgpioD7Busy->pGPIOx->MODER &= ~(0x3 << (2 * pgpioD7Busy->GPIO_PinConfig.GPIO_PinNumber));
	pgpioD7Busy->pGPIOx->MODER |= temp;

	while(1){																			/*hang till Busy flag is Low*/
																						/*set RW and E*/
		GPIO_WriteToOutputPin(gpioControl[1].pGPIOx, gpioControl[1].GPIO_PinConfig.GPIO_PinNumber, HIGH);
		GPIO_WriteToOutputPin(gpioControl[2].pGPIOx, gpioControl[2].GPIO_PinConfig.GPIO_PinNumber, HIGH);

																						/*read Data 7 pin, if 0 set E back to LOW and stop hanging*/
		if(GPIO_ReadFromInputPin(pgpioD7Busy->pGPIOx, pgpioD7Busy->GPIO_PinConfig.GPIO_PinNumber) == 0){
			GPIO_WriteToOutputPin(gpioControl[2].pGPIOx, gpioControl[2].GPIO_PinConfig.GPIO_PinNumber, LOW);
			break;
		}
	}

	temp = (GPIO_MODE_OUT << (2 * pgpioD7Busy->GPIO_PinConfig.GPIO_PinNumber));			/*set D7 back to output*/
	pgpioD7Busy->pGPIOx->MODER &= ~(0x3 << (2 * pgpioD7Busy->GPIO_PinConfig.GPIO_PinNumber));
	pgpioD7Busy->pGPIOx->MODER |= temp;

}

