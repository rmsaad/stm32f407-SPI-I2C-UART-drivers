/*
 * stm32f407xx_gpio_driver.h
 *
 *      Author: Rami
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include <stdint.h>
#include "stm32f407xx.h"

/******************************************************************************/
/************************** Processor Specifics *******************************/

#define NVIC_ISER0			((_IO uint32_t*)0xE000E100)
#define NVIC_ISER1			((_IO uint32_t*)0xE000E104)
#define NVIC_ISER2			((_IO uint32_t*)0xE000E108)
#define NVIC_ISER3			((_IO uint32_t*)0xE000E10C)

#define NVIC_ICER0			((_IO uint32_t*)0xE000E180)
#define NVIC_ICER1			((_IO uint32_t*)0xE000E184)
#define NVIC_ICER2			((_IO uint32_t*)0xE000E188)
#define NVIC_ICER3			((_IO uint32_t*)0xE000E18C)

#define NVIC_PR_BASE		((_IO uint32_t*)0xE000E400)
#define NO_PR_BITS_IMP		4

#define NVIC_IRQ_PRIO_0		0
#define NVIC_IRQ_PRIO_1		1
#define NVIC_IRQ_PRIO_2		2
#define NVIC_IRQ_PRIO_3		3
#define NVIC_IRQ_PRIO_4		4
#define NVIC_IRQ_PRIO_5		5
#define NVIC_IRQ_PRIO_6		6
#define NVIC_IRQ_PRIO_7		7
#define NVIC_IRQ_PRIO_8		8
#define NVIC_IRQ_PRIO_9		9
#define NVIC_IRQ_PRIO_10	10
#define NVIC_IRQ_PRIO_11	11
#define NVIC_IRQ_PRIO_12	12
#define NVIC_IRQ_PRIO_13	13
#define NVIC_IRQ_PRIO_14	14
#define NVIC_IRQ_PRIO_15	15
/******************************************************************************/
/****************************** GPIO Macros ***********************************/

/*GPIO Pin Numbers*/
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/* @GPIO_PIN_MODES
 * GPIO Pin Modes*/
#define GPIO_MODE_IN 		0
#define	GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FL  	4
#define GPIO_MODE_IT_RI  	5
#define GPIO_MODE_IT_RI_FL  6

/* @GPIO_PIN_TYPES
 * GPIO Pin Output Types*/
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/* @GPIO_PIN_SPEEDS
 * GPIO Pin Output Speeds*/
#define GPIO_SPEED_LOW		0
#define	GPIO_SPEED_MID		1
#define	GPIO_SPEED_FAST		2
#define	GPIO_SPEED_HIGH		3

/* @GPIO_PIN_CONTROL
 * GPIO Pin Pull Up Pull Down Configurations*/
#define	GPIO_NO_PUPD		0
#define GPIO_PU				1
#define	GPIO_PD				2

/*Reset GPIOx peripherals*/
#define	GPIOA_REG_RESET() 	do {(RCC->RCC_AHB1RSTR |= (1 << 0));  (RCC->RCC_AHB1RSTR &= ~(1 << 0));}  while(0)
#define	GPIOB_REG_RESET() 	do {(RCC->RCC_AHB1RSTR |= (1 << 1));  (RCC->RCC_AHB1RSTR &= ~(1 << 1));}  while(0)
#define	GPIOC_REG_RESET() 	do {(RCC->RCC_AHB1RSTR |= (1 << 2));  (RCC->RCC_AHB1RSTR &= ~(1 << 2));}  while(0)
#define	GPIOD_REG_RESET() 	do {(RCC->RCC_AHB1RSTR |= (1 << 3));  (RCC->RCC_AHB1RSTR &= ~(1 << 3));}  while(0)
#define	GPIOE_REG_RESET() 	do {(RCC->RCC_AHB1RSTR |= (1 << 4));  (RCC->RCC_AHB1RSTR &= ~(1 << 4));}  while(0)
#define	GPIOF_REG_RESET() 	do {(RCC->RCC_AHB1RSTR |= (1 << 5));  (RCC->RCC_AHB1RSTR &= ~(1 << 5));}  while(0)
#define	GPIOG_REG_RESET() 	do {(RCC->RCC_AHB1RSTR |= (1 << 6));  (RCC->RCC_AHB1RSTR &= ~(1 << 6));}  while(0)
#define	GPIOH_REG_RESET() 	do {(RCC->RCC_AHB1RSTR |= (1 << 7));  (RCC->RCC_AHB1RSTR &= ~(1 << 7));}  while(0)
#define	GPIOI_REG_RESET() 	do {(RCC->RCC_AHB1RSTR |= (1 << 8));  (RCC->RCC_AHB1RSTR &= ~(1 << 8));}  while(0)
#define	GPIOJ_REG_RESET() 	do {(RCC->RCC_AHB1RSTR |= (1 << 9));  (RCC->RCC_AHB1RSTR &= ~(1 << 9));}  while(0)
#define	GPIOK_REG_RESET() 	do {(RCC->RCC_AHB1RSTR |= (1 << 10)); (RCC->RCC_AHB1RSTR &= ~(1 << 10));} while(0)

/******************************************************************************/
/************** GPIO Configuration and Handling Structures ********************/

typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;				/*Modes available @GPIO_PIN_MODES*/
	uint8_t GPIO_PinSpeed;				/*Speeds available @GPIO_PIN_SPEEDS*/
	uint8_t GPIO_PinPuPdControl;		/*Configurations available @GPIO_PIN_CONTROL*/
	uint8_t GPIO_PinOPType;				/*Types available @GPIO_PIN_TYPES*/
	uint8_t GPIO_PinAltFunMode;			/**/
}GPIO_PinConfig_t;

typedef struct{
	GPIO_RegDef_t *pGPIOx;				/*holds base address of GPIO port for the pin*/
	GPIO_PinConfig_t GPIO_PinConfig;	/*holds the GPIO pin configurations*/
}GPIO_Handle_t;

/******************************************************************************/
/*************************** Function ProtoTypes ******************************/

/*Peripheral Clock setup*/
void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx, uint8_t en_di);

/*Initiation De-initiation*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*Read/write Data*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,  uint16_t Value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*Interrupt Configuration and Handling*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t en_di);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
