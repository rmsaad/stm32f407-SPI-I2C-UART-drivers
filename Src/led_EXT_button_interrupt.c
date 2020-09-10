/*
 * led_EXT_button_interrupt.c
 *
 *      Author: Rami
 */

#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(uint16_t kilo_cycles){
	uint16_t c, d;
   for (c = 1; c <= kilo_cycles; c++)
	   for (d = 1; d <= 1000; d++)
	   {}
}

int main(void)
{
	GPIO_Handle_t gpioLED;
	memset(&gpioLED, 0, sizeof(gpioLED)); 	//set member elements to zero
	gpioLED.pGPIOx = GPIOD;
	gpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	gpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Handle_t gpioBTN;
	memset(&gpioBTN, 0, sizeof(gpioBTN));	//set member elements to zero
	gpioBTN.pGPIOx = GPIOB;
	gpioBTN.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	gpioBTN.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FL;
	gpioBTN.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioBTN.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PCLKControl(GPIOD, ENABLE);
	GPIO_PCLKControl(GPIOB, ENABLE);

	GPIO_Init(&gpioLED);
	GPIO_Init(&gpioBTN);

	//IRQ Config
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIO_15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, EN);

	// circuit shorts when button is pressed
	while(1);
}

void EXTI15_10_IRQHandler(void){
	delay(1000);
	GPIO_IRQHandling(GPIO_PIN_12);
	GPIO_TogglePin(GPIOD, GPIO_PIN_13);
}

