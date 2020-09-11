/*
 * led_OD_toggle.c
 *
 *      Author: Rami
 */

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
	gpioLED.pGPIOx = GPIOD;
	gpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	gpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&gpioLED);

	while(1){
		GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		delay(1000);
	}
}
