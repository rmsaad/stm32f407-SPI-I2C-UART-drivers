/*
 *
 *
 *      Author: Rami
 */

#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

/******************************************************************************/
/************************** Peripheral Clock Setup ****************************/

/**
 * @fn			: GPIO_PCLKControl
 *
 * @brief		: enables or disables the peripheral clock for a given GPIO port
 *
 * @param[in]	: base address of the GPIOx peripheral
 * @param[in]	: enable(1) or disable(0)
 *
 * @return		: none
 */
void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx, uint8_t en_di){
	if(en_di == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}else if(pGPIOx == GPIOJ){
			GPIOJ_PCLK_EN();
		}else if(pGPIOx == GPIOK){
			GPIOK_PCLK_EN();
		}

	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}else if(pGPIOx == GPIOJ){
			GPIOJ_PCLK_DI();
		}else if(pGPIOx == GPIOK){
			GPIOK_PCLK_DI();
		}
	}

}

/******************************************************************************/
/********************** GPIO Initiation De-initiation *************************/

/**
 * @fn			: GPIO_Init
 *
 * @brief		: initializes the GPIOx peripheral
 *
 * @param[in]	: pointer to the user create handle structure
 *
 * @return		: none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	/* temporary storage*/
	uint32_t temp = 0;

	//enable the PCLK
	GPIO_PCLKControl(pGPIOHandle->pGPIOx, EN);

	/*initialize mode register*/
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));						//clearing
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;

	}else{
		// interrupt modes
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FL){
			// configure the falling trigger selection register
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear RTSR BIT because it might be set
			EXTI->EXTI_RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RI){
			// configure the rising trigger selection register
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear FTSR BIT because it might be set
			EXTI->EXTI_FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RI_FL){
			// configure both the falling and rising trigger selection registers
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASE_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();

		SYSCFG->SYSCFG_EXTICR[temp1] = portcode << (temp2 * 4);

		// enable the exti interrupt delivery using interrupt mask register
		EXTI->EXTI_IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	/*initialize speed register*/
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	/*initialize pull up pull down register*/
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	/*initialize output type register*/
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	/*initialize Alt FN register*/
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint32_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/**
 * @fn			: GPIO_DeInit
 *
 * @brief		: deinitialize register of GPIOx peripheral
 *
 * @param[in]	: base address of the GPIOx peripheral
 *
 * @return		: none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}else if(pGPIOx == GPIOJ){
		GPIOJ_REG_RESET();
	}else if(pGPIOx == GPIOK){
		GPIOK_REG_RESET();
	}
}

/******************************************************************************/
/*************************** Read and Write Data ******************************/

/**
 * @fn			: GPIO_ReadFromInputPin
 *
 * @brief		: reads state of specified GPIOx pin
 *
 * @param[in]	: base address of the GPIO peripheral
 * @param[in]	: GPIOx pin number
 *
 * @return		: state of specified GPIOx pin
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/**
 * @fn			: GPIO_ReadFromInputPort
 *
 * @brief		: reads entire GPIOx port register
 *
 * @param[in]	: base address of the GPIO peripheral
 *
 * @return		: state of specified GPIOx port
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/**
 * @fn			: GPIO_WriteToOutputPin
 *
 * @brief		: writes a bit of data to specified GPIOx output pin
 *
 * @param[in]	: base address of the GPIO peripheral
 * @param[in]	: GPIOx pin number
 * @param[in]	: data to write to GPIOx pin
 *
 * @return		: none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |= ( 1 << PinNumber);
	}else{
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}

/**
 * @fn			: GPIO_WriteToOutputPort
 *
 * @brief		: writes data to GPIOx output port
 *
 * @param[in]	: base address of the GPIOx peripheral
 * @param[in]	: data to write to output port
 *
 * @return		: none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,  uint16_t Value){
	pGPIOx->ODR  = Value;
}

/**
 * @fn			: GPIO_TogglePin
 *
 * @brief		: Toggles specified GPIOx pin
 *
 * @param[in]	: base address of the GPIO peripheral
 * @param[in]	: GPIOx pin number
 *
 * @return		: none
 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR = pGPIOx->ODR ^ (1 << PinNumber);
}

/******************************************************************************/
/******************* Interrupt Configuration and Handling *********************/

/**
 * @fn			: GPIO_IRQInterruptConfig
 *
 * @brief		: Configure IRQ number for GPIO pin
 *
 * @param[in]	: IRQ Number
 * @param[in]	: IRQ Priority
 * @param[in]	: enable (1) or disable (0)
 *
 * @return		: none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t en_di){

	if(en_di){
		if(IRQNumber <= 31){								/*ISER0 Register*/
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){			/*ISER1 Register*/
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96){		/*ISER2 Register*/
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}

	}else{
		if(IRQNumber <= 31){								/*ICER0 Register*/
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){			/*ICER1 Register*/
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96){		/*ICER2 Register*/
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	// find ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shiftamt = (8 * iprx_section) + (8 - NO_PR_BITS_IMP);
	*(NVIC_PR_BASE + iprx) |= (IRQPriority << shiftamt);
}

/**
 * @fn			: GPIO_IRQHandling
 *
 * @brief		: processes GPIO interrupts
 *
 * @param[in]	: GPIOx pin number
 *
 * @return		: none
 */
void GPIO_IRQHandling(uint8_t PinNumber){
	//clear the exti pr register with respect to the pin number
	if(EXTI->EXTI_PR & (1 << PinNumber)){		// if PR bit postion corresponding to pin number is set
		EXTI->EXTI_PR |= (1 << PinNumber);		// cleardddddddd
	}
}

