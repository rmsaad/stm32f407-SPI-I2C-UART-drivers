/*
 * stm32f407xx.h
 *
 *      Author: Rami
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>


/******************************************************************************/
/**************************** Generic Macros **********************************/

#define ENABLE  		1
#define DISABLE 		0
#define HIGH			1
#define LOW				0
#define EN				ENABLE
#define DI				DISABLE
#define SET     		ENABLE
#define RESET   		DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET
#define _IO 			volatile

/******************************************************************************/
/************************** Base Address Macros *******************************/

/* FLASH AND SRAM MEMORY BASE ADDRESSES */
#define FLASH_BASE				0x08000000UL								/*Flash Memory Base Address*/
#define SRAM1_BASE  			0x20000000UL								/*SRAM1 Base Address*/
#define SRAM2_BASE  			0x2001C000UL								/*SRAM2 Base Address*/
#define ROM_BASE    			0x1FFF0000UL								/*ROM Base Address*/
#define SRAM_BASE      	 		SRAM1_BASE									/*SRAM Base Address*/


/* AHB and APB BASE ADDRESES */
#define PERIPH_BASE				0x40000000UL								/*Peripheral Bus Registers Base Address*/

#define APB1PERIPH_BASE 		PERIPH_BASE									/*APB1 Bus Registers Base Address*/
#define APB2PERIPH_BASE 		0x40010000UL								/*APB2 Bus Registers Base Address*/

#define AHB1PERIPH_BASE 		0x40020000UL								/*AHB1 Bus Registers Base Address*/
#define AHB2PERIPH_BASE 		0x50000000UL								/*AHB2 Bus Registers Base Address*/
#define AHB3PERIPH_BASE 		0xA0000000UL								/*AHB3 Bus Registers Base Address*/

/*APB1 PERIPHERAL BASE ADDRESSES*/
#define TIM2_BASE				(APB1PERIPH_BASE)							/*TIM2 Base Address*/
#define TIM3_BASE				(APB1PERIPH_BASE + 0x00400UL)				/*TIM3 Base Address*/
#define TIM4_BASE				(APB1PERIPH_BASE + 0x00800UL)				/*TIM4 Base Address*/
#define TIM5_BASE				(APB1PERIPH_BASE + 0x00C00UL)				/*TIM5 Base Address*/
#define TIM6_BASE				(APB1PERIPH_BASE + 0x01000UL)				/*TIM6 Base Address*/
#define TIM7_BASE				(APB1PERIPH_BASE + 0x01400UL)				/*TIM7 Base Address*/
#define TIM12_BASE				(APB1PERIPH_BASE + 0x01800UL)				/*TIM12 Base Address*/
#define TIM13_BASE				(APB1PERIPH_BASE + 0x01C00UL)				/*TIM13 Base Address*/
#define TIM14_BASE				(APB1PERIPH_BASE + 0x02000UL)				/*TIM14 Base Address*/
#define RTC_BKP_BASE			(APB1PERIPH_BASE + 0x02800UL)				/*RTC_BKP Base Address*/
#define WWDG_BASE      			(APB1PERIPH_BASE + 0x02C00UL)				/*WWDG Base Address*/
#define IWDG_BASE				(APB1PERIPH_BASE + 0x03000UL)				/*IWDG Base Address*/
#define I2S2ext_BASE			(APB1PERIPH_BASE + 0x03400UL)				/*I2S2ext Base Address*/
#define SPI2_I2S2_BASE  		(APB1PERIPH_BASE + 0x03800UL)				/*SPI2_I2S2 Base Address*/
#define SPI3_I2S3_BASE  		(APB1PERIPH_BASE + 0x03C00UL)				/*SPI3_I2S3 Base Address*/
#define I2S3ext_BASE			(APB1PERIPH_BASE + 0x04000UL)				/*I2S3ext Base Address*/
#define USART2_BASE				(APB1PERIPH_BASE + 0x04400UL)				/*USART2 Base Address*/
#define USART3_BASE				(APB1PERIPH_BASE + 0x04800UL)				/*USART3 Base Address*/
#define UART4_BASE				(APB1PERIPH_BASE + 0x04C00UL)				/*UART4 Base Address*/
#define UART5_BASE				(APB1PERIPH_BASE + 0x05000UL)				/*UART5 Base Address*/
#define I2C1_BASE				(APB1PERIPH_BASE + 0x05400UL)				/*I2C1 Base Address*/
#define I2C2_BASE				(APB1PERIPH_BASE + 0x05800UL)				/*I2C2 Base Address*/
#define I2C3_BASE				(APB1PERIPH_BASE + 0x05C00UL)				/*I2C3 Base Address*/
#define CAN1_BASE				(APB1PERIPH_BASE + 0x06400UL)				/*CAN1 Base Address*/
#define CAN2_BASE				(APB1PERIPH_BASE + 0x06800UL)				/*CAN2 Base Address*/
#define PWR_BASE				(APB1PERIPH_BASE + 0x07000UL)				/*PWR Base Address*/
#define DAC_BASE				(APB1PERIPH_BASE + 0x07400UL)				/*DAC1Base Address*/
#define UART7_BASE				(APB1PERIPH_BASE + 0x07800UL)				/*UART7 Base Address*/
#define UART8_BASE				(APB1PERIPH_BASE + 0x07C00UL)				/*UART8 Base Address*/


/*APB2 PERIPHERAL BASE ADDRESSES*/
#define TIM1_BASE				(APB2PERIPH_BASE)							/*TIM1 Base Address*/
#define TIM8_BASE				(APB2PERIPH_BASE + 0x00400UL)				/*TIM8 Base Address*/
#define USART1_BASE				(APB2PERIPH_BASE + 0x01000UL)				/*USART1 Base Address*/
#define USART6_BASE				(APB2PERIPH_BASE + 0x01400UL)				/*USART6 Base Address*/
#define ADC_1_2_3_BASE			(APB2PERIPH_BASE + 0x02000UL)				/*ADC1, ADC2, and ADC3 Base Address*/
#define SDIO_BASE				(APB2PERIPH_BASE + 0x02C00UL)				/*SDIO Base Address*/
#define SPI1_BASE				(APB2PERIPH_BASE + 0x03000UL)				/*SPI1 Base Address*/
#define SPI4_BASE				(APB2PERIPH_BASE + 0x03400UL)				/*SPI4 Base Address*/
#define SYSCFG_BASE				(APB2PERIPH_BASE + 0x03800UL)				/*SYSCFG Base Address*/
#define EXTI_BASE				(APB2PERIPH_BASE + 0x03C00UL)				/*EXTI Base Address*/
#define TIM9_BASE				(APB2PERIPH_BASE + 0x04000UL)				/*TIM9 Base Address*/
#define TIM10_BASE				(APB2PERIPH_BASE + 0x04400UL)				/*TIM10 Base Address*/
#define TIM11_BASE				(APB2PERIPH_BASE + 0x04800UL)				/*TIM11 Base Address*/
#define SPI5_BASE				(APB2PERIPH_BASE + 0x05000UL)				/*SPI5 Base Address*/
#define SPI6_BASE				(APB2PERIPH_BASE + 0x05400UL)				/*SPI6 Base Address*/
#define SAI1_BASE				(APB2PERIPH_BASE + 0x05800UL)				/*SAI1 Base Address*/
#define LCD_TFT_BASE			(APB2PERIPH_BASE + 0x06800UL)				/*LCD_TFT Base Address*/


/*AHB1 PERIPHERAL BASE ADDRESSES*/
#define GPIOA_BASE				(AHB1PERIPH_BASE)							/*GPIOA Base Address*/
#define GPIOB_BASE				(AHB1PERIPH_BASE + 0x00400UL)				/*GPIOB Base Address*/
#define GPIOC_BASE				(AHB1PERIPH_BASE + 0x00800UL)				/*GPIOC Base Address*/
#define GPIOD_BASE				(AHB1PERIPH_BASE + 0x00C00UL)				/*GPIOD Base Address*/
#define GPIOE_BASE				(AHB1PERIPH_BASE + 0x01000UL)				/*GPIOE Base Address*/
#define GPIOF_BASE				(AHB1PERIPH_BASE + 0x01400UL)				/*GPIOF Base Address*/
#define GPIOG_BASE				(AHB1PERIPH_BASE + 0x01800UL)				/*GPIOG Base Address*/
#define GPIOH_BASE				(AHB1PERIPH_BASE + 0x01C00UL)				/*GPIOH Base Address*/
#define GPIOI_BASE				(AHB1PERIPH_BASE + 0x02000UL)				/*GPIOI Base Address*/
#define GPIOJ_BASE				(AHB1PERIPH_BASE + 0x02400UL)				/*GPIOJ Base Address*/
#define GPIOK_BASE				(AHB1PERIPH_BASE + 0x02800UL)				/*GPIOK Base Address*/
#define CRC_BASE				(AHB1PERIPH_BASE + 0x03000UL)				/*CRC Base Address*/
#define RCC_BASE				(AHB1PERIPH_BASE + 0x03800UL)				/*RCC Base Address*/
#define FLASH_INTERFACE_BASE	(AHB1PERIPH_BASE + 0x03C00UL)				/*FLASH_INTERFACE Base Address*/
#define BKP_SRAM_BASE			(AHB1PERIPH_BASE + 0x04000UL)				/*BKPSRAM Base Address*/
#define DMA1_BASE				(AHB1PERIPH_BASE + 0x06000UL)				/*DMA1 Base Address*/
#define DMA2_BASE				(AHB1PERIPH_BASE + 0x06400UL)				/*DMA2 Base Address*/
#define ETHERNET_MAC_BASE		(AHB1PERIPH_BASE + 0x08000UL)				/*ETHERNET_MAC Base Address*/
#define DMA2D_BASE				(AHB1PERIPH_BASE + 0x0B000UL)				/*DMAMUX1 Base Address*/
#define USB_OTG_HS_BASE			(AHB1PERIPH_BASE + 0x20000UL)				/*USB1_OTG_HS Base Address*/



/*AHB2 PERIPHERAL BASE ADDRESSES*/
#define USB_OTG_FS_BASE			(AHB2PERIPH_BASE)							/*USB2_OTG_HS Base Address*/
#define DCMI_BASE				(AHB2PERIPH_BASE + 0x50000UL)				/*DCMI Base Address*/
#define CRYPTO_BASE				(AHB2PERIPH_BASE + 0x60000UL)				/*CRYPTO Base Address*/
#define HASH_BASE				(AHB2PERIPH_BASE + 0x60400UL)				/*HASH Base Address*/
#define RNG_BASE				(AHB2PERIPH_BASE + 0x60800UL)				/*RNG Base Address*/


/*AHB3 PERIPHERAL BASE ADDRESSES*/
#define FSMC_CONTROL_BASE		(AHB3PERIPH_BASE)							/*FSMC_CONTROL Base Address*/


/******************************************************************************/
/********************* Register Definition Structures *************************/

typedef struct{																/*Off. Description*/
	_IO uint32_t MODER;														/*0x00 GPIO port mode register*/
	_IO uint32_t OTYPER;													/*0x04 GPIO port output type register*/
	_IO uint32_t OSPEEDR;													/*0x08 GPIO port output speed register*/
	_IO uint32_t PUPDR;														/*0x0C GPIO port pull-up/pull-down register*/
	_IO uint32_t IDR;														/*0x10 GPIO port input data register*/
	_IO uint32_t ODR;														/*0x14 GPIO port output data register*/
	_IO uint32_t BSRR;														/*0x18 GPIO port bit set/reset register*/
	_IO uint32_t LCKR;														/*0x1C GPIO port configuration lock register*/
	_IO uint32_t AFR[1];													/*0x20 GPIO alternate function low register*/
																			/*0x24 GPIO alternate function high register*/
}GPIO_RegDef_t;

typedef struct{																/*Off.  Description*/
	_IO uint32_t RCC_CR;													/*0x000 RCC source control register*/
	_IO uint32_t RCC_PLLCFGR;												/*0x004 RCC PLLs configuration register*/
	_IO uint32_t RCC_CFGR;													/*0x008 RCC clock configuration register*/
	_IO uint32_t RCC_CIR;													/*0x00C RCC clock interrupt register*/
	_IO uint32_t RCC_AHB1RSTR;												/*0x010 RCC AHB1 peripheral reset register*/
	_IO uint32_t RCC_AHB2RSTR;												/*0x014 RCC AHB2 peripheral reset register*/
	_IO uint32_t RCC_AHB3RSTR;												/*0x018 RCC AHB3 reset register*/
	_IO uint32_t RESERVED1;													/*0x01C RESERVED*/
	_IO uint32_t RCC_APB1RSTR;												/*0x020 RCC APB1 peripheral reset register*/
	_IO uint32_t RCC_APB2RSTR;												/*0x024 RCC APB2 peripheral reset register*/
	_IO uint32_t RESERVED2[2];												/*0x028...
																			  0x02C RESERVED*/
	_IO uint32_t RCC_AHB1ENR;												/*0x030 RCC AHB1 clock register*/
	_IO uint32_t RCC_AHB2ENR;												/*0x034 RCC AHB2 clock register*/
	_IO uint32_t RCC_AHB3ENR;												/*0x038 RCC AHB3 clock register*/
	_IO uint32_t RESERVED3;													/*0x03C RESERVED*/
	_IO uint32_t RCC_APB1ENR;												/*0x040 RCC APB1 clock register*/
	_IO uint32_t RCC_APB2ENR;												/*0x044 RCC APB2 clock register*/
	_IO uint32_t RESERVED4[2];												/*0x048...
																			  0x04C RESERVED*/
	_IO uint32_t RCC_AHB1LPENR;												/*0x050 RCC AHB1 Sleep clock register*/
	_IO uint32_t RCC_AHB2LPENR;												/*0x054 RCC AHB2 Sleep clock register*/
	_IO uint32_t RCC_AHB3LPENR;												/*0x058 RCC AHB3 Sleep clock register*/
	_IO uint32_t RESERVED5;													/*0x05C RESERVED*/
	_IO uint32_t RCC_APB1LPENR;												/*0x060 RCC APB1 Sleep clock register*/
	_IO uint32_t RCC_APB2LPENR;												/*0x064 RCC APB2 Sleep clock register*/
	_IO uint32_t RESERVED6[2];												/*0x068...
																			  0x06C RESERVED*/
	_IO uint32_t RCC_BDCR;													/*0x070 RCC Backup domain control register*/
	_IO uint32_t RCC_CSR;													/*0x074 RCC clock control and status register*/
	_IO uint32_t RESERVED7[2];												/*0x078...
																			  0x07C RESERVED*/
	_IO uint32_t RCC_SSCGR;													/*0x080 RCC spread spectrum clock generation register*/
	_IO uint32_t RCC_PLLI2CFGR;												/*0x084 RCC PLLI2S configuration register*/


}RCC_RegDef_t;

typedef struct{																/*Off. Description*/
	_IO uint32_t EXTI_IMR;													/*0x00 Interrupt mask register*/
	_IO uint32_t EXTI_EMR;													/*0x04 Event mask register*/
	_IO uint32_t EXTI_RTSR;													/*0x08 Rising trigger selection register*/
	_IO uint32_t EXTI_FTSR;													/*0x0C Falling trigger selection register*/
	_IO uint32_t EXTI_SWIER;												/*0x10 Software interrupt event register*/
	_IO uint32_t EXTI_PR;													/*0x14 Pending register*/
}EXTI_RegDef_t;

typedef struct{																/*Off. Description*/
	_IO uint32_t SYSCFG_MEMRMP;												/*0x00 SYSCFG memory remap register*/
	_IO uint32_t SYSCFG_PMC;												/*0x04 SYSCFG peripheral mode configuration register*/
	_IO uint32_t SYSCFG_EXTICR[4];											/*0x08...
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	  0x14 SYSCFG external interrupt configuration register 1-4*/
	_IO uint32_t RESERVED[2];												/*0x18...
																			  0x1C RESERVED*/
	_IO uint32_t SYSCFG_CMPCR;												/*0x20 Compensation cell control register*/
}SYSCFG_RegDef_t;

typedef struct{																/*Off. Description*/
	_IO uint32_t SPI_CR1;													/*0x00 SPI control register 1*/
	_IO uint32_t SPI_CR2;													/*0x04 SPI control register 2*/
	_IO uint32_t SPI_SR;													/*0x08 SPI status register*/
	_IO uint32_t SPI_DR;													/*0x0C SPI data register*/
	_IO uint32_t SPI_CRCPR;													/*0x10 SPI CRC polynomial register*/
	_IO uint32_t SPI_RXCRCR;												/*0x14 SPI RX CRC register*/
	_IO uint32_t SPI_TXCRCR;												/*0x18 SPI TX CRC register*/
	_IO uint32_t SPI_I2SCFGR;												/*0x1C SPI SPI_I2S configuration register*/
	_IO uint32_t SPI_I2SPR;													/*0x20 SPI_I2S prescaler register*/
}SPI_RegDef_t;

/******************************************************************************/
/************************* Peripheral Definitions *****************************/

#define GPIOA 					((GPIO_RegDef_t*)GPIOA_BASE)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASE)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASE)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASE)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASE)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASE)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASE)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASE)
#define GPIOI					((GPIO_RegDef_t*)GPIOI_BASE)
#define GPIOJ					((GPIO_RegDef_t*)GPIOJ_BASE)
#define GPIOK					((GPIO_RegDef_t*)GPIOK_BASE)

#define RCC						((RCC_RegDef_t*)RCC_BASE)

#define EXTI					((EXTI_RegDef_t*)EXTI_BASE)

#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASE)

#define SPI1					((SPI_RegDef_t*)SPI1_BASE)
#define SPI2					((SPI_RegDef_t*)SPI2_I2S2_BASE)
#define SPI3					((SPI_RegDef_t*)SPI3_I2S3_BASE)
#define SPI4					((SPI_RegDef_t*)SPI4_BASE)
#define SPI5					((SPI_RegDef_t*)SPI5_BASE)
#define SPI6					((SPI_RegDef_t*)SPI6_BASE)

#define GPIO_BASE_TO_CODE(x)	((x == GPIOA) ? 0 :\
								 (x == GPIOB) ? 1 :\
								 (x == GPIOC) ? 2 :\
								 (x == GPIOD) ? 3 :\
								 (x == GPIOE) ? 4 :\
								 (x == GPIOF) ? 5 :\
								 (x == GPIOG) ? 6 :\
								 (x == GPIOH) ? 7 :\
								 (x == GPIOI) ? 8 :\
								 (x == GPIOJ) ? 9 :\
								 (x == GPIOK) ? 10:0)

/*Interrupt Request Number*/
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

/******************************************************************************/
/***************** SPI Peripheral Registers Bit Positions *********************/

/*SPI CR1*/
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

/*SPI CR2*/
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

/*SPI SR*/
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_URD				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8


/******************************************************************************/
/******************* Peripheral Clock Enables/Disables ************************/

/*SYSCFG PERIPHERAL CLOCK ENABLE*/
#define SYSCFG_PCLK_EN()      	(RCC->RCC_APB2ENR |= (1 << 14))

/*SYSCFG PERIPHERAL CLOCK DISABLE*/
#define SYSCFG_PCLK_DI()      	(RCC->RCC_APB2ENR &= ~(1 << 14))

/*GPIO CLOCK ENABLES*/
#define GPIOA_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 10))

/*GPIO CLOCK DISABLES*/
#define GPIOA_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 10))

/*SPI CLOCK ENABLES*/
#define SPI1_PCLK_EN()			(RCC->RCC_APB2ENR  |= (1 << 12))
#define SPI2_PCLK_EN()			(RCC->RCC_APB1ENR  |= (1 << 14))
#define SPI3_PCLK_EN()			(RCC->RCC_APB1ENR  |= (1 << 15))
#define SPI4_PCLK_EN()			(RCC->RCC_APB2ENR  |= (1 << 13))
#define SPI5_PCLK_EN()			(RCC->RCC_APB2ENR  |= (1 << 20))
#define SPI6_PCLK_EN()			(RCC->RCC_APB2ENR  |= (1 << 21))

/*SPI CLOCK DISABLES*/
#define SPI1_PCLK_DI()			(RCC->RCC_APB2ENR  &= ~(1 << 12))
#define SPI2_PCLK_DI()			(RCC->RCC_APB1ENR  &= ~(1 << 14))
#define SPI3_PCLK_DI()			(RCC->RCC_APB1ENR  &= ~(1 << 15))
#define SPI4_PCLK_DI()			(RCC->RCC_APB2ENR  &= ~(1 << 13))
#define SPI5_PCLK_DI()			(RCC->RCC_APB2ENR  &= ~(1 << 20))
#define SPI6_PCLK_DI()			(RCC->RCC_APB2ENR  &= ~(1 << 21))

/*UART CLOCK ENABLES*/
#define USART1_PCLK_EN()        (RCC->RCC_APB2ENR  |= (1 << 4))
#define USART2_PCLK_EN()		(RCC->RCC_APB1ENR  |= (1 << 17))
#define USART3_PCLK_EN()		(RCC->RCC_APB1ENR  |= (1 << 18))
#define UART4_PCLK_EN()			(RCC->RCC_APB1ENR  |= (1 << 19))
#define UART5_PCLK_EN()			(RCC->RCC_APB1ENR  |= (1 << 20))
#define USART6_PCLK_EN()		(RCC->RCC_APB2ENR  |= (1 << 5))
#define UART7_PCLK_EN()			(RCC->RCC_APB1ENR  |= (1 << 30))
#define UART8_PCLK_EN()			(RCC->RCC_APB1ENR  |= (1 << 31))

/*UART CLOCK ENABLES*/
#define USART1_PCLK_DI()        (RCC->RCC_APB2ENR  &= ~(1 << 4))
#define USART2_PCLK_DI()		(RCC->RCC_APB1ENR  &= ~(1 << 17))
#define USART3_PCLK_DI()		(RCC->RCC_APB1ENR  &= ~(1 << 18))
#define UART4_PCLK_DI()			(RCC->RCC_APB1ENR  &= ~(1 << 19))
#define UART5_PCLK_DI()			(RCC->RCC_APB1ENR  &= ~(1 << 20))
#define USART6_PCLK_DI()		(RCC->RCC_APB2ENR  &= ~(1 << 5))
#define UART7_PCLK_DI()			(RCC->RCC_APB1ENR  &= ~(1 << 30))
#define UART8_PCLK_DI()			(RCC->RCC_APB1ENR  &= ~(1 << 31))

/*I2C CLOCK ENABLES*/
#define I2C1_PCLK_EN()			(RCC->RCC_APB1ENR  |= (1 << 21))
#define I2C2_PCLK_EN()			(RCC->RCC_APB1ENR  |= (1 << 22))
#define I2C3_PCLK_EN()			(RCC->RCC_APB1ENR  |= (1 << 23))

/*I2C CLOCK ENABLES*/
#define I2C1_PCLK_DI()			(RCC->RCC_APB1ENR  &= ~(1 << 21))
#define I2C2_PCLK_DI()			(RCC->RCC_APB1ENR  &= ~(1 << 22))
#define I2C3_PCLK_DI()			(RCC->RCC_APB1ENR  &= ~(1 << 23))


#endif /* INC_STM32F407XX_H_ */
