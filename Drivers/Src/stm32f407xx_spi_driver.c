/*
 * stn32f407xx_spi_driver.c
 *
 *  Created on: Sep 8, 2020
 *      Author: Rami
 */

#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"

/******************************************************************************/
/************************** Peripheral Clock Setup ****************************/

/**
 * @fn			: SPI_PCLKControl
 *
 * @brief		: enables or disables the peripheral clock for a given SPI port
 *
 * @param[in]	: base address of the SPIx peripheral
 * @param[in]	: enable(1) or disable(0)
 *
 * @return		: none
 */
void SPI_PCLKControl(SPI_RegDef_t *pSPIx, uint8_t en_di){
	if(en_di == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI4){
			SPI4_PCLK_EN();
		}else if(pSPIx == SPI5){
			SPI5_PCLK_EN();
		}else if(pSPIx == SPI6){
			SPI6_PCLK_EN();
		}

	}else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}else if(pSPIx == SPI4){
			SPI4_PCLK_DI();
		}else if(pSPIx == SPI5){
			SPI5_PCLK_DI();
		}else if(pSPIx == SPI6){
			SPI6_PCLK_DI();
		}
	}
}

/******************************************************************************/
/********************** GPIO Initiation De-initiation *************************/

/**
 * @fn			: SPI_Init
 *
 * @brief		: initializes the SPIx peripheral
 *
 * @param[in]	: pointer to the user create handle structure
 *
 * @return		: none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){
	// configure SPI_CR1
	uint32_t tempreg = 0;

	// device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// bus configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUPLEX){
		// bidirectional cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX){
		// bidirectional set
		tempreg |= (1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX){
		// bidirectional cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

		// RXONLY set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// speed configuration
	tempreg |= pSPIHandle->SPIConfig.SPI_SCLKSpeed << SPI_CR1_BR;

	// Data Frame Format
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// Clock Polarity
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// Clock Phase
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->SPI_CR1 = tempreg;
}

/**
 * @fn			: SPI_DeInit
 *
 * @brief		: deinitialize register of SPIx peripheral
 *
 * @param[in]	: base address of the SPIx peripheral
 *
 * @return		: none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}else if(pSPIx == SPI4){
		SPI4_REG_RESET();
	}else if(pSPIx == SPI5){
		SPI5_REG_RESET();
	}else if(pSPIx == SPI6){
		SPI6_REG_RESET();
	}
}

/******************************************************************************/
/**************************** Data Send/Receive *******************************/

/**
 * @fn			: SPI_SendData
 *
 * @brief		:
 *
 * @param[in]	:
 * @param[in]	:
 * @param[in]	:
 *				  This is a blocking call (polling based)
 * @return		: none
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

	while(Len > 0){

		// wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// check DFF bit
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){
			//16 bit
			// load data into Data register, typecast to 16
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);
			Len--; Len--;
			(uint16_t*)pTxBuffer++;
		}else{
			// 8 bit
			pSPIx->SPI_DR = *(pTxBuffer);
			Len --;
			pTxBuffer++;
		}
	}
}

/**
 * @fn			: SPI_ReceiveData
 *
 * @brief		:
 *
 * @param[in]	:
 * @param[in]	:
 * @param[in]	:
 *
 * @return		: none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){

}

/******************************************************************************/
/******************* IRQ Configuration and ISR Handleing **********************/

/**
 * @fn			:
 *
 * @brief		:
 *
 * @param[in]	:
 * @param[in]	:
 *
 * @return		: none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t en_di){

}

/**
 * @fn			:
 *
 * @brief		:
 *
 * @param[in]	:
 * @param[in]	:
 *
 * @return		: none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

}

/**
 * @fn			:
 *
 * @brief		:
 *
 * @param[in]	:
 *
 * @return		: none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){

}

/******************************************************************************/
/*********************** Other Peripheral Control API *************************/

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if(pSPIx->SPI_SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
