/*
 * stn32f407xx_spi_driver.c
 *
 *  Created on: Sep 8, 2020
 *      Author: Rami
 */

#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"

/*private function prototypes*/
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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

	// enable SPI PCLK
	SPI_PCLKControl(pSPIHandle->pSPIx, EN);

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

	// SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

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
			// load data into Data register, type-cast to 16bit unsigned int pointer
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
	while(Len > 0){

		// wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// check DFF bit
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){
			//16 bit
			// load data From Data register to Rxbuffer, type-cast to 16bit unsigned int pointer
			 *((uint16_t*)pRxBuffer) = pSPIx->SPI_DR;
			Len--; Len--;
			(uint16_t*)pRxBuffer++;
		}else{
			// 8 bit
			*(pRxBuffer) = pSPIx->SPI_DR;
			Len --;
			pRxBuffer++;
		}
	}
}

/******************************************************************************/
/******************* IRQ Configuration and ISR Handling ***********************/

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX){
		// Save Tx buffer address and Len information in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//Enable TXEIE control bit to get interrupt when TXE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX){
		// Save Rx buffer address and Len information in global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//Enable TXEIE control bit to get interrupt when RXNE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
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

	uint8_t temp1, temp2;

	// check for TXE
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2){

		// handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	// check for RXNE
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2){

		// handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	// check for Overrun error
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2){

		// handle OVR error
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}


}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	// check DFF bit
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){
		//16 bit
		// load data into Data register, type-cast to 16bit unsigned int pointer
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--; pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else{
		// 8 bit
		pSPIHandle->pSPIx->SPI_DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen --;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen){
		// close spi transmission and inform application tx is over
		SPI_Close_Transmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	// check DFF bit
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){
		//16 bit
		// load data From Data register to Rxbuffer, type-cast to 16bit unsigned int pointer
		 *((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR;
		 pSPIHandle->RxLen--; pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}else{
		// 8 bit
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->RxLen --;
		pSPIHandle->pRxBuffer++;
	}

	if(!pSPIHandle->RxLen){
		// close spi transmission and inform application rx is over
		SPI_Close_Reception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){

	uint8_t temp;

	// clear ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_SR;
	}
	(void)temp;
	// inform application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;
	(void)temp;
}

void SPI_Close_Transmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);	// prevent interrupts from txe flag
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_Close_Reception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);	// prevent interrupts from rxne flag
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

__attribute__ ((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){
	// weak implementation, application will override this function
}
/******************************************************************************/
/*********************** Other Peripheral Control API *************************/

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if(pSPIx->SPI_SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t en_di){
	if(en_di){
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
	}else{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t en_di){
	if(en_di){
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
	}else{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
