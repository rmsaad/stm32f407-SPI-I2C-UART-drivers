/*
 * stn32f407xx_spi_driver.c
 *
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
	uint32_t tempreg = 0;														/*temporary register value*/

	SPI_PCLKControl(pSPIHandle->pSPIx, EN);										/*enable SPI PCLK*/

	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;			/*device mode*/

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUPLEX){		/*bus configuration...*/
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);									/*bidirectional cleared*/

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX){
		tempreg |= (1 << SPI_CR1_BIDIMODE);										/*bidirectional set*/

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX){
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);									/*bidirectional cleared*/
		tempreg |= (1 << SPI_CR1_RXONLY);										/*RXONLY set*/
	}
	tempreg |= pSPIHandle->SPIConfig.SPI_SCLKSpeed << SPI_CR1_BR;				/*speed configuration*/
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;					/*Data Frame Format*/
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;					/*Clock Polarity*/
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;					/*Clock Phase*/
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;					/*SSM*/
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
 * @param[in]	: SPI peripheral address
 * @param[in]	: pointer to the first byte of the TxBuffer
 * @param[in]	: length of the TxBuffer data
 *
 * @note		  This is a blocking call (polling based)
 * @return		: none
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

	while(Len > 0){
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);			/*wait until TXE is set*/
																				/*check DFF bit*/
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){								/*16 bit*/
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);							/*load data into Data register, type-cast to 16bit unsigned int pointer*/
			Len--; Len--;
			(uint16_t*)pTxBuffer++;
		}else{																	/*8 bit*/
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
 * @param[in]	: SPI peripheral address
 * @param[in]	: pointer to the first byte of the RxBuffer
 * @param[in]	: length of the RxBuffer data
 *
 * @return		: none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){
	while(Len > 0){
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);			/*wait until RXNE is set*/
																				/*check DFF bit*/
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){								/*16 bit*/
			 *((uint16_t*)pRxBuffer) = pSPIx->SPI_DR;							/*load data From Data register to RxBuffer, type-cast to 16bit unsigned integer pointer*/
			Len--; Len--;
			(uint16_t*)pRxBuffer++;
		}else{																	/*8 bit*/
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
		pSPIHandle->pTxBuffer = pTxBuffer;										/*Save TxBuffer address and length information in global variables*/
		pSPIHandle->TxLen = Len;

		pSPIHandle->TxState = SPI_BUSY_IN_TX;									/*Mark the SPI state as busy in transmission so that no other code
																			      can take over same SPI peripheral until transmission over*/
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);						/*Enable TXEIE control bit to get interrupt when TXE flag is set in SR*/
	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX){
		pSPIHandle->pRxBuffer = pRxBuffer;										/*Save RxBuffer address and length information in global variables*/
		pSPIHandle->RxLen = Len;

		pSPIHandle->RxState = SPI_BUSY_IN_RX;									/*Mark the SPI state as busy in transmission so that no other code
																				  can take over same SPI peripheral until transmission over*/
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);					/*Enable TXEIE control bit to get interrupt when RXNE flag is set in SR*/
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

	uint8_t temp1, temp2;														/*temporary register variables*/

	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_TXE);						/*check for TXE*/
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2){
		spi_txe_interrupt_handle(pSPIHandle);									/*handle TXE*/
	}

	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_RXNE);						/*check for RXNE*/
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2){
		spi_rxne_interrupt_handle(pSPIHandle);									/*handle RXNE*/
	}

	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_OVR);						/*check for Overrun error*/
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2){
		spi_ovr_err_interrupt_handle(pSPIHandle);								/*handle OVR error*/
	}
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
																				/*check DFF bit*/
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){						/*16 bit*/
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer);		/*load data into Data register, type-cast to 16bit unsigned int pointer*/
		pSPIHandle->TxLen--; pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else{																		/*8 bit*/
		pSPIHandle->pSPIx->SPI_DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen --;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen){														/*close SPI transmission and inform application tx is over*/
		SPI_Close_Transmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
																				/*check DFF bit*/
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){						/*16 bit*/
		 *((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR;		/*load data From Data register to RxBuffer, type-cast to 16bit unsigned int pointer*/
		 pSPIHandle->RxLen--; pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}else{																		/*8 bit*/
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR;					/*load data From Data register to RxBuffer*/
		pSPIHandle->RxLen --;
		pSPIHandle->pRxBuffer++;
	}

	if(!pSPIHandle->RxLen){														/*close SPI transmission and inform application RX is over*/
		SPI_Close_Reception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){

	uint8_t temp;																/*temporary register variable*/
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){									/*clear OVR flag*/
		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_SR;
	}
	(void)temp;																	/*suppress warning*/
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);				/*inform application*/
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;
	(void)temp;
}

void SPI_Close_Transmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);						/*prevent interrupts from TXE flag*/
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_Close_Reception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);						/*prevent interrupts from RXNE flag*/
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
