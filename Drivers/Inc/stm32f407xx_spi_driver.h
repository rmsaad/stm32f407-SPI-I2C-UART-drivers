/*
 * stm32f407xx_spi_driver.h
 *
 *      Author: Rami
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include <stdint.h>
#include "stm32f407xx.h"

/******************************************************************************/
/****************************** SPI Macros ************************************/

/*Device Modes*/
#define SPI_DEVICE_MODE_SLAVE		0
#define SPI_DEVICE_MODE_MASTER		1

/*Bus Configurations*/
#define SPI_BUS_CONFIG_FULL_DUPLEX	1
#define SPI_BUS_CONFIG_HALF_DUPLEX	2
#define SPI_BUS_CONFIG_SIMPLEX_RX	3

/*Serial Clock Speed*/
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*Data Frame Format*/
#define SPI_DFF_8BITS				0
#define SPI_DFF_16BITS				1

/*Clock Polarity*/
#define SPI_CPOL_LOW				0
#define SPI_CPOL_HIGH				1

/*Clock Phase*/
#define SPI_CPHA_LOW				0
#define SPI_CPHA_HIGH				1

/*Software Slave Management*/
#define SPI_SMM_DI					0
#define SPI_SSM_EN					1

/*Reset SPIx peripherals*/
#define	SPI1_REG_RESET() 	do {(RCC->RCC_APB2RSTR |= (1 << 12));  (RCC->RCC_APB2RSTR &= ~(1 << 12));}  while(0)
#define	SPI2_REG_RESET() 	do {(RCC->RCC_APB1RSTR |= (1 << 14));  (RCC->RCC_APB1RSTR &= ~(1 << 14));}  while(0)
#define	SPI3_REG_RESET() 	do {(RCC->RCC_APB1RSTR |= (1 << 15));  (RCC->RCC_APB1RSTR &= ~(1 << 15));}  while(0)
#define	SPI4_REG_RESET() 	do {(RCC->RCC_APB2RSTR |= (1 << 13));  (RCC->RCC_APB2RSTR &= ~(1 << 13));}  while(0)
#define	SPI5_REG_RESET() 	do {(RCC->RCC_APB2RSTR |= (1 << 20));  (RCC->RCC_APB2RSTR &= ~(1 << 20));}  while(0)
#define	SPI6_REG_RESET() 	do {(RCC->RCC_APB2RSTR |= (1 << 21));  (RCC->RCC_APB2RSTR &= ~(1 << 21));}  while(0)

/*SPI status Flag definitions*/
#define SPI_TXE_FLAG				(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG				(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG				(1 << SPI_SR_BSY)

/*SPI Application States*/
#define SPI_READY					0
#define SPI_BUSY_IN_RX				1
#define SPI_BUSY_IN_TX				2

/*Application Event*/
#define SPI_EVENT_TX_CMPLT			1
#define SPI_EVENT_RX_CMPLT			2
#define SPI_EVENT_OVR_ERR			3
#define SPI_EVENT_CRC_ERR			4

/******************************************************************************/
/***************** SPI Configuration & Handle Structures **********************/

/*SPI Configuration Structure*/
typedef struct{
	uint8_t	SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SCLKSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*SPI Handle Structure*/
typedef struct{
	SPI_RegDef_t 	*pSPIx;		// base address of SPIx peripheral
	SPI_Config_t 	SPIConfig;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint8_t			TxState;
	uint8_t			RxState;
}SPI_Handle_t;

/******************************************************************************/
/*************************** Function ProtoTypes ******************************/

/*Peripheral Clock setup*/
void SPI_PCLKControl(SPI_RegDef_t *pSPIx, uint8_t en_di);

/*Initiation De-initiation*/
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*Data Send/Receive*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*IRQ Configuration and ISR Handling*/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t en_di);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_Close_Transmission(SPI_Handle_t *pSPIHandle);
void SPI_Close_Reception(SPI_Handle_t *pSPIHandle);

/*Application Callback*/
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

/*Other Peripheral Control API*/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t en_di);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t en_di);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
