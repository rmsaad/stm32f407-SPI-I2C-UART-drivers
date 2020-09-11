/*
 * SPI_test_01.c
 *
 *      Author: Rami
 *
 *
 *      PB14	SPI2_MISO
 *      PB15	SPI2_MOSI
 *      PB13	SPI2_SCLK
 *      PB12	SPI2_NSS
 *      ALT func mode 5
 */


#include "stm32f407xx.h"
#include "stdint.h"
#include "string.h"

/*prototypes*/
void SPI2_GPIOInit();
void SPI2_Init();

int main(void){

	char user_data[] ="hello world";

	// configure GPIO pins to behave as SPI pins
	SPI2_GPIOInit();

	SPI2_Init();

	SPI_SSIConfig(SPI2, EN);

	// SPE bit changed only after initialization
	SPI_PeripheralControl(SPI2, EN);

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	SPI_PeripheralControl(SPI2, DI);

	while(1);

	return 0;
}

void SPI2_GPIOInit(){
	GPIO_Handle_t SPIpins;

	SPIpins.pGPIOx = GPIOB;
	SPIpins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIpins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIpins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIpins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIpins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK first
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIpins);

	// MOSI
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIpins);

	// MISO
	//SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	//GPIO_Init(&SPIpins);

	//NSS
	//SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	//GPIO_Init(&SPIpins);
}

void SPI2_Init(){
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULL_DUPLEX;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2handle);

}
