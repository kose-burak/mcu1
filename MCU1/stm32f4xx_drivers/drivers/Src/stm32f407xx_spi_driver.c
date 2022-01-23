/*
 * stm32f407xx_spi_driver.c
 */

#include "stm32f407xx_spi_driver.h"

/*
 *  Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE){
			if (pSPIx == SPI1)
				SPI1_PCLK_EN();
			else if (pSPIx == SPI2)
				SPI2_PCLK_EN();
			else if (pSPIx == SPI3)
				SPI3_PCLK_EN();
			else if (pSPIx == SPI4)
				SPI4_PCLK_EN();
		}
		else {
			if (pSPIx == SPI1)
				SPI1_PCLK_DI();
			else if (pSPIx == SPI2)
				SPI2_PCLK_DI();
			else if (pSPIx == SPI3)
				SPI3_PCLK_DI();
			else if (pSPIx == SPI4)
				SPI4_PCLK_DI();
		}
}

/*
 *  Initialize and De-initialize
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempreg = 0;

	// enable SPI clock peripheral
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// configure the SPI mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// configure the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// BIDI mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// BIDI mode should be cleared and RXONLY set
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// configure the SPI serial clock speed ( baud rate )
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;

}
void SPI_DeInit(SPI_RegDef_t *pSPIx);


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SR & FlagName)
		return FLAG_SET;

	return FLAG_RESET;
}

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		// wait until TXE is set
		while ( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

		// check DFF
		if (	(pSPIx->CR1 & ( 1 << SPI_CR1_DFF))	)
		{
			// 16 bit DFF	// load the data in to the DR
			pSPIx->DR |= *((uint16_t *)pTxBuffer );
			Len--;
			Len--;
			(uint16_t *)pTxBuffer++;
		}
		else
		{
			// 8 bit DFF
			pSPIx->DR |= *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		// wait until RXNE is set
		while ( SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET );

		// check DFF
		if (	(pSPIx->CR1 & ( 1 << SPI_CR1_DFF))	)
		{
			// 16 bit DFF	// load the data from DR to RxBuffer address
			*((uint16_t *)pRxBuffer ) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t *)pRxBuffer++;
		}
		else
		{
			// 8 bit DFF
			*(pRxBuffer ) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= ( 1 << SPI_CR1_SPE);
	}
	else
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE);
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= ( 1 << SPI_CR1_SSI);
	}
	else
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SSI);
}
/*
 *  IRQ configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE){
		if (IRQNumber <= 31){
			*NVIC_ISER0 |= ( 1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64){
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64));
		}
	}
	else {
		if (IRQNumber <= 31){
			*NVIC_ICER0 |= ( 1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64){
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96){
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64));
		}
	}
}
void SPI_IRQPRiorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section) +( 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR +  iprx) |= ( IRQPriority << shift_amount );

}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if ( state != SPI_BUSY_IN_TX) {
		// Save the TxBuffer address and Len information in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// Mark the SPI state as busy in transmission 		// until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE);
	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if ( state != SPI_BUSY_IN_RX) {
		// Save the TxBuffer address and Len information in global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// Mark the SPI state as busy in transmission 		// until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE);
	}
	return state;

}

/*
 *  SPI interrupts handling and helper functions
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
		// check DFF
		if (	(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF))	)
		{
			// 16 bit DFF	// load the data in to the DR
			pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer );
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;
			(uint16_t *)pSPIHandle->pTxBuffer++;
		}
		else
		{
			// 8 bit DFF
			pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++;
		}
		if ( !pSPIHandle->TxLen ) {
			// TxLen is zero, close the SPI transmission and inform the application that Tx is over

			// prevents interrupts from TXE flag
			SPI_CloseTransmission(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
		}

}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check DFF
	if (	(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF))	)
	{
		// 16 bit DFF	// load the data in to the DR
		*((uint16_t *)pSPIHandle->pRxBuffer ) = (uint8_t )pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;
		pSPIHandle->pRxBuffer--;
	}
	else
	{
		// 8 bit DFF
		*( pSPIHandle->pRxBuffer ) = ( uint8_t )pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;
	}
	if ( !pSPIHandle->RxLen ) {
		// TxLen is zero, close the SPI transmission and inform the application that Tx is over
		// prevents interrupts from TXE flag
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}


}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	// clear the OVR flag
	if ( pSPIHandle->TxState != SPI_BUSY_IN_TX) {
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	// inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE );
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE );
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;

	//check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE );
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE );

	if ( temp1 && temp2 ) {
		// handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE );
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE );

	if ( temp1 && temp2 ) {
		// handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	//check for OVR
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR );
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE );

	if ( temp1 && temp2 ) {
		// handle RXNE
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	// this is a weak implementation. The application may override this function
}



