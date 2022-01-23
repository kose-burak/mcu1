/*
 * stm32f407xx_usart_driver.c
 */

#include "stm32f407xx_usart_driver.h"

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t Cmd)
{
	if ( Cmd == ENABLE ) {
		pUSARTx->CR1 |= ( 1 << 13 );
	}
	else {
		pUSARTx->CR1 &= ~( 1 << 13);
	}
}

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if ( EnorDi == ENABLE ) {
		if ( pUSARTx == USART1) {
			USART1_PCLK_EN();
		}
		else if ( pUSARTx == USART2) {
			USART2_PCLK_EN();
		}
		else if ( pUSARTx == USART3) {
			USART3_PCLK_EN();
		}
		else if ( pUSARTx == USART4) {
			USART4_PCLK_EN();
		}
	}
	else {
		if (pUSARTx == USART1)
			USART1_PCLK_DI();
		else if (pUSARTx == USART2)
			USART2_PCLK_DI();
		else if (pUSARTx == USART3)
			USART3_PCLK_DI();
	}
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t StatusFlagName)
{
	if (pUSARTx->SR1 & StatusFlagName)
		return FLAG_SET;

	return FLAG_RESET;
}
