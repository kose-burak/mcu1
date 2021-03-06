/*
 * stm32f407xx_gpio_driver.c
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * peripheral clock control
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE){
		if (pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if (pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if (pGPIOx == GPIOC)
					GPIOC_PCLK_EN();
		else if (pGPIOx == GPIOD)
					GPIOD_PCLK_EN();
		else if (pGPIOx == GPIOE)
					GPIOE_PCLK_EN();
		else if (pGPIOx == GPIOF)
					GPIOF_PCLK_EN();
		else if (pGPIOx == GPIOG)
					GPIOG_PCLK_EN();
		else if (pGPIOx == GPIOH)
					GPIOH_PCLK_EN();
		else if (pGPIOx == GPIOI)
					GPIOI_PCLK_EN();
	}
	else {
		if (pGPIOx == GPIOA)
					GPIOA_PCLK_DI();
				else if (pGPIOx == GPIOB)
					GPIOB_PCLK_DI();
				else if (pGPIOx == GPIOC)
							GPIOC_PCLK_DI();
				else if (pGPIOx == GPIOD)
							GPIOD_PCLK_DI();
				else if (pGPIOx == GPIOE)
							GPIOE_PCLK_DI();
				else if (pGPIOx == GPIOF)
							GPIOF_PCLK_DI();
				else if (pGPIOx == GPIOG)
							GPIOG_PCLK_DI();
				else if (pGPIOx == GPIOH)
							GPIOH_PCLK_DI();
				else if (pGPIOx == GPIOI)
							GPIOI_PCLK_DI();
	}
}

/*
 * Initialize and deinitialize
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	// enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// configure the mode of pin
	if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		//non interrupt mode
		temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber );
		pGPIOHandle -> pGPIOx ->MODER &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); //clear
		pGPIOHandle -> pGPIOx ->MODER |= temp; //set
	}
	else {
		//interrupt mode
		if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			//clear the RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			// configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			//clear the FTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			// configure both FTSR and RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= ( 1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}
		// configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (4 * temp2);

		//enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;

	// configure the speed
	temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx ->OSPEEDR &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); //clear
	pGPIOHandle -> pGPIOx ->OSPEEDR |= temp; //set
	temp = 0;

	// configure pupd settings
	temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx ->PUPDR &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); //clear
	pGPIOHandle -> pGPIOx ->PUPDR |= temp; //set
	temp = 0;

	//configure output type
	temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx ->OTYPER &= ~(0x1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); //clear
	pGPIOHandle -> pGPIOx ->OTYPER |= temp; //set
	temp = 0;

	// configure alternate functionality

	if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		uint8_t temp1,temp2;

		temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle -> pGPIOx -> AFR[temp1] &= ~(0xF << (4 * temp2 )); //clear
		pGPIOHandle -> pGPIOx -> AFR[temp1] |= (pGPIOHandle -> GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2 )); //set

	}

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
				GPIOA_REG_RESET();
	else if (pGPIOx == GPIOB)
				GPIOB_REG_RESET();
	else if (pGPIOx == GPIOC)
				GPIOC_REG_RESET();
	else if (pGPIOx == GPIOD)
				GPIOD_REG_RESET();
	else if (pGPIOx == GPIOE)
				GPIOE_REG_RESET();
	else if (pGPIOx == GPIOF)
				GPIOF_REG_RESET();
	else if (pGPIOx == GPIOG)
				GPIOG_REG_RESET();
	else if (pGPIOx == GPIOH)
				GPIOH_REG_RESET();
	else if (pGPIOx == GPIOI)
				GPIOI_REG_RESET();

}

/*
 * Read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)(( pGPIOx ->IDR >> PinNumber ) & 0x00000001 );

	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)( pGPIOx ->IDR );

	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if ( Value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber); // if the value = 1 set the corresponding output data register pin
	}
	else
		pGPIOx->ODR &= ~(1 << PinNumber); // if the value = 0 reset the corresponding output data register pin

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= ( 1 << PinNumber );
}

/*
 * IRQ configuration and handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void GPIO_IRQPRiorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section) +( 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR +  iprx) |= ( IRQPriority << shift_amount );

}

void GPIO_IRQHandling(uint8_t PinNumber){
	// clear the EXTI pr register corresponding pin number
	if (EXTI->PR & ( 1 << PinNumber )){
		EXTI->PR |= ( 1 << PinNumber);
	}


}

