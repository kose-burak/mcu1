/*
 * stm32f407xx_i2c_driver.c
 */
#include "stm32f407xx_i2c_driver.h"

uint16_t AHB_PreScaller[8] = { 2, 4, 8, 16, 64, 128, 256, 512 };
uint8_t APB1_PreScaller[4] = { 2, 4, 8, 16 };

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt (I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt (I2C_Handle_t *pI2CHandle);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START );
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP );
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); // SlaveAddr is slave address + r/w bit = 0
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= (1); // SlaveAddr is slave address + r/w bit = 1
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	// check for device mode
	if ( pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL ) ) {
		// device in master mode
		if ( pI2CHandle->TxRxState == I2C_BUSY_IN_RX ) {
			if ( pI2CHandle->RxSize > 1 ) {
				// disable the ack
				I2C_ManageAcking( pI2CHandle->pI2Cx, I2C_ACK_DISABLE );

				// clear the ADDR flag
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				( void ) dummy_read;
			}
		}
		else { // RX is not busy
			// clear the ADDR flag
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			( void ) dummy_read;
		}
	}
	else {
		// device in slave mode
		// clear the ADDR flag
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		( void ) dummy_read;
	}
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		pI2Cx->CR1 |= ( 1 << I2C_CR1_PE );
	}
	else {
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_PE );
	}
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if ( EnorDi == ENABLE ) {
		if ( pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		}
		else if ( pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		}
		else if ( pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	}
	else {
		if (pI2Cx == I2C1)
			I2C1_PCLK_DI();
		else if (pI2Cx == I2C2)
			I2C2_PCLK_DI();
		else if (pI2Cx == I2C3)
			I2C3_PCLK_DI();
	}
}
uint32_t RCC_GetPLLOutputClock()
{
	return 0;
}



uint32_t RCC_GetPCLK1Value (void)
{
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ( ( RCC->CFGR >> 2 ) & 0x3 );

	if ( clksrc == 0 ) {
		SystemClk = 16000000;
	}
	else if ( clksrc == 1 ) {
		SystemClk = 8000000;
	}
	else if ( clksrc == 2 ) {
		SystemClk = RCC_GetPLLOutputClock();
	}

	// for ahbp
	temp = ( ( RCC->CFGR >> 4 ) & 0xF );

	if ( temp < 8 ) {
		ahbp = 1;
	}
	else {
		ahbp = AHB_PreScaller[temp - 8];
	}

	// for apb1
	temp = ( ( RCC->CFGR >> 10 ) & 0x7 );

	if ( temp < 4 ) {
		apb1p = 1;
	}
	else {
		apb1p = APB1_PreScaller[temp - 4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}

/*
 *  Initialize and de-initialize the I2Cx peripheral
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	// ack control bit
	tempreg |= ( pI2CHandle->I2C_Config.I2C_AckControl << 10 );
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 |= ( tempreg & 0x3F );

	// program the device address
	tempreg = 0;
	tempreg |= ( pI2CHandle->I2C_Config.I2C_DeviceAddress << 1 );
	tempreg |= ( 1 << 14 ); // keep 14th bit 1 always according to reference manual
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if ( pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM ) {
		// mode is standard mode
		ccr_value = RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed );
		tempreg |= ( ccr_value & 0xFFF );
	}
	else {
		// fast mode
		tempreg |= ( 1 << 15 );
		tempreg |= ( pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14 );
		if ( pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2 ) {
			ccr_value = RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed );
		}
		else {
			ccr_value = RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed );
		}
		tempreg |= ( ccr_value & 0xFFF );
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	// TRISE configuration
	if ( pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM ) {
		// mode is standard mode
		tempreg = ( RCC_GetPCLK1Value() / 1000000U ) + 1 ;
	}
	else {
		// fast mode
		tempreg = ( ( RCC_GetPCLK1Value() * 300 ) / 1000000000U ) + 1 ;
	}
	pI2CHandle->pI2Cx->TRISE = ( tempreg & 0x3F );

}
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->SR1 & FlagName)
		return FLAG_SET;

	return FLAG_RESET;
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	// generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//confirm that start generation is completed by checking the SB flag in SR1
	// until SB is cleared SCL pulled to low ( stretched )
	while (  !	I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB ) );

	// send the address of the slave with r/w bit set to w(0) total 8 bits
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	// confirm that address phase is completed by checking the ADDR flag in the SR1
	while (  !	I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR ) );

	// clear the ADDR flag according to its software sequence
	// until ADDR is cleared SCL pulled to low ( stretched )
	I2C_ClearADDRFlag(pI2CHandle);

	// send data until length becomes zero
	while ( Len > 0 ) {
		while (   !	I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	// when len = 0 wait for TxE = 1 and BTF = 1 before generating stop condition
	// TxE = 1, BTF = 1 means both SR and DR are empty and next transmission should begin
	// when BTF = 1 SCL pulled to low ( stretched )
	while (   !	I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while (   !	I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	// generate STOP condition
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr)
{
	// generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//confirm that start generation is completed by checking the SB flag in SR1
	// until SB is cleared SCL pulled to low ( stretched )
	while (  !	I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB ) );

	// send the address of the slave with r/w bit set to r(1) total 8 bits
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	// confirm that address phase is completed by checking the ADDR flag in the SR1
	while (  !	I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR ) );

	// procedure to read only 1 byte from slave
	if ( Len == 1 ) {
		// disable acking
		I2C_ManageAcking( pI2CHandle->pI2Cx, I2C_ACK_DISABLE );

		// generate stop condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// wait until RxNE becomes 1
		while (   !	I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE ) );

		// read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;


		return ;
	}

	// procedure to read data from slave when Len > 1
	if ( Len > 1 ) {
		//	clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// read the data until Len = 0
		for ( uint32_t i = Len; i > 0; i--) {
			// wait until RxNE becomes 1
			while (   !	I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE ) );

			if ( i == 2 ) {
				// clear the ack bit
				I2C_ManageAcking( pI2CHandle->pI2Cx, I2C_ACK_DISABLE );

				// generate stop condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			// read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			// increment the buffer address
			pRxBuffer++;
		}
	}
	// re-enable acking
	if ( pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE ) {
		I2C_ManageAcking( pI2CHandle->pI2Cx, I2C_ACK_ENABLE );
	}
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if ( EnorDi == I2C_ACK_ENABLE ) {
		// enable acking
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK );
	}
	else {
		// disable acking
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK );
	}
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		// implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		// implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	return busystate;
}

static void I2C_MasterHandleTXEInterrupt (I2C_Handle_t *pI2CHandle)
{
	if ( pI2CHandle->TxLen > 0 ) {
		// load the data to DR
		pI2CHandle->pI2Cx->DR = *( pI2CHandle->pTxBuffer );
		// decrement the TxLen and increment the buffer address
		pI2CHandle->TxLen--;
		pI2CHandle->pTxBuffer++;
	}
}

static void I2C_MasterHandleRXNEInterrupt( I2C_Handle_t *pI2CHandle)
{
	// data reception
	if ( pI2CHandle->RxSize == 1 ) {
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}
	if ( pI2CHandle->RxSize > 1 ) {
		if ( pI2CHandle->RxLen == 2 ) {
			// clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE );
		}
		// read data from DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}
	if ( pI2CHandle->RxLen == 0 ) {
		// close the I2C data receive and notify the app
		// generate stop condition
		if ( pI2CHandle->Sr == I2C_DISABLE_SR )
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		I2C_CloseReceiveData(pI2CHandle);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN );
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN );


	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB );

	if ( temp1 && temp3 ) {
		// interrupt is generated because of SB event // this block not executed for slave mode hence SB is always zero
		if ( pI2CHandle->TxRxState == I2C_BUSY_IN_TX ) {
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if ( pI2CHandle->TxRxState == I2C_BUSY_IN_RX ) {
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,  pI2CHandle->DevAddr);
		}
	}

	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR );

	if ( temp1 && temp3 ) {
		// interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF );

	if ( temp1 && temp3 ) {
		// BTF flag is set
		if ( pI2CHandle->TxRxState == I2C_BUSY_IN_TX ) {
			// check TXE is also set
			if ( pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE ) ) {
				// BTF = 1 , TXE = 1
				if ( pI2CHandle->TxLen == 0 ) {
					// generate stop condtion
					if ( pI2CHandle->Sr == I2C_DISABLE_SR )
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					// reset all elements of handle structure
					I2C_CloseSendData(pI2CHandle);

					// notify the app the transmission is complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT );
				}
			}
		}
		else if ( pI2CHandle->TxRxState == I2C_BUSY_IN_RX ) {
			;
		}
	}

	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF );

	if ( temp1 && temp3 ) {
		// STOPF flag is set
		// clear the STOPF --> read SR1 --> write CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		// notify the app the STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP );
	}

	//5. Handle For interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE );

	if ( temp1 && temp2 && temp3 ) {
		// check if it is master mode
		if ( pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL ) ) {
			// TXE flag is set
			if ( pI2CHandle->TxRxState == I2C_BUSY_IN_TX ) {
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
	}

	//6. Handle For interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE );

	if ( temp1 && temp2 && temp3 ) {
		// check for device mode
		if ( pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL ) ) {
			// device is master mode and RXNE flag is set
			if ( pI2CHandle->TxRxState == I2C_BUSY_IN_RX ) {
				I2C_MasterHandleRXNEInterrupt( pI2CHandle);
			}
		}
	}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	// disable the ITBUFEN control bit and ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN );

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	if ( pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE )
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE );
}
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	// disable the ITBUFEN control bit and ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN );

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag

		//Implement the code to notify the application about the error

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag

		//Implement the code to notify the application about the error
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag

		//Implement the code to notify the application about the error
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag

		//Implement the code to notify the application about the error
	}

}

