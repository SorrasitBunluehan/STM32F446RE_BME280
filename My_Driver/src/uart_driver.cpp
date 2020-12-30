#include "uart_driver.h"

USART::USART(USART_RegDef_t *pUSARTx){
/* Set Usart Base Address */
    this->pUSART = pUSARTx;
    /* Set Irq number */
    if (pUSART == USART1){
        this->irq_number = 37; 
    }else if (pUSART == USART2){
        this->irq_number = 38; 
    }else if (pUSART == USART3){
        this->irq_number = 39; 
    }else if (pUSART == UART4){
        this->irq_number = 52; 
    }else if (pUSART == UART5){
        this->irq_number = 53; 
    }else if (pUSART == USART6){
        this->irq_number = 71; 
    }
}

void USART::CfgSetup(uint8_t _mode, uint8_t _NoOfStopBit,
                uint8_t _wordlength, uint8_t _parity, uint8_t _hwflowcontrol, uint32_t _baudrate)
{
    this->mode  = _mode;
    this->NoOfStopBit = _NoOfStopBit;
    this->wordlength = _wordlength;
    this->parity = _parity;    
    this->hwflowcontrol = _hwflowcontrol;
    this->baudrate = _baudrate; 
}

void USART::Init()
{
        //Temporary variable
        uint32_t tempreg=0;

    /******************************** Configuration of CR1******************************************/

        //Implement the code to enable the Clock for given USART peripheral
        this->PeriClkCtrl(ENABLE);

        //Enable USART Tx and Rx engines according to the USART_Mode configuration item
        if ( this->mode == USART_MODE_ONLY_RX)
        {
            //Implement the code to enable the Receiver bit field 
            tempreg |= (1 << USART_CR1_RE);
            tempreg &= ~( 1 << USART_CR1_TE);
        }else if (this->mode== USART_MODE_ONLY_TX)
        {
            //Implement the code to enable the Transmitter bit field 
            tempreg |= ( 1 << USART_CR1_TE);
            tempreg &= ~(1 << USART_CR1_RE);

        }else if (this->mode == USART_MODE_TXRX)
        {
            //Implement the code to enable the both Transmitter and Receiver bit fields 
            tempreg|= (1 << USART_CR1_RE);
            tempreg |= ( 1 << USART_CR1_TE);
        }

        //Implement the code to configure the Word length configuration item 
        tempreg |= this->wordlength << USART_CR1_M ;


        //Configuration of parity control bit fields
        if ( this->parity == USART_PARITY_EN_EVEN)
        {
            //Implement the code to enale the parity control 
            tempreg |= ( 1 << USART_CR1_PCE);

            //Implement the code to enable EVEN parity 
            //Not required because by default EVEN parity will be selected once you enable the parity control 

        }else if (this->parity == USART_PARITY_EN_ODD )
        {
            //Implement the code to enable the parity control 
            tempreg |= ( 1 << USART_CR1_PCE);

            //Implement the code to enable ODD parity 
            tempreg |= ( 1 << USART_CR1_PS);

        }

    //Program the CR1 register 
        this->pUSART->CR1 |= tempreg;

    /******************************** Configuration of CR2******************************************/

        tempreg=0;

        //Implement the code to configure the number of stop bits inserted during USART frame transmission 
        tempreg |= this->NoOfStopBit << USART_CR2_STOP;

        //Program the CR2 register 
        this->pUSART->CR2= tempreg;

    /******************************** Configuration of CR3******************************************/

        tempreg=0;
        
        //Configuration of USART hardware flow control 
        if ( this->hwflowcontrol == USART_HW_FLOW_CTRL_CTS)
        {
            //Implement the code to enable CTS flow control 
            tempreg |= ( 1 << USART_CR3_CTSE);


        }else if (this->hwflowcontrol == USART_HW_FLOW_CTRL_RTS)
        {
            //Implement the code to enable RTS flow control 
            tempreg |= (1 << USART_CR3_RTSE); 

        }else if (this->hwflowcontrol == USART_HW_FLOW_CTRL_CTS_RTS)
        {
            //Implement the code to enable both CTS and RTS Flow control 
            tempreg |= ( 1 << USART_CR3_CTSE);
            tempreg |= (1 << USART_CR3_RTSE); 
        }


        this->pUSART->CR3 = tempreg;

    /******************************** Configuration of BRR(Baudrate register)******************************************/

        //Implement the code to configure the baud rate
        //We will cover this in the lecture. No action required here 
        this->USART_SetBaudRate();
}

    void USART::Deinit()
    {
        if (pUSART == USART1){
            USART1_RST();
        }else if (pUSART == USART2){
            USART2_RST();
        }else if (pUSART == USART2){
            USART3_RST();
        }else if (pUSART == UART4){
            UART4_RST();
        }else if (pUSART == UART5){
            UART5_RST();
        }else if (pUSART == USART6){
            USART6_RST();
        }
    }

    /* Enable Peripheral Clk Uart */
void USART::PeriClkCtrl(uint8_t EnorDi)
{
    if (EnorDi == ENABLE){
        if (pUSART == USART1){
            USART1_PCLK_EN();
        }else if (pUSART == USART2){
            USART2_PCLK_EN();
        }else if (pUSART == USART3){
            USART3_PCLK_EN();
        }else if (pUSART == UART4){
            UART4_PCLK_EN();
        }else if (pUSART == UART5){
            UART5_PCLK_EN();
        }else if (pUSART == USART6){
            USART6_PCLK_EN();
        }
    }else{
        if (pUSART == USART1){
            USART1_PCLK_DIS();
        }else if (pUSART == USART2){
            USART2_PCLK_DIS();
        }else if (pUSART == USART3){
            USART3_PCLK_DIS();
        }else if (pUSART == UART4){
            UART4_PCLK_DIS();
        }else if (pUSART == UART5){
            UART5_PCLK_DIS();
        }else if (pUSART == USART6){
            USART6_PCLK_DIS();
        }
    }
}                                 

/* Enable Uart */
void USART::enable()
{
    this->pUSART->CR1 |= (1 << USART_CR1_UE);
}                                      

/* Disable Uart */
void USART::disable()
{
    this->pUSART->CR1 &= ~(1 << USART_CR1_UE);
}                                     

/* Enable Interrupt */
void USART::INTEnable()
{

}                                   

/* Disable Interrupt */
void USART::INTDisable()                                  
{

}

/* Set Interrupt Priority */
void USART::INTPriorityConfig(uint8_t priority)           
{


}

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

*/
void USART::SendData(uint8_t *pTxBuffer, uint32_t Len)
{

uint16_t *pdata;
//Loop over until "Len" number of bytes are transferred
uint32_t dummy_reg = this->pUSART->CR1;

	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! this->GetFlagStatus(USART_SR_TXE));

			//Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(this->wordlength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			this->pUSART->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(this->parity == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			this->pUSART->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}
	//Implement the code to wait till TC flag is set in the SR
	while( !this->GetFlagStatus(USART_SR_TC));
}




/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void USART::ReceiveData(uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
        while(!this->GetFlagStatus(USART_SR_TXE))

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(this->wordlength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(this->parity == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
                *(uint16_t*)pRxBuffer  = this->pUSART->DR & (uint16_t)0x1FF;

				//Now increment the pRxBuffer two times
				(uint16_t*)pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (this->pUSART->DR & (uint8_t)0xFF);
				 
				 //Increment the pRxBuffer
                 pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(this->parity == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = this->pUSART->DR & (uint8_t)0xFF;
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = this->pUSART->DR & (uint8_t)0x7F;
			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

/*********************************************************************
 * @fn      		  - USART_SendDataWithIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs 

 */
uint8_t USART::SendDataIT(uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = this->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		this->tx_len = Len;
		this->pTxbuf= pTxBuffer;
		this->TxBusyState = USART_BUSY_IN_TX ;

		//Implement the code to enable interrupt for TXE
        this->pUSART->CR1 |= (1 << USART_CR1_TXEIE);

		//Implement the code to enable interrupt for TC 
        this->pUSART->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txstate;

}


/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs 

 */
uint8_t USART::ReceiveDataIT(uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = this->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		this->rx_len = Len;
		this->pRxbuf= pRxBuffer;
		this->RxBusyState = USART_BUSY_IN_RX ;

		//Implement the code to enable interrupt for RXNE
		this->pUSART->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxstate;

}



uint8_t USART::GetFlagStatus(uint8_t _flagname){
    uint8_t temp = 0;
    temp = (this->pUSART->SR >> _flagname) & 0x1;
    return temp;
}

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Resolve all the TODOs

 */
void USART::USART_SetBaudRate()
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(this->pUSART == USART1 || this->pUSART == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = I2C_RCCGetAPB2ClkFREQ();
  }else
  {
	   PCLKx = I2C_RCCGetAPB1ClkFREQ();
  }

  //Check for OVER8 configuration bit
  if( (this->pUSART->CR1 >> USART_CR1_OVER8) & 0x1)
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *this->baudrate));
  }else
  {
	   //over sampling by 16
	   usartdiv = ((25 * PCLKx) / (4 *this->baudrate));
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(this->pUSART->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100) & ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  this->pUSART->BRR = tempreg;
}

