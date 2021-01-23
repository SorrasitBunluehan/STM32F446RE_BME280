
#include "spi_driver.h"

/*********************************************************************
 * @fn      		  - Constructor
 *
 * @brief             - Use to link handle with the SPI structure
 *
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  User have to set up GPIO Altenative Function and
 * 						 in	mode in an Application.
 */
/********************************************************************/
HSPI::HSPI(SPI_RegDef_t *_spix)
{
	this->SPIx = _spix;
        this->rx_state = SPI_READY;
        this->tx_state = SPI_READY;
}

/*********************************************************************
 * @fn      		  - SetCfg
 *
 * @brief             - Set SPI Handle Specific configuration inside class
 *
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  
 */
/********************************************************************/
void HSPI::SetCfg(SPI_DEVICE_MODE _devmode, SPI_BUSCFG _buscfg,
			   SPI_DFF _dff, SPI_SLV_MNG_MODE _slvmgnmode,
			   SPI_BAUD_RATE_DIV _bauddiv,SPI_CPOL_CFG _cpol,
			   SPI_CPHA_CFG _cpha)
{
	this->DevMode = _devmode;   
	this->BusCfg = _buscfg;
	this->Dff = _dff;
	this->SlvMngMode = _slvmgnmode;
	this->BaudDiv = _bauddiv;
	this->Cpol = _cpol;
	this->Cpha = _cpha;
}

/*********************************************************************
 * @fn      		  - Init
 *
 * @brief             - Enable CLK Peripheral and  setup all Handle 
 * 						Config into SPIx Reg
 *
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  
 *
 */
/********************************************************************/
void HSPI::Init(void)
{
	/***********************
	 * Enable Peripheral CLK
	 ***********************/
	if(this->SPIx == SPI1){
		SPI1_PCLK_EN();
	}else if(this->SPIx == SPI2){
		SPI2_PCLK_EN();
	}else if(this->SPIx == SPI3){
		SPI3_PCLK_EN();
	}else if(this->SPIx == SPI4){
		SPI4_PCLK_EN();
	}
	
	/***********************
	 * Set CR1 Register
	 ***********************/
	uint32_t temp_reg = 0;

	/*
	 * Set Baud Rate 
	 */
	temp_reg |= ((this->BaudDiv) << SPI_CR1_BR);

	/*
	 * Set CPHA
	 */
	temp_reg |= ((this->Cpha) << SPI_CR1_CPHA);
	
	/*
	 * Set CPOL
	 */
	temp_reg |= ((this->Cpol) << SPI_CR1_CPOL);

	/*
	 * Set Bus Mode
	 */
	// Set BIDI Mode
	if(this->BusCfg == SPI_BUSCFG_HD){
		// Half Duplex
		temp_reg |= (1 << SPI_CR1_BIDIMODE);
		// Set BIDIOE
		if(this->DevMode == SPI_DEVICE_MODE_MASTER ){
			// Transmitted only in Half Duplex Mode (BDIOE = 1)
			temp_reg |= (1 << SPI_CR1_BIDIOE);
		}else{
			// Received only in Half Duplex Mode (BDIOE = 0)
			temp_reg &= ~(1 << SPI_CR1_BIDIOE);
		}
	}else{
		// Full Duplex & Simplex
		temp_reg &= ~(1 << SPI_CR1_BIDIMODE);
		// Set Simplex Mode RX only or Tx Only
		if(this->BusCfg == SPI_BUSCFG_SIMPLEX_RXONLY){
			temp_reg |= (1 << SPI_CR1_RXONLY);
		}else if (this->BusCfg == SPI_BUSCFG_SIMPLEX_TXONLY){
			temp_reg &= ~(1 << SPI_CR1_RXONLY);
		}
	}
				
	/*
	 * Config Slave Management 
	 */
	if (this->SlvMngMode == SPI_SLV_MNG_SW){
		temp_reg <= (1 << SPI_CR1_SSM);
		// Set SSI
		if(this->DevMode == SPI_DEVICE_MODE_MASTER ){
			temp_reg <= (1 << SPI_CR1_SSI);
		}else{
			temp_reg &= ~(1 << SPI_CR1_SSI);
		}
	}else{
		temp_reg &= ~(1 << SPI_CR1_SSM);
		// Set SSOE <By Default Only have 1 Master in the Bus (SSOE=1)>
		this->SPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}


	/*
	 * Set Master Selection 
	 */
	temp_reg |= ((this->DevMode) << SPI_CR1_MSTR);

	/*
	 * Set Data Fram Format (DFF)
	 */
	temp_reg |= ((this->Dff) << SPI_CR1_DFF);
	
	this->SPIx->CR1 = temp_reg;
}

/*********************************************************************
 * @fn      		  - EnableInterruptMode
 *
 * @brief             - Enable TXEIE ,RXNEIE, and ERRIE in RegDefStructure
 *
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  Not use for now since implement enable TXEIE and RXNEIE 
 *                      inside SPI_SendData_it and SPI_ReadData_it.
 */
/********************************************************************/
//void HSPI::EnableInterruptMode(void)
//{
//	this->SPIx->CR2 |= (1 << SPI_CR2_TXEIE); 
//	this->SPIx->CR2 |= (1 << SPI_CR2_RXNEIE); 
//	this->SPIx->CR2 |= (1 << SPI_CR2_ERRIE); 
//}

/*********************************************************************
 * @fn      	      - DisableInterruptMode
 *
 * @brief             - Disable TXEIE ,RXNEIE, and ERRIE in RegDefStructure
 *
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - Not use for now since implement enable TXEIE and RXNEIE 
 *                      inside SPI_SendData_it and SPI_ReadData_it.
 */
/********************************************************************/
//void HSPI::DisableInterruptMode(void)
//{
//	this->SPIx->CR2 &= ~(1 << SPI_CR2_TXEIE); 
//	this->SPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE); 
//	this->SPIx->CR2 &= ~(1 << SPI_CR2_ERRIE); 
//}

/*********************************************************************
 * @fn      		  - SetSSI
 *
 * @brief             - Select Slave or Master When SSM is 0
 *
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  
 */
/********************************************************************/
void HSPI::SetSSI(uint8_t EnorDi)
{
	if (EnorDi == ENABLE){
		this->SPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else{
		this->SPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*********************************************************************
 * @fn      	      - SPI_SendData_it
 *
 * @brief             - Transmitted data with interrupt mode. This API does
 *                      not do any tramission but only set up config Tx buffer,
 *                      Transmission Length, and set the status of the handle.
 *                      The transmission will be take place in Txeie IRQ Handler. 
 *
 * @param[in]         - [1] Tx bufer
 *                      [2] Transmitted Length
 *
 * @return            - SUCCESS for success setup all parameter 
 *                      FAILURE for when SPI handle is busy in other mode
 *
 * @Note              -  
 */
/********************************************************************/
uint8_t HSPI::SPI_SendData_it(uint8_t *pTxbuf, uint32_t Len)
{
    if (this->tx_state != SPI_BUSY_TX){
        this->pTxBuffer = pTxbuf;
        this->TxLen = Len;
        this->tx_state = SPI_BUSY_TX;
        this->SPIx->CR2 |= (1 << SPI_CR2_TXEIE);
        return SUCCESS;
    }else {
        return FAILURE;
    }
}

/*********************************************************************
 * @fn      	      - SPI_ReadData_it 
 *
 * @brief             - Received data with interrupt mode. This API does
 *                      not do any Receiving but only set up config Rx buffer,
 *                      Receiving Length, and set the status of the handle.
 *                      The transmission will be take place in Rxeie IRQ Handler. 
 *
 * @param[in]         - [1] Rx bufer
 *                      [2] Receiving Length
 *
 * @return            - SUCCESS for success setup all parameter 
 *                      FAILURE for when SPI handle is busy in other mode
 *
 * @Note              -  
 */
/********************************************************************/
uint8_t HSPI::SPI_ReadData_it(uint8_t *pRxbuf, uint32_t Len)
{
    if (this->rx_state != SPI_BUSY_RX){
        this->pRxBuffer = pRxbuf;
        this->RxLen = Len;
        this->rx_state = SPI_BUSY_RX;
        this->SPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
        return SUCCESS;
    }else {
        return FAILURE;
    }
}


/*********************************************************************
 * @fn      	      - SPI_IRQHandling 
 *
 * @brief             - This API check source of an interrupt and call 
 *                      spicific api accordingly.
 *                      API Called Choices:
 *                          [1] Spi_TX_Interrupt_Helper
 *                          [2] Spi_RX_Interrupt_Helper
 *                          [2] Spi_OVR_Interrupt_Helper
 *
 * @param[in]         - 
 *
 * @return            - none 
 *                    
 * @Note              - This function need to be call from ISR.   
 */
/********************************************************************/
void HSPI::SPI_IRQHandling()
{
    uint8_t temp1,temp2;
    // 1. Check for Interrupt from TXIE
    temp1 = (this->SPIx->CR2 & (1 << SPI_CR2_TXEIE));
    temp2 = (this->SPIx->SR & (1 << SPI_SR_TXE));	

    if (temp1 && temp2){
        Spi_TX_Interrupt_Helper();
    }

    // 2. Check for Interrupt from RXNEIE
    temp1 = (this->SPIx->CR2 & (1 << SPI_CR2_RXNEIE)); 
    temp2 = (this->SPIx->SR & (1 << SPI_SR_RXNE));	

    if (temp1 && temp2){
        Spi_RX_Interrupt_Helper();        
    }

    // 3. Check for Interrupt from OVR  
    temp1 = (this->SPIx->CR2 & (1 << SPI_CR2_ERRIE)); 
    temp2 = (this->SPIx->SR & (1 << SPI_SR_OVR));	

    if (temp1 && temp2){
        Spi_OVR_Interrupt_Helper();
    }
}



