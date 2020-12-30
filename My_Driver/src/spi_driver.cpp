
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
 * @Note              -  
 */
/********************************************************************/
void HSPI::EnableInterruptMode(void)
{
	this->SPIx->CR2 |= (1 << SPI_CR2_TXEIE); 
	this->SPIx->CR2 |= (1 << SPI_CR2_RXNEIE); 
	this->SPIx->CR2 |= (1 << SPI_CR2_ERRIE); 
}

/*********************************************************************
 * @fn      		  - DisableInterruptMode
 *
 * @brief             - Disable TXEIE ,RXNEIE, and ERRIE in RegDefStructure
 *
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  
 */
/********************************************************************/
void HSPI::DisableInterruptMode(void)
{
	this->SPIx->CR2 &= ~(1 << SPI_CR2_TXEIE); 
	this->SPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE); 
	this->SPIx->CR2 &= ~(1 << SPI_CR2_ERRIE); 
}

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
