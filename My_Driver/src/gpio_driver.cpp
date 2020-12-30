/*
 * stm32f446re_gpio_driver.c
 *
 *  Created on: Dec 3, 2020
 *      Author: bsorr
 */


#include <gpio_driver.h>

/*********************************************************************
 * @fn      		  - Constructor
 *
 * @brief             - This function Initialize all Private parameter
 *
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
/********************************************************************/
GPIO::GPIO(GPIO_RegDef_t *pGPIO){
	this->pinnumber = GPIO_PIN0;
	this->mode		= GPIO_MODE_IN;
	this->ospeed	= GPIO_ST_FS;			
	this->otype		= GPIO_OT_PP;			
	this->pupd		= GPIO_PUPD_NPUPD;
	this->altmode	= GPIO_AF_0;
	this->pGPIOX 	= pGPIO;

}

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
/********************************************************************/
void GPIO::PeriClkCtrl(uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (this->pGPIOX == GPIOA) {
			GPIOA_PCLK_EN();
		}else if(this->pGPIOX == GPIOB){
			GPIOB_PCLK_EN();
		}else if(this->pGPIOX == GPIOC){
			GPIOC_PCLK_EN();
		}else if(this->pGPIOX == GPIOD){
			GPIOD_PCLK_EN();
		}else if(this->pGPIOX == GPIOE){
			GPIOE_PCLK_EN();
		}else if(this->pGPIOX == GPIOF){
			GPIOF_PCLK_EN();
		}else if(this->pGPIOX == GPIOG){
			GPIOG_PCLK_EN();
		}else if(this->pGPIOX == GPIOH){
			GPIOH_PCLK_EN();
		}
	}else
	{
		if (this->pGPIOX == GPIOA) {
			GPIOA_PCLK_DIS();
		}else if(this->pGPIOX == GPIOB){
			GPIOB_PCLK_DIS();
		}else if(this->pGPIOX == GPIOC){
			GPIOC_PCLK_DIS();
		}else if(this->pGPIOX == GPIOD){
			GPIOD_PCLK_DIS();
		}else if(this->pGPIOX == GPIOE){
			GPIOE_PCLK_DIS();
		}else if(this->pGPIOX == GPIOF){
			GPIOF_PCLK_DIS();
		}else if(this->pGPIOX == GPIOG){
			GPIOG_PCLK_DIS();
		}else if(this->pGPIOX == GPIOH){
			GPIOH_PCLK_DIS();
		}
	}
}

/*********************************************************************
 * Funciton OverLoad for Setting up Config Parameter of GPIO Class
/*********************************************************************/
void GPIO::GpioSetupCfg (uint8_t _pinnum, uint8_t _mode, uint8_t _otype, 
						 uint8_t _ospeed, uint8_t _pupd, uint8_t _altmode)
{
	this->pinnumber = _pinnum;
	this->mode		= _mode;
	this->otype		= _otype;
	this->ospeed 	= _ospeed;
	this->pupd		= _pupd;
	this->altmode 	= _altmode;
}

void GPIO::GpioSetupCfg (uint8_t _pinnum, uint8_t _mode, uint8_t _otype, 
						 uint8_t _ospeed, uint8_t _pupd )
{
	this->pinnumber = _pinnum;
	this->mode		= _mode;
	this->otype		= _otype;
	this->ospeed 	= _ospeed;
	this->pupd		= _pupd;
}


void GPIO::GpioSetupCfg (uint8_t _pinnum, uint8_t _mode, uint8_t _pupd)
{
	this->pinnumber = _pinnum;
	this->mode		= _mode;
	this->pupd		= _pupd;
}


/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function setup register of a GPIO port according to the handle information
 *
 * @param[in]         - Handler function
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO::Init(){

	// Enable Peripheral Clock Control
	PeriClkCtrl(ENABLE);

	uint32_t temp = 0;
	//1. Configure mode
	if (this->mode <= GPIO_MODE_ANALOG)
	{
		temp = (this-> mode << (2*this->pinnumber));
		/* Reset target register field first before set */
		this->pGPIOX->MODER &= ~(0x3 << (2*this->pinnumber));
		/* Set the Mode of the Register*/
		this->pGPIOX->MODER |= temp;
		temp = 0;
	}else
	{
		if (this->pGPIOX->MODER == GPIO_MODE_INT_RT)
		{
			// Config EXTI Rising Edge detection
			EXTI->RTSR 	|= ( 1 << this->pinnumber);
			// Clear EXTI Falling Edge detection
			EXTI->FTSR 	&= ~( 1 << this->pinnumber);
		}else if (this->pGPIOX->MODER == GPIO_MODE_INT_FT)
		{
			// Config EXTI Falling Edge detection
			EXTI->FTSR 	|= ( 1 << this->pinnumber);
			// Clear EXTI Falling Edge detection
			EXTI->RTSR 	&= ~( 1 << this->pinnumber);
		}else if (this->pGPIOX->MODER == GPIO_MODE_INT_FRT)
		{
			// Config EXTI Falling Edge detection
			EXTI->FTSR 	|= ( 1 << this->pinnumber);
			// Config EXTI Rising Edge detection
			EXTI->RTSR 	|= ( 1 << this->pinnumber);
		}

		// Config SYSCFG Register to select which GPIO will link to which EXTI
		SYSCFG_PCLK_EN();
		uint8_t temp1, temp2;
		temp1 = this->pinnumber/4;
		temp2 = this->pinnumber%4;
		if (this->pGPIOX == GPIOA){
			SYSCFG->EXTICR[temp1] |= (SYSCFG_PORTA << temp2*4);
		}else if(this->pGPIOX == GPIOB){
			SYSCFG->EXTICR[temp1] |= (SYSCFG_PORTB << temp2*4);
		}else if(this->pGPIOX == GPIOC){
			SYSCFG->EXTICR[temp1] |= (SYSCFG_PORTC << temp2*4);
		}else if(this->pGPIOX == GPIOD){
			SYSCFG->EXTICR[temp1] |= (SYSCFG_PORTD << temp2*4);
		}else if(this->pGPIOX == GPIOE){
			SYSCFG->EXTICR[temp1] |= (SYSCFG_PORTE << temp2*4);
		}else if(this->pGPIOX == GPIOF){
			SYSCFG->EXTICR[temp1] |= (SYSCFG_PORTF << temp2*4);
		}else if(this->pGPIOX == GPIOG){
			SYSCFG->EXTICR[temp1] |= (SYSCFG_PORTG << temp2*4);
		}else if(this->pGPIOX == GPIOH){
			SYSCFG->EXTICR[temp1] |= (SYSCFG_PORTH << temp2*4);
		}

		// Config EXTI MASK REG to choose which EXTI will enable to send interrupt signal to MCU
		EXTI->IMR |= (1 << this->pinnumber);
	}

	//2. Configure speed
	temp = 0;
	temp = (this->ospeed << (2*this->pinnumber));

	/* Reset target register field first before set */
	this->pGPIOX->OSPEEDR &= ~(0x3 << (2*this->pinnumber));
	this->pGPIOX->OSPEEDR |= temp;
	
	//3. Pull-up ,Pull-down config
	temp = 0;
	temp = (this->pupd << (2*this->pinnumber));
	/* Reset target register field first before set */
	this->pGPIOX->PUPDR &= ~(0x3 << (2*this->pinnumber));
	this->pGPIOX->PUPDR |= temp;

	//4. Config output type
	temp = 0;
	temp = (this->pupd << this->pinnumber);
	/* Reset target register field first before set */
	this->pGPIOX->PUPDR &= ~(0x1 << this->pinnumber);
	this->pGPIOX->OTYPER |= temp;

	//5. Config AF
	uint32_t temp1, temp2 = 0;
	temp1 = this->pinnumber/8;
	temp2 = this->pinnumber % 8;
	if (this->mode == GPIO_MODE_ALTFN)
	{
		/* Reset target register field first before set */
		this->pGPIOX->AFR[temp1] &= ~(0xF << 4*temp2);
		this->pGPIOX->AFR[temp1] |= (this->altmode << 4*temp2);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function clear peripheral by setting register RCC_AHB1RSTR
 *
 * @param[in]         - Base Address of GPIO port that you want to deinit
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO::DeInit(){
	if (this->pGPIOX == GPIOA) {
		GPIOA_RST();
	}else if(this->pGPIOX == GPIOB){
		GPIOB_RST();
	}else if(this->pGPIOX == GPIOC){
		GPIOC_RST();
	}else if(this->pGPIOX == GPIOD){
		GPIOD_RST();
	}else if(this->pGPIOX == GPIOE){
		GPIOE_RST();
	}else if(this->pGPIOX == GPIOF){
		GPIOF_RST();
	}else if(this->pGPIOX == GPIOG){
		GPIOG_RST();
	}else if(this->pGPIOX == GPIOH){
		GPIOH_RST();
	}
}

/*********************************************************************
 * @fn      		  - GPIO_ReadPin 
 *
 * @brief             - Read data from spicific pin from specific port
 *
 * @param[in]         - Specific Port 
 * @param[in]         - Specific Pin
 * @param[in]         -
 *
 * @return            -  Bool
 *
 * @Note              -  none
 */
uint8_t GPIO::ReadPin(uint8_t pinnumber){
	uint8_t temp = 0;
	temp = (uint8_t)((this->pGPIOX->IDR >> pinnumber) & 0x00000001);
	return temp;
}

uint16_t GPIO::ReadPort(){
	uint16_t temp = 0;
	temp = (uint16_t)this->pGPIOX->IDR;
	return temp;
}

/*
 * Write API
 */
void GPIO::WritePin(uint8_t pinnumber, uint8_t data){
	if (data == SET)
	{
		this->pGPIOX->ODR |= (data << pinnumber);
	} else
	{
		this->pGPIOX->ODR &= ~(1 << pinnumber);
	}
}

void GPIO::WritePort(uint16_t data){
	this->pGPIOX->ODR |= data;
}
void GPIO::ToggleOutputPin(uint8_t pinnumber){
	this->pGPIOX->ODR ^= (1 << pinnumber);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             - Config Enable or Disable NVIC IRQ inside Processor Core
 *
 * @param[in]         - IRQ number
 * @param[in]         - Priority
 * @param[in]         - Enable or Disable
 *
 * @return            -  SUCCESS OR FAILURE
 *
 * @Note              -  none
 */
uint8_t GPIO::IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		if (IRQNumber < 32){
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber >= 32 && IRQNumber < 64){
			*NVIC_ISER1 |= (1 << IRQNumber%32);
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			*NVIC_ISER2 |= (1 << IRQNumber%64);
		}else{
			return FAILURE;
		}
	}else{
		if (IRQNumber < 32){
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber >= 32 && IRQNumber < 64){
			*NVIC_ICER1 |= (1 << IRQNumber%32);
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			*NVIC_ICER2 |= (1 << IRQNumber%64);
		}else{
			return FAILURE;
		}
	}

	uint8_t iprx,iprx_section;
	iprx = IRQNumber/4;
	iprx_section = IRQNumber%4;

	*(NVIC_PR_BASE_ADDR + iprx*4) |=  (IRQPriority << (8*iprx_section + (8 - NO_PR_BITS_IMPLEMENTED)));


	return SUCCESS;
}
void GPIO::IRQHandling(uint8_t pinnumber){
	if (EXTI->PR & (1 << pinnumber))
	{
		EXTI->PR |= (1 << pinnumber);
	}
}
