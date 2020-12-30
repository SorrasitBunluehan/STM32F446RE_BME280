/*
 * stm32f446re.h
 *
 *  Created on: Dec 3, 2020
 *      Author: bsorr
 */

#ifndef INC_STM32F446RE_H_
#define INC_STM32F446RE_H_

#include <stdint.h>
#include <stddef.h>


#define __vo volatile
#define __weak __attribute__((weak))

/********************************** START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 			4


/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


/*******************************************************************************************************
* Peripherals Base Address Macro
*******************************************************************************************************/
/*
 *	Base Address of Flash and SRAM
 */
#define FLASH_BASEADDR 		0x08000000U
#define SRAM1_BASEADDR 		0x20000000U
#define SRAM2_BASEADDR 		0x2001C000U
#define ROM_BASEADDR 		0x0x1FFF0000U
#define SRAM SRAM1_BASEADDR

/*
 *	Base Address of AHB and APB
 */
#define PERIPH_BASE		0x40000000U
#define APB1_BASEADDR 		PERIPH_BASE
#define APB2_BASEADDR 		0x40010000U
#define AHB1_BASEADDR 		0x40020000U
#define AHB2_BASEADDR 		0x50000000U


/*
 *	Peripheral Address on APB1
 */
#define SPI2_BASEADDR		(APB1_BASEADDR+0x3800)
#define SPI3_BASEADDR		(APB1_BASEADDR+0x3C00)
#define UART4_BASEADDR		(APB1_BASEADDR+0x4C00)
#define UART5_BASEADDR		(APB1_BASEADDR+0x5000)
#define I2C1_BASEADDR		(APB1_BASEADDR+0x5400)
#define I2C2_BASEADDR		(APB1_BASEADDR+0x5800)
#define I2C3_BASEADDR		(APB1_BASEADDR+0x5C00)
#define CAN1_BASEADDR		(APB1_BASEADDR+0x6400)
#define CAN2_BASEADDR		(APB1_BASEADDR+0x6800)
#define USART2_BASEADDR		(APB1_BASEADDR+0x4400)
#define USART3_BASEADDR		(APB1_BASEADDR+0x4800)

/*
 *	Peripheral Address on APB2
 */
#define SPI1_BASEADDR		(APB2_BASEADDR+0x3000)
#define SPI4_BASEADDR		(APB2_BASEADDR+0x3400)
#define SYSCFG_BASEADDR		(APB2_BASEADDR+0x3800)
#define EXTI_BASEADDR		(APB2_BASEADDR+0x3C00)
#define USART1_BASEADDR		(APB2_BASEADDR+0x1000)
#define USART6_BASEADDR		(APB2_BASEADDR+0x1400)


/*
 *	Peripheral Address on AHB1
 */
#define GPIOA_BASEADDR		(AHB1_BASEADDR+0x0000)
#define GPIOB_BASEADDR		(AHB1_BASEADDR+0x0400)
#define GPIOC_BASEADDR		(AHB1_BASEADDR+0x0800)
#define GPIOD_BASEADDR		(AHB1_BASEADDR+0x0C00)
#define GPIOE_BASEADDR		(AHB1_BASEADDR+0x1000)
#define GPIOF_BASEADDR		(AHB1_BASEADDR+0x1400)
#define GPIOG_BASEADDR		(AHB1_BASEADDR+0x1800)
#define GPIOH_BASEADDR		(AHB1_BASEADDR+0x1C00)
#define RCC_BASEADDR		(AHB1_BASEADDR+0x3800)


/*******************************************************************************************************
* Peripheral Register Definition Structure Macro
*******************************************************************************************************/

/*
 * 	GPIO Peripheral Register Structure 
 */
typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
} GPIO_RegDef_t; 				/*!< GPIO Register Definition Structure >!*/


/*
 * 	EXTI Register Structure
 */
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t; 				/*!< EXTI Register Definition Structure >!*/


/*
 * 	SYSCFG Register Structure
 */
typedef struct
{
	__vo uint32_t MEMRMP;			/*!< memory remap register >!*/
	__vo uint32_t PMC;				/*!< peripheral mode configuration register >!*/
	__vo uint32_t EXTICR[4];			/*!< external interrupt configuration register 1,2,3,4 >!*/
	uint32_t RSV1[2];				/*!< Reserve >!*/
	__vo uint32_t CMPCR;			/*!< Compensation cell control register >!*/
	uint32_t RSV2[2];				/*!< Reserve >!*/
	__vo uint32_t CFGR;				/*!< configuration register >!*/
}SYSCFG_RegDef_t; 					/*!< EXTI Register Definition Structure >!*/

/*
 * 	RCC Peripheral Register Structure 
 */
typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHBRSTR[3]; 				/*!< AHB1RSTR, AHB2RSTR, AHB3RSTR >!*/
	uint32_t RSV0; 							/*!< RESERVE >!*/
	__vo uint32_t APBRSTR[2]; 				/*!< APB1RSTR, APB2RSTR>!*/
	uint32_t RSV1[2]; 						/*!< RESERVE >!*/
	__vo uint32_t AHBENR[3]; 				/*!< AHB1ENG, AHB2ENG, AHB3ENG>!*/
	uint32_t RSV2; 							/*!< RESERVE >!*/
	__vo uint32_t APBENR[2]; 				/*!< ABP1ENR, ABP2ENR>!*/
	uint32_t RSV3[2]; 						/*!< RESERVE >!*/
	__vo uint32_t AHBLPENR[3]; 				/*!< AHBLPENR 1,2,3>!*/
	uint32_t RSV4; 							/*!< RESERVE >!*/
	__vo uint32_t APBLPENR[2]; 				/*!< APBLPENR 1,2>!*/
	uint32_t RSV5[2]; 						/*!< RESERVE >!*/
	__vo uint32_t BDCR; 					/*!< >!*/
	__vo uint32_t CSR; 						/*!< >!*/
	uint32_t RSV6[2]; 						/*!< RESERVE >!*/
	__vo uint32_t SSCGR; 					/*!< >!*/
	__vo uint32_t PLLI2SCFGR; 				/*!< >!*/
	__vo uint32_t PLLSAICFGR; 				/*!< >!*/
	__vo uint32_t DCKCFGR; 					/*!< >!*/
	__vo uint32_t CKGATENR; 				/*!< >!*/
	__vo uint32_t DCKCFGR2; 				/*!< >!*/

}RCC_RegDef_t; 				/*!< GPIO Register Definition Structure >!*/ 

/*
 * SPI register definition structure
 */
typedef struct
{
	__vo uint32_t CR1;       	 /*!< Control Register,     										Address offset: 0x00 */
	__vo uint32_t CR2;       	 /*!< Control Register,     										Address offset: 0x04 */
	__vo uint32_t SR;        	 /*!< Status Register,     										Address offset: 0x08 */
	__vo uint32_t DR;        	 /*!< Data Register,     										Address offset: 0x0C */
	__vo uint32_t CRCPR;     	 /*!< CRC polynomial register,     										Address offset: 0x10 */
	__vo uint32_t RXCRCR;    	 /*!< RX CRC register,     										Address offset: 0x14 */
	__vo uint32_t TXCRCR;    	 /*!< TX CRC register,     										Address offset: 0x18 */
	__vo uint32_t I2SCFGR;   	 /*!< SPI_I2S configuration register,     										Address offset: 0x1C */
	__vo uint32_t I2SPR;     	 /*!< SPI_I2S prescaler register,     										Address offset: 0x20 */
} SPI_RegDef_t;

/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
  __vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t OAR1;       /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t OAR2;       /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t DR;         /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t SR1;        /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t SR2;        /*!< TODO,     										Address offset: 0x18 */
  __vo uint32_t CCR;        /*!< TODO,     										Address offset: 0x1C */
  __vo uint32_t TRISE;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t FLTR;       /*!< TODO,     										Address offset: 0x24 */
}I2C_RegDef_t;

/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t BRR;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t CR3;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t GTPR;       /*!< TODO,     										Address offset: 0x18 */
} USART_RegDef_t;

/*******************************************************************************************************
* Peripheral Structure Instant Macro
*******************************************************************************************************/

/*
 * 	GPIOx structure instant 
 */
#define GPIOA 	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 	((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 	((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 	((GPIO_RegDef_t*)GPIOH_BASEADDR)

/*
 * 	RCC structure instant 
*/
#define RCC 	((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * 	EXTI structure instant
*/
#define EXTI 	((EXTI_RegDef_t*)EXTI_BASEADDR)

/*
 * 	SYSCFG structure instant
*/
#define SYSCFG 	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * 	SPI structure instant
*/
#define SPI1	((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2	((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3	((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4	((SPI_RegDef_t*)SPI4_BASEADDR)

/*
 * 	SPI structure instant
*/
#define I2C1	((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2	((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3	((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)



/*******************************************************************************************************
* Peripheral CLK Enable/Disable Macro
*******************************************************************************************************/

/*
 * CLK Disable for SPI
 */
#define SPI_PCLK_DIS() 	(RCC->AHBENR[1] &= ~(1 << 12))

/*
 * CLK Enable for GPIO
 */
#define GPIOA_PCLK_EN() 	(RCC->AHBENR[0] |= (1 << 0))
#define GPIOB_PCLK_EN() 	(RCC->AHBENR[0] |= (1 << 1))
#define GPIOC_PCLK_EN() 	(RCC->AHBENR[0] |= (1 << 2))
#define GPIOD_PCLK_EN() 	(RCC->AHBENR[0] |= (1 << 3))
#define GPIOE_PCLK_EN() 	(RCC->AHBENR[0] |= (1 << 4))
#define GPIOF_PCLK_EN() 	(RCC->AHBENR[0] |= (1 << 5))
#define GPIOG_PCLK_EN() 	(RCC->AHBENR[0] |= (1 << 6))
#define GPIOH_PCLK_EN() 	(RCC->AHBENR[0] |= (1 << 7))

/*
 * CLK Disable for GPIO
 */
#define GPIOA_PCLK_DIS() 	(RCC->AHBENR[0] &= ~(1 << 0))
#define GPIOB_PCLK_DIS() 	(RCC->AHBENR[0] &= ~(1 << 1))
#define GPIOC_PCLK_DIS() 	(RCC->AHBENR[0] &= ~(1 << 2))
#define GPIOD_PCLK_DIS() 	(RCC->AHBENR[0] &= ~(1 << 3))
#define GPIOE_PCLK_DIS() 	(RCC->AHBENR[0] &= ~(1 << 4))
#define GPIOF_PCLK_DIS() 	(RCC->AHBENR[0] &= ~(1 << 5))
#define GPIOG_PCLK_DIS() 	(RCC->AHBENR[0] &= ~(1 << 6))
#define GPIOH_PCLK_DIS() 	(RCC->AHBENR[0] &= ~(1 << 7))

/*
 * CLK Enable for I2C
 */
#define I2C1_PCLK_EN() 		(RCC->APBENR[0] |= (1 << 21))
#define I2C2_PCLK_EN() 		(RCC->APBENR[0] |= (1 << 22))
#define I2C3_PCLK_EN() 		(RCC->APBENR[0] |= (1 << 23))

/*
 * CLK Disable for I2C
 */
#define I2C1_PCLK_DIS() 	(RCC->APBENR[0] &= ~(1 << 21))
#define I2C2_PCLK_DIS() 	(RCC->APBENR[0] &= ~(1 << 22))
#define I2C3_PCLK_DIS() 	(RCC->APBENR[0] &= ~(1 << 23))

/*
 * CLK Enable for SPI
 */
#define SPI1_PCLK_EN() 		(RCC->APBENR[1] |= (1 << 12))
#define SPI2_PCLK_EN() 		(RCC->APBENR[0] |= (1 << 14))
#define SPI3_PCLK_EN() 		(RCC->APBENR[0] |= (1 << 15))
#define SPI4_PCLK_EN() 		(RCC->APBENR[1] |= (1 << 13))

/*
 * CLK Disable for SPI
 */
#define SPI1_PCLK_DIS() 	(RCC->APBENR[1] &= ~(1 << 12))
#define SPI2_PCLK_DIS() 	(RCC->APBENR[0] &= ~(1 << 14))
#define SPI3_PCLK_DIS() 	(RCC->APBENR[0] &= ~(1 << 15))
#define SPI4_PCLK_DIS() 	(RCC->APBENR[1] &= ~(1 << 13))

/*
 * CLK Enable for SYSCFG
 */
#define SYSCFG_PCLK_EN() 		(RCC->APBENR[1] |= (1 << 14))

/*
 * SYSCFG_EXTICR port choice
 */
#define SYSCFG_PORTA 0x0
#define SYSCFG_PORTB 0x1
#define SYSCFG_PORTC 0x2
#define SYSCFG_PORTD 0x3
#define SYSCFG_PORTE 0x4
#define SYSCFG_PORTF 0x5
#define SYSCFG_PORTG 0x6
#define SYSCFG_PORTH 0x7

/*
 * CLK Disable for SYSCFG
 */
#define SYSCFG_PCLK_DIS() 		(RCC->APBENR[1] &= ~(1 << 14))

/*
 * CLK Enable for USART/UART
 */
#define USART1_PCLK_EN() 		(RCC->APBENR[1] |= (1 << 4))
#define USART2_PCLK_EN() 		(RCC->APBENR[0] |= (1 << 17))
#define USART3_PCLK_EN() 		(RCC->APBENR[0] |= (1 << 18))
#define UART4_PCLK_EN() 		(RCC->APBENR[0] |= (1 << 19))
#define UART5_PCLK_EN() 		(RCC->APBENR[0] |= (1 << 20))
#define USART6_PCLK_EN() 		(RCC->APBENR[1] |= (1 << 5))

/*
 * CLK Disable for USART/UART
 */
#define USART1_PCLK_DIS() 		(RCC->APBENR[1] &= ~(1 << 4))
#define USART2_PCLK_DIS() 		(RCC->APBENR[0] &= ~(1 << 17))
#define USART3_PCLK_DIS() 		(RCC->APBENR[0] &= ~(1 << 18))
#define UART4_PCLK_DIS() 		(RCC->APBENR[0] &= ~(1 << 19))
#define UART5_PCLK_DIS() 		(RCC->APBENR[0] &= ~(1 << 20))
#define USART6_PCLK_DIS() 		(RCC->APBENR[1] &= ~(1 << 5))

/*******************************************************************************************************
* Reset Peripheral Macro
*******************************************************************************************************/
/*
 *  Reset GPIO Peripheral
 */
#define GPIOA_RST() do{ (RCC->AHBRSTR[0] |= (1 << 0)); (RCC->AHBRSTR[0] &= ~(1 << 0)); } while(0)
#define GPIOB_RST() do{ (RCC->AHBRSTR[0] |= (1 << 1)); (RCC->AHBRSTR[0] &= ~(1 << 1)); } while(0)
#define GPIOC_RST() do{ (RCC->AHBRSTR[0] |= (1 << 2)); (RCC->AHBRSTR[0] &= ~(1 << 2)); } while(0)
#define GPIOD_RST() do{ (RCC->AHBRSTR[0] |= (1 << 3)); (RCC->AHBRSTR[0] &= ~(1 << 3)); } while(0)
#define GPIOE_RST() do{ (RCC->AHBRSTR[0] |= (1 << 4)); (RCC->AHBRSTR[0] &= ~(1 << 4)); } while(0)
#define GPIOF_RST() do{ (RCC->AHBRSTR[0] |= (1 << 5)); (RCC->AHBRSTR[0] &= ~(1 << 5)); } while(0)
#define GPIOG_RST() do{ (RCC->AHBRSTR[0] |= (1 << 6)); (RCC->AHBRSTR[0] &= ~(1 << 6)); } while(0)
#define GPIOH_RST() do{ (RCC->AHBRSTR[0] |= (1 << 7)); (RCC->AHBRSTR[0] &= ~(1 << 7)); } while(0)

/*
 *  Reset SPI Peripheral
 */
#define SPI1_RST() do{ (RCC->APBRSTR[1] |= (1 << 12)); (RCC->APBRSTR[0] &= ~(1 << 12)); } while(0)
#define SPI2_RST() do{ (RCC->APBRSTR[1] |= (1 << 14)); (RCC->APBRSTR[0] &= ~(1 << 12)); } while(0)
#define SPI3_RST() do{ (RCC->APBRSTR[1] |= (1 << 15)); (RCC->APBRSTR[0] &= ~(1 << 12)); } while(0)
#define SPI4_RST() do{ (RCC->APBRSTR[1] |= (1 << 13)); (RCC->APBRSTR[0] &= ~(1 << 12)); } while(0)

/*
 *  Reset USART Peripheral
 */
#define USART1_RST() do{ (RCC->APBRSTR[1] |= (1 << 4)); (RCC->APBRSTR[0] &= ~(1 << 4)); } while(0)
#define USART2_RST() do{ (RCC->APBRSTR[0] |= (1 << 17)); (RCC->APBRSTR[0] &= ~(1 << 17)); } while(0)
#define USART3_RST() do{ (RCC->APBRSTR[0] |= (1 << 18)); (RCC->APBRSTR[0] &= ~(1 << 18)); } while(0)
#define UART4_RST() do{ (RCC->APBRSTR[0] |= (1 << 19)); (RCC->APBRSTR[0] &= ~(1 << 19)); } while(0)
#define UART5_RST() do{ (RCC->APBRSTR[0] |= (1 << 20)); (RCC->APBRSTR[0] &= ~(1 << 20)); } while(0)
#define USART6_RST() do{ (RCC->APBRSTR[0] |= (1 << 5)); (RCC->APBRSTR[0] &= ~(1 << 5)); } while(0)


/*******************************************************************************************************
* Generic Macro
*******************************************************************************************************/
//Some generic Macro
#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define SUCCESS			1
#define FAILURE			0
#define FULL			1
#define EMPTY			0



/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15


/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9


/****************************************************************************************************************************************
 * 	NAME: Driver Include Part
 * 	NOTE: Need this part so that in main file you don't need to add every driver you want to use just include stm32f446re and the
 * 		  rest will come automatically
 ****************************************************************************************************************************************/

#include "gpio_driver.h"
#include "uart_driver.h"
#include "rcc_driver.h"
#include "spi_driver.h"


#endif /* INC_STM32F446RE_H_ */

