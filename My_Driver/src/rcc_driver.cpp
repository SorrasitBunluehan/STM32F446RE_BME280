#include "rcc_driver.h"

uint16_t AHBPRE_LIST[8] = {2,4,8,16,64,128,256,512};
uint16_t APBPRE_LIST[4] = {2,4,8,16};

uint32_t I2C_RCCGetAPB1ClkFREQ(void){
	uint32_t pclk_value;
	uint32_t sysclk,AHBPRE,APBPRE;
	uint8_t sysclksrc,temp;
	sysclksrc = (RCC->CFGR >> 2) & 0x3;

	// Finding sysclk
	if (sysclksrc == 0) {
		sysclk = 16000000;
	}else if (sysclksrc == 1) {
		sysclk = 8000000;
	}else{
		// TODO: sysclk is depend on PLL which we don't use in this case
		return 0;
	}

	//Find AHB Prescalar
	temp = (RCC->CFGR >> 4) & 0xF;
	if (temp < 8) {
		AHBPRE = 1;
	}else {
		AHBPRE = AHBPRE_LIST[temp-8];
	}

	//Find APB1 Prescalar
	temp = (RCC->CFGR >> 10) & 0x7;
	if (temp < 4) {
		APBPRE = 1;
	}else {
		APBPRE = APBPRE_LIST[temp-4];
	}

	pclk_value = (sysclk / AHBPRE) / APBPRE;
	return pclk_value;

}

uint32_t I2C_RCCGetAPB2ClkFREQ(void){
	uint32_t pclk_value;
	uint32_t sysclk,AHBPRE,APB2PRE;
	uint8_t sysclksrc,temp;
	sysclksrc = (RCC->CFGR >> 2) & 0x3;

	// Finding sysclk
	if (sysclksrc == 0) {
		sysclk = 16000000;
	}else if (sysclksrc == 1) {
		sysclk = 8000000;
	}else{
		// TODO: sysclk is depend on PLL which we don't use in this case
		return 0;
	}

	//Find AHB Prescalar
	temp = (RCC->CFGR >> 4) & 0xF;
	if (temp < 8) {
		AHBPRE = 1;
	}else {
		AHBPRE = AHBPRE_LIST[temp-8];
	}

	//Find APB2 Prescalar
	temp = (RCC->CFGR >> 13) & 0x7;
	if (temp < 4) {
        APB2PRE = 1;
	}else {
		APB2PRE = APBPRE_LIST[temp-4];
	}

	pclk_value = (sysclk / AHBPRE) / APB2PRE;
	return pclk_value;
}