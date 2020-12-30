/*
 * stm32f446re_gpio_driver.h
 *
 *  Created on: Dec 3, 2020
 *      Author: bsorr
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include "stm32f446re.h"

typedef struct
{
	uint8_t pinnumber;
	uint8_t mode;
	uint8_t otype;
	uint8_t ospeed;
	uint8_t pupd;
	uint8_t altmode;
}GPIO_PinConf_t;

/*
 * GPIO structure is use to handle the task from the user (eg. set pin mode, speed, output type)
 *
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOX;
	GPIO_PinConf_t GPIO_PinConf;

}GPIO_Handle_t;

class GPIO{
    private:
        uint8_t pinnumber;
        uint8_t mode;
        uint8_t otype;
        uint8_t ospeed;
        uint8_t pupd;
        uint8_t altmode;
        GPIO_RegDef_t *pGPIOX;
    public:

        // Constructor Use to set all private value to default 
        GPIO(GPIO_RegDef_t *pGPIO);

        /*
        * GPIO Config as Altenative Function 
        */
        void GpioSetupCfg (uint8_t _pinnum, uint8_t _mode, uint8_t _otype,
                                 uint8_t _ospeed, uint8_t _pupd, uint8_t _altmode);
        /*
        * GPIO Config as Output
        */
        void GpioSetupCfg (uint8_t _pinnum, uint8_t _mode, uint8_t _otype, 
                           uint8_t _ospeed, uint8_t _pupd);

        /*
        * GPIO Config as Input Or Interrupt 
        */
        void GpioSetupCfg (uint8_t _pinnum, uint8_t _mode, uint8_t _pupd);

        /*
        * Peripheral CLK Control
        */
        void PeriClkCtrl(uint8_t EnorDi);

        /*
        * Init and Deinit GPIO
        */
        void Init();
        void DeInit();

        /*
        * Read API
        */
        uint8_t ReadPin(uint8_t pinnumber);
        uint16_t ReadPort();

        /*
        * Write API
        */
        void WritePin(uint8_t pinnumber, uint8_t data);
        void WritePort(uint16_t data);
        void ToggleOutputPin(uint8_t pinnumber);

        // IRQ Config and ISR Handling
        uint8_t IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);

        void IRQHandling(uint8_t pinnumber);
    
};

/*
 * @GPIO MODE
 */
#define GPIO_MODE_IN			0x0
#define GPIO_MODE_OUT			0x1
#define GPIO_MODE_ALTFN			0x2		/*!< Alternative Function>!*/
#define GPIO_MODE_ANALOG		0x3
#define GPIO_MODE_INT_FT		0X4		/*!< Interrupt Falling Edge Trigger >!*/
#define GPIO_MODE_INT_RT		0X5		/*!< Interrupt Rising Edge Trigger >!*/
#define GPIO_MODE_INT_FRT		0X6		/*!< Interrupt Rising and Falling Edge Trigger >!*/

/*
 * GPIO PIN NUMBER CHOICE
 */
#define GPIO_PIN0			0x0
#define GPIO_PIN1			0x1
#define GPIO_PIN2			0x2
#define GPIO_PIN3			0x3
#define GPIO_PIN4			0x4
#define GPIO_PIN5			0x5
#define GPIO_PIN6			0x6
#define GPIO_PIN7			0x7
#define GPIO_PIN8			0x8
#define GPIO_PIN9			0x9
#define GPIO_PIN10			0xa
#define GPIO_PIN11			0xb
#define GPIO_PIN12			0xc
#define GPIO_PIN13			0xd
#define GPIO_PIN14			0xe
#define GPIO_PIN15			0xf


/*
 * GPIO OUTPUT TYPE CHOICE
 */
#define GPIO_OT_PP			0x0			/*!< Push-Pull Configuration >!*/
#define GPIO_OT_OD			0x1			/*!< Open Drain Configuration >!*/

/*
 * GPIO SPEED CHOICE
 */
#define GPIO_ST_LS			0x0			/*!< Low Speed >!*/
#define GPIO_ST_MS			0x1			/*!< Medium Speed >!*/
#define GPIO_ST_FS			0x2			/*!< Fast Speed >!*/
#define GPIO_ST_HS			0x3			/*!< High Speed >!*/

/*
 * GPIO PULL UP/DOWN  TYPE
 */
#define GPIO_PUPD_NPUPD			0x0			/*!< No Pull Up/Down >!*/
#define GPIO_PUPD_PU			0x1			/*!< Pull Up >!*/
#define GPIO_PUPD_PD			0x2			/*!< Pull Down >!*/

/*
 * GPIO ALTENATIVE FUNCTION CHOICE
 */
#define GPIO_AF_0			0x0			/*!< Alternative Function 0 >!*/
#define GPIO_AF_1			0x1			/*!< Alternative Function 1  >!*/
#define GPIO_AF_2			0x2			/*!< Alternative Function 2  >!*/
#define GPIO_AF_3			0x3			/*!< Alternative Function 3  >!*/
#define GPIO_AF_4			0x4			/*!< Alternative Function 4  >!*/
#define GPIO_AF_5			0x5			/*!< Alternative Function 5  >!*/
#define GPIO_AF_6			0x6			/*!< Alternative Function 6  >!*/
#define GPIO_AF_7			0x7			/*!< Alternative Function 7  >!*/
#define GPIO_AF_8			0x8			/*!< Alternative Function 8  >!*/
#define GPIO_AF_9			0x9			/*!< Alternative Function 9  >!*/
#define GPIO_AF_10			0xA			/*!< Alternative Function 10  >!*/
#define GPIO_AF_11			0xB			/*!< Alternative Function 11  >!*/
#define GPIO_AF_12			0xC			/*!< Alternative Function 12  >!*/
#define GPIO_AF_13			0xD			/*!< Alternative Function 13  >!*/
#define GPIO_AF_14			0xE			/*!< Alternative Function 14  >!*/
#define GPIO_AF_15			0xF			/*!< Alternative Function 15  >!*/

#endif /* INC_GPIO_DRIVER_H_ */
