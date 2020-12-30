#ifndef UART_DRIVER
#define UART_DRIVER

#include "stm32f446re.h"

class USART{
    private:
        uint8_t mode;
        uint8_t NoOfStopBit;
        uint8_t wordlength;
        uint8_t parity;                 /* En or Dis */
        uint8_t hwflowcontrol;
        uint32_t baudrate;

        /* Interrupt Related Parameter */
        uint8_t irq_number;

        /* Tx Interrupt Parameter */
        uint8_t tx_len;
        uint8_t *pTxbuf;
        uint8_t TxBusyState;

        /* Rx Interrupt Parameter */
        uint8_t rx_len;
        uint8_t *pRxbuf;
        uint8_t RxBusyState;

        USART_RegDef_t *pUSART;
        uint8_t GetFlagStatus(uint8_t _flagname);
        uint8_t ClearFlag(uint8_t _flagname);
        void USART_SetBaudRate();

    public:
		USART(USART_RegDef_t *pUSARTx);

		void CfgSetup(uint8_t _mode, uint8_t _NoOfStopBit,
					 uint8_t _wordlength, uint8_t _parity, uint8_t _hwflowcontrol, uint32_t _baudrate);

		void Init();
		void Deinit();

		void PeriClkCtrl(uint8_t EnorDi);                                 /* Enable Peripheral Clk Uart */
		void enable();                                      /* Enable Uart */
		void disable();                                     /* Disable Uart */
		void INTEnable();                                   /* Enable Interrupt */
		void INTDisable();                                  /* Disable Interrupt */
		void INTPriorityConfig(uint8_t priority);           /* Set Interrupt Priority */

		/* Trasmission API */
		void SendData(uint8_t *pTxBuffer, uint32_t Len);
		void ReceiveData(uint8_t *pRxBuffer, uint32_t Len);
		uint8_t SendDataIT(uint8_t *pTxBuffer, uint32_t Len);
		uint8_t ReceiveDataIT(uint8_t *pRxBuffer, uint32_t Len);
};

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
 * Application states
 */
#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
#define USART_READY 0

#endif
