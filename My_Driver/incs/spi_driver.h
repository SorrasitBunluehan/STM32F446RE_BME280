
#ifndef SPI_DRIVER_H_
#define SPI_DRIVER_H_

#include "stm32f446re.h"


typedef enum
{
	READY = 0,
	BUSY_IN_TX, 
	BUSY_IN_RX
}SPI_COM_STATUS;

typedef enum
{
	SPI_DEVICE_MODE_SLAVE = 0,
	SPI_DEVICE_MODE_MASTER 
}SPI_DEVICE_MODE;

typedef enum
{
	SPI_BUSCFG_FD = 0,
	SPI_BUSCFG_HD,
	SPI_BUSCFG_SIMPLEX_RXONLY,
	SPI_BUSCFG_SIMPLEX_TXONLY
}SPI_BUSCFG;

/*
 * @SPI_DFF 
 */
typedef enum
{
	SPI_DFF_8 = 0,
	SPI_DFF_16
}SPI_DFF;

/*
 * @SPI_SLV_MNG_MODE 
 */
typedef enum
{
	SPI_SLV_MNG_SW = 0,
	SPI_SLV_MNG_HW
}SPI_SLV_MNG_MODE;

typedef enum
{
	SPI_FRQ_DIV2 = 0,
	SPI_FRQ_DIV4,
	SPI_FRQ_DIV8,
	SPI_FRQ_DIV16,
	SPI_FRQ_DIV32,
	SPI_FRQ_DIV64,
	SPI_FRQ_DIV128,
	SPI_FRQ_DIV256
}SPI_BAUD_RATE_DIV;

typedef enum
{
	SPI_CPOL_LOW = 0,
	SPI_CPOL_HIGH 
}SPI_CPOL_CFG;

typedef enum
{
	SPI_CPHA_FIRST_CAP = 0,
	SPI_CPHA_SECOND_CAP
}SPI_CPHA_CFG;


class HSPI{
	private:
		SPI_RegDef_t 		*SPIx;
		SPI_DEVICE_MODE		DevMode;
		SPI_BUSCFG			BusCfg;
		SPI_DFF				Dff;
		SPI_SLV_MNG_MODE	SlvMngMode;
		SPI_BAUD_RATE_DIV	BaudDiv;
		SPI_CPOL_CFG		Cpol;
		SPI_CPHA_CFG 		Cpha;
		uint8_t 			*pTxBuffer;
		uint8_t 			*pRxBuffer;
		uint32_t 			TxLen;
		uint32_t 			RxLen;
	public:
		HSPI(SPI_RegDef_t *_spix);
		void SetCfg(SPI_DEVICE_MODE _devmode, SPI_BUSCFG _buscfg,
			   SPI_DFF _dff, SPI_SLV_MNG_MODE _slvmgnmode,
			   SPI_BAUD_RATE_DIV _bauddiv,SPI_CPOL_CFG _cpol,
			   SPI_CPHA_CFG _cpha);
		void Init(void);
		void DeInit(void);
		void Enable(void);
		void Disable(void);
		void EnableInterruptMode(void);
		void DisableInterruptMode(void);

		void SetSSI(uint8_t EnorDi);


		/*
		 *	Blocking Transmitting API 
		 */
		void SPI_SendData(uint8_t *pTxbuf, uint32_t Len);
		void SPI_ReadData(uint8_t *pRxbuf, uint32_t Len);

		/*
		 *	Non-Blocking Transmitting API 
		 */
		void SPI_SendData_it(uint8_t *pTxbuf, uint32_t Len);
		void SPI_ReadData_it(uint8_t *pRxbuf, uint32_t Len);

};














































#endif
