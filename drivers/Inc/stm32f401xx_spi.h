/*
 * stm32f401xx_spi.h
 *
 *  Created on: 11 Mar 2023
 *      Author: Mustafa
 */

#ifndef INC_STM32F401XX_SPI_H_
#define INC_STM32F401XX_SPI_H_
#include "stm32f401xx.h"
/*
 *  Configuration structure for SPIx Peripheral
 */
typedef struct
{

	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SCLKSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;


}SPI_Config_t;
/*
 * Handle Structure for SPIx Peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;      /*!\<This holds the base address of SPIx(x:1,2,3,4) peripheral>/!*/
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;   /*!\<To store the app. tx buffer address>/!*/
	uint8_t *pRxBuffer;   /*!\<To store the app. rx buffer address>/!*/
	uint32_t TxLen;       /*!\<To store the app. tx length>/!*/
	uint32_t RxLen;       /*!\<To store the app. rx length>/!*/
	uint8_t TxState;      /*!\<To store the app. tx state>/!*/
	uint8_t RxState;      /*!\<To store the app. tx state>/!*/
}
SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE 0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD             1
#define SPI_BUS_CONFIG_HF             2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 3
/*
 * @SPI_SCLKSpeed
 */

#define SPI_SCLK_SPEED_DIV2     0
#define SPI_SCLK_SPEED_DIV4     1
#define SPI_SCLK_SPEED_DIV8     2
#define SPI_SCLK_SPEED_DIV16    3
#define SPI_SCLK_SPEED_DIV32    4
#define SPI_SCLK_SPEED_DIV64    5
#define SPI_SCLK_SPEED_DIV128   6
#define SPI_SCLK_SPEED_DIV256   7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS  0
#define SPI_DFF_16BITS 1
/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH   1
#define SPI_CPOL_LOW    0
/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH   1
#define SPI_CPHA_LOW    0
/*
 * @SPI_SSM
 */
#define SPI_SSM_EN      1
#define SPI_SSM_DI      0

/*
 * SPI RELATED FLAG STATUS
 */
#define SPI_TXE_FLAG (1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG (1<<SPI_SR_RXNE)
#define SPI_BUSY_FLAG (1<<SPI_SR_BSY)

#define SPI_Ready        0
#define SPI_Busy_IN_RX   1
#define SPI_Busy_IN_TX   2
/*
 * Possibe SPI Application Events
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4
/******************************************************************************************************
 *                      APIs supported by this driver
 *                For more information about the APIs check the function definitions
 ******************************************************************************************************/
/*
 * *Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

/*
 * *Init and Deinit
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * *Data Send and Recieve
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Length);
void SPI_RecieveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Length);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t Length);
uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t Length);
/*
 * * IRQ config and Handling
 */
void SPI_IRQITConfig(uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 *  OTHER PERIPHERAL CONTROLS
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

void SPI_TXE_INTERRUPT_Handle(SPI_Handle_t *pSPIHandle);

void SPI_RXNE_INTERRUPT_Handle(SPI_Handle_t *pSPIHandle);

void SPI_OVR_INTERRUPT_Handle(SPI_Handle_t *pSPIHandle);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName);


#endif /* INC_STM32F401XX_SPI_H_ */
