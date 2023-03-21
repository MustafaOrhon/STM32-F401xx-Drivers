/*
 * stm32f401xx_i2c.h
 *
 *  Created on: 14 Mar 2023
 *      Author: Mustafa
 */


#ifndef INC_STM32F401XX_I2C_H_
#define INC_STM32F401XX_I2C_H_
#include "stm32f401xx.h"
/*
 *  Configuration structure for I2Cx Peripheral
 */
typedef struct
{

	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint8_t I2C_FMDutyCycle;


}I2C_Config_t;
/*
 * Handle Structure for I2Cx Peripheral
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx;      /*!\<This holds the base address of I2Cx(x:1,2,3,4) peripheral>/!*/
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer;       /*!\<To store the app. tx buffer address>/!*/
	uint8_t *pRxBuffer;       /*!\<To store the app. rx buffer address>/!*/
	uint32_t TxLen;           /*!\<To store the app. tx length>/!*/
	uint32_t RxLen;           /*!\<To store the app. rx length>/!*/
	uint8_t TxRxState;        /*!\<To store the app. TxRx state>/!*/
	uint8_t DevAddr;          /*!\<To store the slave device addres state>/!*/
	uint32_t RxSize;          /*!\<To store the app. rx size state>/!*/
	uint8_t  Sr;              /*!\<To store the repeated start value>/!*/
}
I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCLSpeed_SM     100000
#define I2C_SCLSpeed_FM     400000
/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE      1
#define I2C_ACK_DISABLE     0
/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2       0
#define I2C_FM_DUTY_16_9    1
/*
 * I2C application states
 */
#define I2C_READY      0
#define I2C_BUSY_IN_RX 1
#define I2C_BUSY_IN_TX 2
/******************************************************************************************************
 *                               APIs supported by this driver
 *                      For more information about the APIs check the function definitions
 ******************************************************************************************************/
/*
 * *Peripheral Clock Setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);

/*
 * *Init and Deinit
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * *Data Send and Receive
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t Len,uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer,uint32_t Len,uint8_t SlaveAddr);
/*
 * *Data Send and Receive In Interrupt
 */

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
/*
 * * Slave Data Send And Recive
 */

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);
/*
 * * IRQ config and Handling
 */
void I2C_IRQITConfig(uint8_t IRQNumber,uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_CloseRecieveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
/*
 *  OTHER PERIPHERAL CONTROLS
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t flagname);

/*
* SPI RELATED FLAG STATUS
*/
#define I2C_FLAG_TXE (1<<I2C_SR1_TXE)
#define I2C_FLAG_RXNE (1<<I2C_SR1_RXNE)
#define I2C_FLAG_SB (1<<I2C_SR1_SB)
#define I2C_FLAG_OVR  		( 1 << I2C_SR1_OVR)
#define I2C_FLAG_AF   		( 1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO 		( 1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR 		( 1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF 		( 1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 		( 1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF  		( 1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR 		( 1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 	( 1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET

/*
 * I2C application event macros
 */
#define I2C_EV_TX_CMPLT  0
#define I2C_EV_RX_CMPLT  1
#define I2C_EV_STOP      2
#define I2C_ERROR_BERR  3
#define I2C_ERROR_ARLO  4
#define I2C_ERROR_AF    5
#define I2C_ERROR_OVR   6
#define I2C_ERROR_TIMEOUT 7
#define I2C_EV_DATA_REQ   8
#define I2C_EV_DATA_RCV   9
/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t );




#endif /* INC_STM32F401XX_I2C_H_ */
