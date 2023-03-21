/*
 * stm32f401xx_gpio.h
 *
 *  Created on: 5 Mar 2023
 *      Author: Mustafa
 */

#ifndef INC_STM32F401XX_GPIO_H_
#define INC_STM32F401XX_GPIO_H_

#include "stm32f401xx.h"
/**
 * This is a configuration structure for GPIO pin
 */
typedef struct
{

	uint8_t GPIO_PinNumber;             /*!< possible values from @GPIO_PIN_NUMBERS>*/
	uint8_t GPIO_PinMode;               /*!< possible values from @GPIO_PIN_MODES>*/
	uint8_t GPIO_PinSpeed;              /*!< possible values from @GPIO_PIN_SPEED>*/
	uint8_t GPIO_PinPuPdControl;        /*!< possible values from @GPIO_PIN_PUPD>*/
	uint8_t GPIO_PinOPType;             /*!< possible values from @GPIO_PIN_OTYPE>*/
	uint8_t GPIO_PinAFMode;


}GPIO_PinConfig_t;

/**
 * This is a handle structure for GPIO pin
 */
typedef struct
{

	GPIO_RegDef_t *pGPIOx; //Pointer to hold the base address of GPIO port to which pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; // This holds GPIO pin config settings




}GPIO_Handle_t;
/*
 * @GPIO_PIN_NUMBERS
 *
 * GPIO pin numbers
 */
#define GPIO_PIN_0 0
#define GPIO_PIN_1 1
#define GPIO_PIN_2 2
#define GPIO_PIN_3 3
#define GPIO_PIN_4 4
#define GPIO_PIN_5 5
#define GPIO_PIN_6 6
#define GPIO_PIN_7 7
#define GPIO_PIN_8 8
#define GPIO_PIN_9 9
#define GPIO_PIN_10 10
#define GPIO_PIN_11 11
#define GPIO_PIN_12 12
#define GPIO_PIN_13 13
#define GPIO_PIN_14 14
#define GPIO_PIN_15 15

/*
 * @GPIO_PIN_MODES
 * GPIO Possible Modes
 */
#define GPIO_MODE_INPUT 0
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_AF 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT 4
#define GPIO_MODE_IT_RT 5
#define GPIO_MODE_IT_RFT 6
/*
 * @GPIO_PIN_OTYPE
 * GPIO PIN possible output types
 */
#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD 1
/*
 * @GPIO_PIN_SPEED
 * GPIO PIN possible Speed Values
 */
#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_HIGH 2
#define GPIO_SPEED_VERYHIGH 3
/*
 * @GPIO_PIN_PUPD
 * GPIO PIN pullup and pulldown configuration
 */
#define GPIO_NO_PUPD 0
#define GPIO_PIN_PU 1
#define GPIO_PIN_PD 2


/******************************************************************************************************
 *                      APIs supported by this driver
 *                For more information about the APIs check the function definitions
 ******************************************************************************************************/
/*
 * *Periph Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);
/*
 * *Init and Deinit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
/*
 * *Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
/*
 * * IRQ config and Handling
 */
void GPIO_IRQITConfig(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);




#endif /* INC_STM32F401XX_GPIO_H_ */
