/*
 * stm32f401xx_gpio.c
 *
 *  Created on: 5 Mar 2023
 *      Author: Mustafa
 */
#include "stm32f401xx_gpio.h"
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
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{
   if(EnorDi == ENABLE)
   {
	   if(pGPIOx == GPIOA_BASEADDR)
	   {
		   GPIOA_PCLK_EN();
	   }
	   else if(pGPIOx == GPIOB_BASEADDR)
	   {
		   GPIOB_PCLK_EN();
	   }
	   else if(pGPIOx == GPIOC_BASEADDR)
	   {
		   GPIOC_PCLK_EN();
	   }
	   else if(pGPIOx == GPIOD_BASEADDR)
	   {
		   GPIOD_PCLK_EN();
	   }
	   else if(pGPIOx == GPIOE_BASEADDR)
	   {
		   GPIOE_PCLK_EN();
	   }
	   else if(pGPIOx == GPIOH_BASEADDR)
	   {
		   GPIOH_PCLK_EN();
	   }

   }
   else
   {
	   if(pGPIOx == GPIOA_BASEADDR)
	   {
		   GPIOA_PCLK_DI();
	   }
	   else if(pGPIOx == GPIOB_BASEADDR)
	   {
		   GPIOB_PCLK_DI();
	   }
	   else if(pGPIOx == GPIOC_BASEADDR)
	   {
		   GPIOC_PCLK_DI();
	   }
	   else if(pGPIOx == GPIOD_BASEADDR)
	   {
		   GPIOD_PCLK_DI();
	   }
	   else if(pGPIOx == GPIOE_BASEADDR)
	   {
		   GPIOE_PCLK_DI();
	   }
	   else if(pGPIOx == GPIOH_BASEADDR)
	   {
		   GPIOH_PCLK_DI();
	   }


   }

}
/*
 * *Init and Deinit
 */
/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initilize the selected GPIO port to configured value
 *
 * @param[in]         - Takes the structure of GPIO Handle
 *
 *
 * @Note              -
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
   uint32_t temp = 0; //Temp Register
	//1.Configure the mode of gpio pin

   GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
   //setting
   if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<= GPIO_MODE_ANALOG)
	{
		//Non interrupt modes
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting

	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
            EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);

		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	//2. Configure the speed
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//3.Configure the pupd settings
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->PUPDR |= temp;
	//4.Configure the output setting
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->OTYPER |= temp;


	//5.Configure the alternate functionality
	    	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_AF)
	    	{
               //config alt function
	    		uint8_t temp1=0,temp2=0;
	    		temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
	    		temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
	    		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
	    		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAFMode <<(4*temp2));

	    	}


}
/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function deinitilize the selected GPIO port to reset values
 *
 * @param[in]         - Takes the base address of the gpio peripheral
 *
 *
 * @Note              -
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

		   if(pGPIOx == GPIOA_BASEADDR)
		   {
			   GPIOA_REG_RESET();
		   }
		   else if(pGPIOx == GPIOB_BASEADDR)
		   {
			   GPIOB_REG_RESET();
		   }
		   else if(pGPIOx == GPIOC_BASEADDR)
		   {
			   GPIOC_REG_RESET();
		   }
		   else if(pGPIOx == GPIOD_BASEADDR)
		   {
			   GPIOD_REG_RESET();
		   }
		   else if(pGPIOx == GPIOE_BASEADDR)
		   {
			   GPIOE_REG_RESET();
		   }
		   else if(pGPIOx == GPIOH_BASEADDR)
		   {
			   GPIOH_REG_RESET();
		   }

}
/*
 * *Data read and write
 */
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - Reads the input data from selected gpio peripherals pin number
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - pin number which input data will read
 *
 * @return            -   0 or 1
 *
 * @Note              -
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
  uint8_t value;
  value = (uint8_t) ((pGPIOx->IDR>>PinNumber) & 0x00000001);
  return value;
}
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - Reads the input from entire port
 *
 * @param[in]         - base address of the gpio peripheral

 *
 * @return            - return 16 bit of input data
 *
 * @Note              -
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

	  uint16_t value;

	  value = (uint16_t) pGPIOx->IDR;

	  return value;
}
/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - Write data to selected ports pin
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - pin number which output data will written
 * @param[in]         - Value to be write
 *
 *
 * @Note              -
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}
/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - Write 16 bit data to selected port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - 16 bit value to be write
 *
 *
 * @Note              -
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value)
{
	pGPIOx->ODR = value;

}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             -  Toggle the selected pin on chosen port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - pin number which output data will written
 *
 *
 * @Note              -
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR  ^= (1<<PinNumber);
}
/*
 * * IRQ config and Handling
 */
/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void GPIO_IRQITConfig(uint8_t IRQNumber,uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <=31) // Program ISER0 Register
		{
             *NVIC_ISER0 |= ( 1<< IRQNumber );
		}
		else if(IRQNumber>31 && IRQNumber <64 ) //Program ISER1 Register
		{
			*NVIC_ISER1 |= ( 1<< (IRQNumber%32) );
		}
		else if(IRQNumber >=64 && IRQNumber <96) //Program ISER2 Register
		{
			*NVIC_ISER2 |= ( 1<< (IRQNumber%64) );
		}


	}
	else
	{
		if(IRQNumber <=31)
		{
			*NVIC_ICER0 |= ( 1<< IRQNumber );
		}
		else if(IRQNumber>31 && IRQNumber <64 )
		{
			*NVIC_ICER1 |= ( 1<< (IRQNumber%32) );
		}
		else if(IRQNumber >=64 && IRQNumber <96)
		{
			*NVIC_ICER2 |= ( 1<< (IRQNumber%64) );
		}

	}

}
/*********************************************************************
 * @fn      		  - @GPIO_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
     //Find IPR Register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount );


}
/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
    //Clear the exti pr register
	if(EXTI->PR & ( 1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);

	}


}


