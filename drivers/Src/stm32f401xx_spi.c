/*
 * stm32f401xx_spi.c
 *
 *  Created on: 11 Mar 2023
 *      Author: Mustafa ORHON
 */
#include <stm32f401xx_spi.h>

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	   if(EnorDi == ENABLE)
	   {
		   if(pSPIx == SPI1_BASEADDR)
		   {
			   SPI1_PCLK_EN();
		   }
		   else if(pSPIx == SPI2_BASEADDR)
		   {
			   SPI2_PCLK_EN();
		   }
		   else if(pSPIx == SPI3_BASEADDR)
		   {
			   SPI3_PCLK_EN();
		   }
		   else if(pSPIx == SPI4_BASEADDR)
		   {
			   SPI4_PCLK_EN();
		   }


	   }
	   else
	   {
		   if(pSPIx == SPI1_BASEADDR)
		   {
			   SPI1_PCLK_DI();
		   }
		   else if(pSPIx == SPI2_BASEADDR)
		   {
			   SPI2_PCLK_DI();
		   }
		   else if(pSPIx == SPI3_BASEADDR)
		   {
			   SPI3_PCLK_DI();
		   }
		   else if(pSPIx == SPI4_BASEADDR)
		   {
			   SPI4_PCLK_DI();
		   }


	   }


}

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function initilize the selected SPI port to configured value
 *
 * @param[in]         - Takes the structure of SPI Handle
 *
 *
 * @Note              -
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

  SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);

  // CONFIGURE THEE SPI_CR1 Reg'ster

  uint32_t tempreg =  0;
  // Device Mode
  tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode <<2;
  // Bus Config
  if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) // Clear BIDI BIT
  {
     tempreg &= ~(1<<15);

  }
  else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HF) //ENABLE BIDI
  {
	  tempreg |= (1<<15);
  }
  else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HF) //Clear BIDI and RXONLY BIT SET
    {
	  tempreg &= ~(1<<15);

	  tempreg |= (1<<10);
    }
  //SClock Speed
      tempreg |= (pSPIHandle->SPIConfig.SPI_SCLKSpeed <<3);

  // DFF CONFIG
      tempreg |= (pSPIHandle->SPIConfig.SPI_DFF <<11);
  // CPOL CONFIG

      tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL <<1);
  // CPHA CONFIG

      tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA <<0);
      tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

      pSPIHandle->pSPIx->CR1 = tempreg;



}
/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function deinitilize the selected SPI port to reset values
 *
 * @param[in]         - Takes the base address of the  SPI peripheral
 *
 *
 * @Note              -
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

	   if(pSPIx == SPI1_BASEADDR)
	   {
		   SPI1_REG_RESET();
	   }
	   else if(pSPIx == SPI2_BASEADDR)
	   {
		   SPI2_REG_RESET();
	   }
	   else if(pSPIx == SPI3_BASEADDR)
	   {
		   SPI3_REG_RESET();
	   }
	   else if(pSPIx == SPI4_BASEADDR)
	   {
		   SPI4_REG_RESET();
	   }


}
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName)
{
     if(pSPIx->SR & FlagName)
     {
    	 return FLAG_SET;
     }

	return FLAG_RESET;
}

/*
 * *Data Send and Recieve
 */
/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - Send the data from selected SPI peripherals pin number
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - pointer to the TxBuffer
 * @param[in]         - Length of the TxBuffer
 *
 * @Note              -
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Length)
{

	while(Length >0)
	{
		//.Wait until  TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		// Check DFF BIT in CR1
		if((pSPIx->CR1 & (1<< SPI_CR1_DFF)))
		{
			//16 Bit
			pSPIx->DR =  *((uint16_t*)pTxBuffer);
			Length--;
			Length--;
			(uint16_t*)pTxBuffer++;

		}
		else
		{  //( bit
			pSPIx->DR =  *pTxBuffer;
			Length--;
			pTxBuffer++;
		}

	}


}
/*********************************************************************
 * @fn      		  - SPI_RecieveData
 *
 * @brief             - Send the data from selected SPI peripherals pin number
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - pointer to the RxBuffer
 * @param[in]         - Length of the RxBuffer
 *
 * @Note              -
 */
void SPI_RecieveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Length)
{

	while(Length >0)
	{
		//.Wait until  RXNE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET);

		// Check DFF BIT in CR1
		if((pSPIx->CR1 & (1<< SPI_CR1_DFF)))
		{
			//16 Bit
			*((uint16_t*)pRxBuffer)=pSPIx->DR;
			Length--;
			Length--;
			(uint16_t*)pRxBuffer++;

		}
		else
		{  //( bit
			  *pRxBuffer=pSPIx->DR;
			Length--;
			pRxBuffer++;
		}

	}


}


/*
 * * IRQ config and Handling
 */
void SPI_IRQITConfig(uint8_t IRQNumber,uint8_t EnorDi)
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
    //Find IPR Register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
   *(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount );

}

void SPI_TXE_INTERRUPT_Handle(SPI_Handle_t *pSPIHandle)
{
	// Check DFF BIT in CR1
	if((pSPIHandle->pSPIx->CR1 & (1<< SPI_CR1_DFF)))
	{
		//16 Bit
		pSPIHandle->pSPIx->DR =  *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
		(uint16_t*)pSPIHandle->pTxBuffer++;

	}
	else
	{  // 8 bit DFF
		pSPIHandle->pSPIx->DR =  *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
     if(!pSPIHandle->TxLen)
     {
    	 //TXlen is zero,so close the spi transmission and inform application
    	 SPI_CloseTransmission(pSPIHandle);
    	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);

     }

}

void SPI_RXNE_INTERRUPT_Handle(SPI_Handle_t *pSPIHandle)
{
	// Check DFF BIT in CR1
		if(pSPIHandle->pSPIx->CR1 & (1<< 11))
		{
			//16 Bit
			*((uint16_t*)pSPIHandle->pRxBuffer)=(uint16_t)pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen-=2;
			pSPIHandle->pRxBuffer--;
			pSPIHandle->pRxBuffer--;

		}
		else
		{  // 8 bit DFF
	         *pSPIHandle->pRxBuffer=(uint8_t)pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer++;
		}
	     if(!pSPIHandle->RxLen)
	     {
	    	 //TXlen is zero,so close the spi transmission and inform application
	         SPI_CloseReception(pSPIHandle);
	    	 SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);

	     }
}

void SPI_OVR_INTERRUPT_Handle(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp;
	//Clear the OVR flag
	if(pSPIHandle->TxState != SPI_Busy_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;

	}
	(void)temp;
  //Inform application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);



}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
   uint8_t temp1,temp2;

   //First check for TXE
   temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_SR_TXE);
   temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);

   if(temp1 && temp2)
   {
	   //Handle TXE
	   SPI_TXE_INTERRUPT_Handle(pSPIHandle);

   }

   // check for RXNE
   temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_SR_RXNE);
   temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_RXNEIE);
   if(temp1 && temp2)
   {
	   //Handle RXNE
	   SPI_RXNE_INTERRUPT_Handle(pSPIHandle);

   }
   //Check for ovr flag
   temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_SR_OVR);
   temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_ERRIE);
   if(temp1 && temp2)
   {
	   //Handle RXNE
	   SPI_OVR_INTERRUPT_Handle(pSPIHandle);

   }

}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);

	}
	else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}

}
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SSI);

	}
	else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
	}

}
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t Length)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_Busy_IN_TX)
	{
	//1. Save TX buffer address and Length information on Handle Variable
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = Length;

	//2. Mark the SPI state as busy in transmission so that no other code can take over until it is over
	pSPIHandle->TxState = SPI_Busy_IN_TX;

	//3.Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
    pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);

	//4. Data transmission will be handled by the ISR code (will implement later)
    }

   return state;

}
uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t Length)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_Busy_IN_RX)
	{
	//1. Save RX buffer address and Length information on Handle Variable
	pSPIHandle->pRxBuffer = pRxBuffer;
	pSPIHandle->RxLen = Length;

	//2. Mark the SPI state as busy in transmission so that no other code can take over until it is over
	pSPIHandle->RxState = SPI_Busy_IN_RX;

	//3.Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
    pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);

	//4. Data transmission will be handled by the ISR code (will implement later)
    }

   return state;

}
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
   uint8_t temp;
   temp= pSPIx->DR;
   temp= pSPIx->SR;
   (void)temp;
}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	//TXlen is zero,so close the spi transmission and inform application
		    	 pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
		    	 pSPIHandle->pTxBuffer = NULL;
		    	 pSPIHandle->TxLen=0;
		    	 pSPIHandle->TxState = SPI_Ready;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	//TXlen is zero,so close the spi transmission and inform application
		    	 pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
		    	 pSPIHandle->pRxBuffer = NULL;
		    	 pSPIHandle->RxLen=0;
		    	 pSPIHandle->RxState = SPI_Ready;
}
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

}

