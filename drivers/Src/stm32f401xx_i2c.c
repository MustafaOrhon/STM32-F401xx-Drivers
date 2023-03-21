/*
 * stm32f401xx_i2c.c
 *
 *  Created on: 14 Mar 2023
 *      Author: Mustafa
 */
#include "stm32f401xx_i2c.h"
#include "stm32f401xx_rcc.h"
static void I2C_GenerateStart(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddress);
static void I2C_GenerateStop(I2C_RegDef_t *pI2Cx);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_ExecuteReceiveAddress(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddress);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStart(I2C_RegDef_t *pI2Cx)
 {

  pI2Cx->CR1 |= (1<<I2C_CR1_START); // Generates the start bit

 }

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddress)
{
	SlaveAddress = SlaveAddress << 1; //Make space for write
	SlaveAddress &=  ~(1);
	pI2Cx->DR = SlaveAddress;

}
static void I2C_ExecuteReceiveAddress(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddress)
{
	SlaveAddress = SlaveAddress << 1; //Make space for write
	SlaveAddress |= (1<<0);
	pI2Cx->DR = SlaveAddress;

}
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyread;
   // Check For Device Mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//Device is Master
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				if(pI2CHandle->RxSize == 1)
				{
					pI2CHandle->pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK); // Disable Acking
					//Clear ADDR
					dummyread = pI2CHandle->pI2Cx->SR1;
					dummyread = pI2CHandle->pI2Cx->SR2;
					(void)dummyread;
				}

			}
		else
	           	{

					//Clear ADDR
					dummyread = pI2CHandle->pI2Cx->SR1;
					dummyread = pI2CHandle->pI2Cx->SR2;
					(void)dummyread;

		}

	}
	else
	{   //Device is Slave Mode
		//Clear ADDR
		dummyread = pI2CHandle->pI2Cx->SR1;
		dummyread = pI2CHandle->pI2Cx->SR2;
		(void)dummyread;

	}
}
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
static void I2C_GenerateStop(I2C_RegDef_t *pI2Cx)
{

	pI2Cx->CR1 |= (1<<I2C_CR1_STOP); // Generates the Stop bit

}
void I2C_CloseRecieveData(I2C_Handle_t *pI2CHandle)
{
	//Disable the ITBUFEN Control BIT
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	//Disable the ITEVTEN Control BIT
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxSize = 0;
    pI2CHandle->RxLen = 0;

    if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
    {
    pI2CHandle->pI2Cx->CR1 |= (1<<I2C_CR1_ACK);
    }



}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Disable the ITBUFEN Control BIT
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	//Disable the ITEVTEN Control BIT
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen = 0;

}
/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given I2C port
 *
 * @param[in]         - base address of the I2C peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	   if(EnorDi == ENABLE)
	   {
		   if(pI2Cx == I2C1_BASEADDR)
		   {
			   I2C1_PCLK_EN();
		   }
		   else if(pI2Cx == I2C2_BASEADDR)
		   {
			   I2C2_PCLK_EN();
		   }
		   else if(pI2Cx == I2C3_BASEADDR)
		   {
			   I2C3_PCLK_EN();
		   }
	   }
	   else
	   {
		   if(pI2Cx == I2C1_BASEADDR)
		   {
			   I2C1_PCLK_DI();
		   }
		   else if(pI2Cx == I2C2_BASEADDR)
		   {
			   I2C2_PCLK_DI();
		   }
		   else if(pI2Cx == I2C3_BASEADDR)
		   {
			   I2C3_PCLK_DI();
		   }
	   }
}





/*********************************************************************
 * @Fun      		  - I2C_Init
 *
 * @brief             - This function initialize the selected I2C port to configured value
 *
 * @param[in]         - Takes the structure of I2C Handle
 *
 *
 * @Note              -
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{

  I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

  // CONFIGURE THEE I2C_CR1 Reg'ster

  uint32_t tempreg =  0;
	uint8_t trise;
  // Ack control bit
  tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl<<I2C_CR1_ACK;
  pI2CHandle->pI2Cx->CR1 |=1;
  pI2CHandle->pI2Cx->CR1 |= tempreg;

  //Configure the Freq Field of CR2
  tempreg =0;
  tempreg |= RCC_GetPCLK1Value() / 1000000U ;
  pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);
  //Configure the Device Own Address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;
  //CCR Calculations
	uint16_t ccr_value=0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCLSpeed_SM)
	{
		//Mode is standard
		ccr_value = ((RCC_GetPCLK1Value()/2)/pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |=ccr_value & 0xFFF;


	}else
	{

		  //Mode is fast mode
		tempreg |= (1<<I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = ((RCC_GetPCLK1Value()/3)/pI2CHandle->I2C_Config.I2C_SCLSpeed);

		}
		else
		{
			ccr_value = ((RCC_GetPCLK1Value()/25)/pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= ccr_value & 0xFFF;
	}
   pI2CHandle->pI2Cx->CCR = tempreg;

   //Trise Config
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCLSpeed_SM)
	{
		//Mode is standard

		trise= (RCC_GetPCLK1Value()/1000000U) + 1;


	}else
	{


		trise= ((RCC_GetPCLK1Value()*300) /1000000U) + 1;

	}

    pI2CHandle->pI2Cx->TRISE = (trise & 0x3F);





}
/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             - This function deinitilize the selected I2C port to reset values
 *
 * @param[in]         - Takes the base address of the I2C peripheral
 *
 *
 * @Note              -
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

	   if(pI2Cx == I2C1_BASEADDR)
	   {
		   I2C1_REG_RESET();
	   }
	   else if(pI2Cx == I2C2_BASEADDR)
	   {
		   I2C2_REG_RESET();
	   }
	   else if(pI2Cx == I2C3_BASEADDR)
	   {
		   I2C3_REG_RESET();
	   }


}
/*********************************************************************
 * @fn      		  - I2C_MasterSendData
 *
 * @brief             - This function sends data to desired slave address
 *
 * @param[in]         - Takes the base address of the I2C Handle Structure
 *
 * @param[in]         - Takes the pointer to the data address
 *
 *@param[in]         - Takes the lenght of the Tx Buffer

 *@param[in]         - Takes the address of the slave that will receive data
 *
 * @Note              -
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t Len,uint8_t SlaveAddr)
{
	//1.Generate the START Condition
	I2C_GenerateStart(pI2CHandle->pI2Cx);
   //2.Confirm that Generation is completed by checking the SB Flag in the SR1
	//Until SB is cleared SCL will be streched (Pulled Low)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));
	//3. Send the address of the slave with r/w bit set to w(0)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,SlaveAddr);
    //4.Confirm that address phase is completed by checking ADDR Flag in SR1 Register
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR) );
    //5.Clear the ADDR flag according to its software sequence
	I2C_ClearADDRFlag(pI2CHandle);
	// Send Data Until Len Becomes
	while(Len > 0)
		{
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) ); //Wait till TXE is set
			pI2CHandle->pI2Cx->DR = *pTxBuffer;
			pTxBuffer++;
			Len--;
		}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
		//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
		//   when BTF=1 SCL will be stretched (pulled to LOW)

		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF)){};


		//8. Generate STOP condition and master need not to wait for the completion of stop condition.
		//   Note: generating STOP, automatically clears the BTF
		I2C_GenerateStop(pI2CHandle->pI2Cx);


}
/*********************************************************************
 * @fn      		  - I2C_MasterReceiveData
 *
 * @brief             - This function sends data to desired slave address
 *
 * @param[in]         - Takes the base address of the I2C Handle Structure
 *
 * @param[in]         - Takes the pointer to the data address
 *
 *@param[in]         - Takes the length of the Rx Buffer

 *@param[in]         - Takes the address of the slave that will receive data
 *
 * @Note              -
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer,uint32_t Len,uint8_t SlaveAddr)
{
	//1.Generate the START Condition
	I2C_GenerateStart(pI2CHandle->pI2Cx);
   //2.Confirm that Generation is completed by checking the SB Flag in the SR1
	//Until SB is cleared SCL will be streched (Pulled Low)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));
	//3. Send the address of the slave with r/w bit set to w(0)
	I2C_ExecuteReceiveAddress(pI2CHandle->pI2Cx,SlaveAddr);
    //4.Confirm that address phase is completed by checking ADDR Flag in SR1 Register
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//Procedure for Reading only 1 byte
	if(Len == 1)
	{
		//Disable Acking
		pI2CHandle->pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK);

		//Clear ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle);
		//Wait until RXNE Becomes zero
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));
		//Generate STOP
		I2C_GenerateStop(pI2CHandle->pI2Cx);
		//Read Data TO Buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}
	//Procedure for Reading more than  1 byte
	if(Len > 1)
	{
		//Clear ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle);
		for(uint32_t i=Len ; i>0 ;i--)
		{
			//Wait until RXNE Becomes zero
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));
			if(i==2)
			{
				//Disable Acking
				pI2CHandle->pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK);
				//Generate STOP
				I2C_GenerateStop(pI2CHandle->pI2Cx);

			}

			//Read Data TO Buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			 pRxBuffer++;

		}

	}
    if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
    {
	pI2CHandle->pI2Cx->CR1 |= (1<<I2C_CR1_ACK);
    }



}


/*
 * * IRQ config and Handling
 */
void I2C_IRQITConfig(uint8_t IRQNumber,uint8_t EnorDi)
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
    //Find IPR Register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
   *(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount );

}
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1<<I2C_CR1_PE);

	}
	else
	{
		pI2Cx->CR1 &= ~(1<<I2C_CR1_PE);
	}

}
/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Complete the below code . Also include the function prototype in header file

 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pTxBuffer = pTxBuffer;
			pI2CHandle->TxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			//Implement code to Generate START Condition
			 I2C_GenerateStart(pI2CHandle->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);


		}

		return busystate;

}
/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
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
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
	      uint8_t busystate = pI2CHandle->TxRxState;

		  if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			  pI2CHandle->pRxBuffer = pRxBuffer;
			  pI2CHandle->RxLen =Len;
			  pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
			  pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
			  pI2CHandle->DevAddr = SlaveAddr;
			  pI2CHandle->Sr = Sr;

			   //Implement code to Generate START Condition
			    I2C_GenerateStart(pI2CHandle->pI2Cx);

				//Implement the code to enable ITBUFEN Control Bit
				pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

				//Implement the code to enable ITEVFEN Control Bit
				pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

				//Implement the code to enable ITERREN Control Bit
				pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;

}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx,uint8_t data)
{
   pI2Cx->DR = data;

}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{

	return (uint8_t) pI2Cx->DR;
}




static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	    		 {

	    	     	pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

	    			pI2CHandle->TxLen--;

	    			pI2CHandle->pTxBuffer++;
	    		 }
}
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxSize == 1)
		    	      {
	                      *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
	                       pI2CHandle->RxLen--;


		    	      }

		    	   if(pI2CHandle->RxSize > 1)
		    	  	  {
	                    if(pI2CHandle->RxLen == 2)
	                    {
	                    	pI2CHandle->pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK); // Disable Acking

	                    }
	                    *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
	                     pI2CHandle->pRxBuffer++;
	                     pI2CHandle->RxLen--;

		    	  	  }
		    	   if(pI2CHandle->RxLen == 0)
		    	   {
	   		    	//Close transmission
	   			    I2C_GenerateStop(pI2CHandle->pI2Cx);
	   			    // Reset All the members elements of the handle structure
	   			    I2C_CloseRecieveData(pI2CHandle);
	   			    // Notify The App. About Transmission Complete


		    	   }
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
         uint32_t temp1,temp2,temp3;

         temp1 |= pI2CHandle->pI2Cx->CR2 & (1<<I2C_CR2_ITEVTEN);
         temp2 |= pI2CHandle->pI2Cx->CR2 & (1<<I2C_CR2_ITBUFEN);

         temp3 |= pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_SB);
		//1. Handle For interrupt generated by SB event
		//	Note : SB flag is only applicable in Master mode
	    if(temp1 && temp3)
	    {

	    	// The interrupt generated because of SB event (Only in Master Mod)
	    	// In this block lets execute the address phase
	    	if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
	    	{
	    		I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);

	    	}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
	    	{
	    		I2C_ExecuteReceiveAddress(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);

	    	}

	    }



	    temp3 |= pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_ADDR);
		//2. Handle For interrupt generated by ADDR event
		//Note : When master mode : Address is sent
		//		 When Slave mode   : Address matched with own address
	    if(temp1 && temp3)
	    {

	    	// ADDR FLAG IS SET
	    	I2C_ClearADDRFlag(pI2CHandle);
	    }

	    temp3 |= pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_BTF);
		//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	    if(temp1 && temp3)
	    {

	    	// BTF Flag is set
	    	if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
	    	{
	    		// Make Sure TXE is also set
	    		if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
	    		{
	    			if(pI2CHandle->TxLen == 0)
	    			{
	    		    	//Close transmission
	    			    I2C_GenerateStop(pI2CHandle->pI2Cx);
	    			    // Reset All the members elements of the handle structure
	    			    I2C_CloseSendData(pI2CHandle);
	    			    // Notify The App. About Transmission Complete
	    			}

	    		}

	    	}
	    	else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
	    	{

	    		;
	    	}

	    }
	    temp3 |= pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_STOPF);
		//4. Handle For interrupt generated by STOPF event
		// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	    if(temp1 && temp3)
	   	    {

	         	// STOPF FLAG IS SET
	    	    // Clear STOPF (Read SR1 and Write to CR1)
	    	    pI2CHandle->pI2Cx->CR1 |= 0x0000;
	   	    }


	    temp3 |= pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_TXE);
		//5. Handle For interrupt generated by TXE event
	    if(temp1 && temp3 && temp2)
	   	    {

	    	  if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	    	 {
	   	    	// TXE FLAG IS SET
	    	    if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
	    	   {
	    		I2C_MasterHandleTXEInterrupt(pI2CHandle);

	    	    }
	    	  }
	    	  else
	    	  {
	    		   // Slave mode
	    		  if(pI2CHandle->pI2Cx->SR2 &( 1 << I2C_SR2_TRA))
	    		  {
	    		    I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
	    		  }

	    	  }
	   	    }
	    temp3 |= pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_RXNE);
		//6. Handle For interrupt generated by RXNE event
	    if(temp1 && temp3 && temp2)
	   	   {

	    	     if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	    	        {
	    	   // RXNE FLAG IS SET
	    	            if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
	    	              {

                             I2C_MasterHandleRXNEInterrupt(pI2CHandle);

	    	               }
	                  }
	    	     else
	    	     {
		    		   // Slave mode
		    		  if(!(pI2CHandle->pI2Cx->SR2 &( 1 << I2C_SR2_TRA)))
		    		  {
		    		    I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
		    		  }

	    	     }

	          }

}


void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}



