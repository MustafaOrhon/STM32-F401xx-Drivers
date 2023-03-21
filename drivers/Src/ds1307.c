/*
 * ds1307.c
 *
 *  Created on: 21 Mar 2023
 *      Author: Mustafa
 */
#include "ds1307.h"
static void ds1307_i2c_pin_config(void);
static void ds1307_i2c_config(void);
static void ds1307_write(uint8_t value,uint8_t reg_addr);
static uint8_t binary_to_bcd(uint8_t value);
static uint8_t bcd_to_binary(uint8_t value);
static uint8_t ds1307_read(uint8_t reg_addr);
I2C_Handle_t g_ds1307I2CHandle;

//Returns 1 : CH = 1  Init Failed
//Returns 0 : CH = 0  Init Success
uint8_t ds1307_init(void)
{
    //1->Initialize the I2C Pıns
	 ds1307_i2c_pin_config();

    //2->Initialize the I2C Peripheral
	 ds1307_i2c_config();

	 //3->ENABLE the i2c periph
	 I2C_PeripheralControl(DS1307_I2C,ENABLE);

	 //4-> Make clock halt =0;
	 ds1307_write(0x00,DS1307_ADDR_SEC);

	 //5-> Read back clock bit
	 uint8_t clock_state = ds1307_read(DS1307_ADDR_SEC);
	 return ((clock_state>>7) & 0x1);

}
void ds1307_set_current_time(RTC_time_t *rtc_time)
{
    uint8_t seconds,hours;
    seconds = binary_to_bcd(rtc_time->seconds);
    seconds &= ~(1<<7); // Making sure CH PIN IS OFF
    ds1307_write(seconds, DS1307_ADDR_SEC);
    ds1307_write(binary_to_bcd(rtc_time->minutes), DS1307_ADDR_MIN);
    hours = binary_to_bcd(rtc_time->hours);
     if(rtc_time->time_format == TIME_FORMAT_24HOURS)
     {
       hours &= ~(1<<6);  //24 Hour Format
     }
     else
     {
    	 hours |= (1<<6);  // 12 HOUR FORMAT
    	 hours = (rtc_time->time_format == TIME_FORMAT_12HRS_PM) ? hours | (1 << 5) : hours & ~(1<<5);
     }
    ds1307_write(hours, DS1307_ADDR_HRS);


}


void ds1307_set_current_date(RTC_date_t *rtc_date)
{
	 ds1307_write(binary_to_bcd(rtc_date->date), DS1307_ADDR_DATE);

	 ds1307_write(binary_to_bcd(rtc_date->day), DS1307_ADDR_DAY);

	 ds1307_write(binary_to_bcd(rtc_date->month), DS1307_ADDR_MONTH);

	 ds1307_write(binary_to_bcd(rtc_date->year), DS1307_ADDR_YEAR);


}
void ds1307_get_current_time(RTC_time_t *rtc_time)
{
     uint8_t seconds,hours;
	 seconds = ds1307_read(DS1307_ADDR_SEC);
	 seconds &= ~(1<<7); // Making sure CH PIN IS OFF
	 rtc_time->seconds = bcd_to_binary(seconds);
	 rtc_time->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MIN));
	 hours = ds1307_read(DS1307_ADDR_HRS);
	 if(hours & (1<<6))
	 {
		  // 12 Hours Format
		 rtc_time->time_format =  !((hours & (1<<5)) == 0); // IF 5th BIT ZERO TIME FORMAT AM OTHERWISE PM
		 hours &= ~(0x3<<5); // CLEAR 5th and 6th BIT
	 }
	 else
	 {
		   // 24 HOURS FORMAT
		 rtc_time->time_format = TIME_FORMAT_24HOURS;

	 }
	 rtc_time->hours = bcd_to_binary(hours);

}
void ds1307_get_current_date(RTC_date_t *rtc_date)
{
   rtc_date->date = bcd_to_binary(ds1307_read(DS1307_ADDR_DATE));

   rtc_date->day = bcd_to_binary(ds1307_read(DS1307_ADDR_DAY));

   rtc_date->month = bcd_to_binary(ds1307_read(DS1307_ADDR_MONTH));

   rtc_date->year = bcd_to_binary(ds1307_read(DS1307_ADDR_YEAR));

}
static void ds1307_i2c_pin_config(void)
{
	GPIO_Handle_t i2c_sda,i2c_scl;
	memset(&i2c_sda,0,sizeof(i2c_sda));
	memset(&i2c_sda,0,sizeof(i2c_scl));

	/*
	 * I2C1_SCL ==> PB6
	 * I2C1_SDA ==> PB7
	 */
	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIO_PinConfig.GPIO_PinAFMode = 4;
	i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
	i2c_sda.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_sda.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;

	GPIO_Init(&i2c_sda);
	i2c_scl.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_scl.GPIO_PinConfig.GPIO_PinAFMode = 4;
	i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;
	i2c_scl.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_scl.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;

	GPIO_Init(&i2c_scl);


}
static void ds1307_i2c_config(void)
{
	g_ds1307I2CHandle.pI2Cx = DS1307_I2C;
	g_ds1307I2CHandle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	g_ds1307I2CHandle.I2C_Config.I2C_SCLSpeed = I2C_SCLSpeed_SM;
	I2C_Init(&g_ds1307I2CHandle);


}
static void ds1307_write(uint8_t value,uint8_t reg_addr)
{
	uint8_t tx[2];
	tx[0] = reg_addr;
	tx[1] = value;

	I2C_MasterSendData(&g_ds1307I2CHandle,tx,2, DS1307_I2C_ADDRESS);

}
static uint8_t ds1307_read(uint8_t reg_addr)
{
	uint8_t data;
	I2C_MasterSendData(&g_ds1307I2CHandle,&reg_addr,1, DS1307_I2C_ADDRESS);
	I2C_MasterReceiveData(&g_ds1307I2CHandle, &data, 1, DS1307_I2C_ADDRESS);
    return data;

}

static uint8_t binary_to_bcd(uint8_t value)
{
  uint8_t m,n,bcd;
  bcd = value;
  if(value >= 10)
  {
	  m = value/10;
	  n = value %10;
	  bcd = (uint8_t) ((m<<4) | n);

  }

  return bcd;

}
static uint8_t bcd_to_binary(uint8_t value)
{
      uint8_t m,n;
	  m = (uint8_t)((value >> 4) * 10);
	  n = value & 0x0F;



  return (m+n);

}
