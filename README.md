# STM32-F401xx-Drivers

This repository contains a collection of drivers and libraries for the STM32F401RE microcontroller. These drivers were written using the reference manual and provide an easy-to-use interface for controlling the various peripherals of the microcontroller.

## Installation

To use these drivers in your project, simply clone this repository into your project directory:


You can then include the relevant driver files in your project and configure them according to your needs.

## Available Drivers

The following drivers are included in this repository:

- MCU-specific driver
- GPIO driver
- I2C driver
- USART driver
- SPI driver
- RCC driver
- DS1307 library and driver

## Usage

To use a driver in your project, include its header file and use the provided functions to configure and control the peripheral. For example, to use the GPIO driver:

```c
/*
 * 002_LED_BUTTON.c
 *
 *  Created on: 8 Mar 2023
 *      Author: Mustafa
 */
#include "stm32f401xx.h"
void delay(void)
{
 for(uint32_t i=0;i<500000;i++);
}
int main(void)
{
	GPIO_Handle_t GpioLed,Gpio_UserButton;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioLed);

	Gpio_UserButton.pGPIOx = GPIOC;
	Gpio_UserButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	Gpio_UserButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	Gpio_UserButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	Gpio_UserButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;
	GPIO_Init(&Gpio_UserButton);

while(1)
{
	if(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_0))
    {
		while(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_0));
		delay();
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_0);
    }

}

  return 0;
}
```
## Contributing
If you find a bug or would like to contribute to this project, please open an issue or a pull request. Contributions are welcome and appreciated!
