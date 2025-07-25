/*
 * ledtoogle.c
 *
 *  Created on: Jul 23, 2025
 *      Author: rupak
 */


#include <stm32l452rexx.h>

void delay(void)
{
	for(uint32_t i =0; i<500000/4;i++);
}
int main(void)
{
	GPIO_Handle_t gpioled;
	gpioled.PGPIOX = GPIOB;
	gpioled.GPIO_PinConfig.GPIO_PinNumber =  GPIO_PIN_NO_13;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PERICLK_CTRL(GPIOB, ENABLE);
  GPIO_INIT(&gpioled);
  while(1)
  {
	  GPIO_TOGGLEOUTPUTPIN(GPIOB, GPIO_PIN_NO_13);
	  delay();
  }


	return 0;

}

