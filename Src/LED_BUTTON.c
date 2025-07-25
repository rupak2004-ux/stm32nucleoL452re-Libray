/*
 * LED_BUTTON.C
 *
 *  Created on: Jul 25, 2025
 *      Author: HP
 */


#include <stm32l452rexx.h>

#define LOW  0
#define BTN_PRESS  LOW
void delay(void)
{
	for(uint32_t i =0; i<500000/3;i++);
}
int main(void)
{
	GPIO_Handle_t gpioled,GPIO_BTN;
	gpioled.PGPIOX = GPIOB;
	gpioled.GPIO_PinConfig.GPIO_PinNumber =  GPIO_PIN_NO_13;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PERICLK_CTRL(GPIOB, ENABLE);
  GPIO_INIT(&gpioled);

  GPIO_BTN.PGPIOX = GPIOC;
  GPIO_BTN.GPIO_PinConfig.GPIO_PinNumber =  GPIO_PIN_NO_13;
  GPIO_BTN.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
  GPIO_BTN.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  //GPIO_BTN.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
  GPIO_BTN.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

  GPIO_PERICLK_CTRL(GPIOC, ENABLE);
  GPIO_INIT(&GPIO_BTN);




  while(1)
  {
	  if(GPIO_READFROM_INPUT_PIN(GPIOC, GPIO_PIN_NO_13) != BTN_PRESS)
	  {
		  delay();
	  GPIO_TOGGLEOUTPUTPIN(GPIOB, GPIO_PIN_NO_13);

	  }

  }

	return 0;

}
