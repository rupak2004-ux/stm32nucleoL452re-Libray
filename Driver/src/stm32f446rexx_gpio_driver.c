/*
 * stm32f446rexx_gpio_driver.c
 *
 *  Created on: Jul 21, 2025
 *      Author: rupak
 */
#include <stm32l452rexx.h>
#include <stm32l452rexx_gpio_driver.h>

//driver api required functions

//GPIO INIT AND DEINIT
void GPIO_INIT(GPIO_Handle_t *pGPIOHANDLE)
{
	uint32_t temp = 0;

	// 1. Configure the mode of the GPIO pin
	if(pGPIOHANDLE->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// NON-INTERRUPT MODE
		temp = (pGPIOHANDLE->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber));

		
		pGPIOHANDLE->PGPIOX->MODER &= ~(0x3 << (2 * pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHANDLE->PGPIOX->MODER |= temp;
	}
	else
	{
		// INTERRUPT MODE

	}

	temp = 0;

	// 2. Configure the speed
	temp = (pGPIOHANDLE->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHANDLE->PGPIOX->OSPEEDR &= ~(0x3 << (2 * pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHANDLE->PGPIOX->OSPEEDR |= temp;

	temp = 0;

	// 3. Configure the pull-up/pull-down
	temp = (pGPIOHANDLE->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHANDLE->PGPIOX->PUPDR &= ~(0x3 << (2 * pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHANDLE->PGPIOX->PUPDR |= temp;

	temp = 0;

	// 4. Configure the output type
	temp = (pGPIOHANDLE->GPIO_PinConfig.GPIO_PinOPType << pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHANDLE->PGPIOX->OTYPER &= ~(0x1 << pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHANDLE->PGPIOX->OTYPER |= temp;

	temp = 0;

	// 5. Configure the alternate functionality
	if(pGPIOHANDLE->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber / 8;  // AFR[0] or AFR[1]
		temp2 = pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber % 8;  // Position within AFR

		pGPIOHANDLE->PGPIOX->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHANDLE->PGPIOX->AFR[temp1] |= (pGPIOHANDLE->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

void GPIO_DEINIT(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}

			else if(pGPIOx == GPIOB)
					{
						GPIOB_REG_RESET();
					}
			else if(pGPIOx == GPIOC)
					{
						GPIOC_REG_RESET();
					}
			else if(pGPIOx == GPIOD)
					{
						GPIOD_REG_RESET();
					}
			else if(pGPIOx == GPIOE)
					{
				GPIOE_REG_RESET();
					}

			else if(pGPIOx == GPIOH)
						{
				GPIOH_REG_RESET();
						}
}

// GPIO CLK CONTROL
void GPIO_PERICLK_CTRL(GPIO_RegDef_t *pGPIOx,uint8_t ENORDI)
{
	if(ENORDI == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN;
		}

		else if(pGPIOx == GPIOB)
				{
					GPIOB_PCLK_EN;
				}
		else if(pGPIOx == GPIOC)
				{
					GPIOC_PCLK_EN;
				}
		else if(pGPIOx == GPIOD)
				{
					GPIOD_PCLK_EN;
				}
		else if(pGPIOx == GPIOE)
				{
			GPIOE_PCLK_EN;
				}

		else if(pGPIOx == GPIOH)
					{
			GPIOH_PCLK_EN;
					}
	}
	else
	{
		if(pGPIOx == GPIOA)
				{
               GPIOA_PCLK_DI;
				}
				else if(pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI;
				}
				else if(pGPIOx == GPIOC)
						{
							GPIOC_PCLK_DI;
						}

				else if(pGPIOx == GPIOD)
						{
							GPIOD_PCLK_DI;
						}
				else if(pGPIOx == GPIOE)
						{
					GPIOE_PCLK_DI;
						}

				else if(pGPIOx == GPIOH)
							{
					GPIOH_PCLK_DI;
							}

	}

}

//GPIO READ AND WRITE
uint8_t GPIO_READFROM_INPUT_PIN(GPIO_RegDef_t *pGPIOx,uint8_t pinnumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> pinnumber) & 0x00000001);
	return value;

}
uint32_t GPIO_READFROM_INPUT_PORT(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
		value = (uint16_t)pGPIOx->IDR;
		return value;

}
void GPIO_WRITETO_OUTPUT_PIN(GPIO_RegDef_t *pGPIOx,uint8_t pinnumber,uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		// write 1 to the odr at the bit coreesponding to the pin number
		pGPIOx->ODR |= (1<< pinnumber);
	}
	else
	{
		// else clear that bit
		pGPIOx->ODR &= ~(1<< pinnumber);

	}

}
void GPIO_WRITETO_OUTPUT_PORT(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
  pGPIOx->ODR = value;
}
void GPIO_TOGGLEOUTPUTPIN(GPIO_RegDef_t *pGPIOx,uint8_t pinnumber )
{
	pGPIOx->ODR ^= (1<<pinnumber);

}

//GPIO ISR HANDLING AND IRQ CONFIG
void IRQ_CONFIG(uint8_t IRQNUMBER,uint8_t IRQPRIORITY,uint8_t ENORDI)
{

}
void IRQ_HANDLING(uint8_t pinnumber )
{

}



