/*
 * stm32f446rexx_gpio_driver.h
 *
 *  Created on: Jul 21, 2025
 *      Author: rupak
 */

#ifndef INC_STM32L452REXX_GPIO_DRIVER_H_
#define INC_STM32L452REXX_GPIO_DRIVER_H_
#include <stm32l452rexx.h>

typedef struct {
    uint8_t GPIO_PinNumber;        // possible values from 0 to 15
    uint8_t GPIO_PinMode;          // Input, Output, Alt Function, Analog
    uint8_t GPIO_PinSpeed;         // Low, Medium, Fast, High
    uint8_t GPIO_PinPuPdControl;   // No pull, Pull-up, Pull-down
    uint8_t GPIO_PinOPType;        // Push-pull or Open-drain
    uint8_t GPIO_PinAltFunMode;    // Alternate function mode
} GPIO_PinConfig;



typedef struct
{
	GPIO_RegDef_t *PGPIOX;   //HOLDS THE BASE ADDRESS OF THE GPIO PORT FOR WHERE THE PIN BELONGD
	GPIO_PinConfig GPIO_PinConfig ;
}GPIO_Handle_t;


// GPIO PIN NUMBERS
#define GPIO_PIN_NO_0    0
#define GPIO_PIN_NO_1    1
#define GPIO_PIN_NO_2    2
#define GPIO_PIN_NO_3    3
#define GPIO_PIN_NO_4    4
#define GPIO_PIN_NO_5    5
#define GPIO_PIN_NO_6    6
#define GPIO_PIN_NO_7    7
#define GPIO_PIN_NO_8    8
#define GPIO_PIN_NO_9    9
#define GPIO_PIN_NO_10    10
#define GPIO_PIN_NO_11   11
#define GPIO_PIN_NO_12   12
#define GPIO_PIN_NO_13   13
#define GPIO_PIN_NO_14   14
#define GPIO_PIN_NO_15    15



// gpio PIN  mode
#define GPIO_MODE_IN   0
#define GPIO_MODE_OUT  1
#define GPIO_MODE_ALTFN   2
#define GPIO_MODE_ANALOG   3
#define GPIO_MODE_IT_FT   4   // INTERRUPT FALLING EDGE TRIGGERED
#define GPIO_MODE_IT_RT   5   // INTERRUPT RISING EDGE TRIGGERED
#define GPIO_MODE_RFT     6    // INTERRUPT RISING AND FALLING EDGE TRIGGERED


// GPIO OUTPUT TYPE
#define GPIO_OP_TYPE_PP  0
#define GPIO_OP_TYPE_OD  1

//GPIO OUTPUT SPEED REGISTER
#define GPIO_SPEED_LOW   0
#define GPIO_SPEED_MED   1
#define GPIO_SPEED_FAST  2
#define GPIO_SPEED_HIGH  3

//GPIO PULLUP AND PULL DOWN CONFIGURATION
#define GPIO_NO_PUPD        0
#define GPIO_PIN_PU         1
#define GPIO_PIN_PD         2


//driver api required functions

//GPIO INIT AND DEINIT
void GPIO_INIT(GPIO_Handle_t *pGPIOHANDLE);
void GPIO_DEINIT(GPIO_RegDef_t *pGPIOx);

// GPIO CLK CONTROL
void GPIO_PERICLK_CTRL(GPIO_RegDef_t *pGPIOx,uint8_t ENORDI);

//GPIO READ AND WRITE
uint8_t GPIO_READFROM_INPUT_PIN(GPIO_RegDef_t *pGPIOx,uint8_t pinnumber);
uint32_t GPIO_READFROM_INPUT_PORT(GPIO_RegDef_t *pGPIOx);
void GPIO_WRITETO_OUTPUT_PIN(GPIO_RegDef_t *pGPIOx,uint8_t pinnumber,uint8_t value);
void GPIO_WRITETO_OUTPUT_PORT(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_TOGGLEOUTPUTPIN(GPIO_RegDef_t *pGPIOx,uint8_t pinnumber );

//GPIO ISR HANDLING AND IRQ CONFIG
void IRQ_CONFIG(uint8_t IRQNUMBER,uint8_t IRQPRIORITY,uint8_t ENORDI);
void IRQ_HANDLING(uint8_t pinnumber );

#endif /* INC_STM32L452REXX_GPIO_DRIVER_H_ */
