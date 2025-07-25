/*
 * stm32f446xx.h
 *
 *  Created on: Jul 21, 2025
 *      Author: rupak
 */


#ifndef INC_STM32L452REXX_H_
#define INC_STM32L452REXX_H_
#include <stdint.h>

#define _vo  volatile

// base address of flash and sram memories
#define   FLASH_BASEADDR       0x08000000U
#define  SRAM1_BASEADDR        0x20000000U
#define SRAM2_BASEADDR         0x10000000U
#define ROM_BASEADDR           0x1FFF0000U
#define SRAM_BASEADDR           SRAM1_BASEADDR

// BASE ADDRESSES FOR THE BUSES THAT ARE AVILABLE
#define PERIPH_BASEADDR             0x40000000U
#define APB1PERIPH_BASEADDR         PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR         0x40010000U
#define AHB1PERIPH_BASEADDR          0x40020000U
#define AHB2PERIPH_BASEADDR          0x48000000U


// BASE ADDRESSES OF THE PERIPHERALS THATAR HANGING ON THE APB1

#define TIM2_BASEADDR           (APB1PERIPH_BASEADDR + 0x0000U)
#define TIM3_BASEADDR           (APB1PERIPH_BASEADDR + 0x0400U)
// 0x0800 - 0x0FFF: Reserved
#define TIM6_BASEADDR           (APB1PERIPH_BASEADDR + 0x1000U)
#define TIM7_BASEADDR           (APB1PERIPH_BASEADDR + 0x1400U)
// 0x1800 - 0x23FF: Reserved
#define LCD_BASEADDR            (APB1PERIPH_BASEADDR + 0x2400U)
#define RTC_BASEADDR            (APB1PERIPH_BASEADDR + 0x2800U)
#define WWDG_BASEADDR           (APB1PERIPH_BASEADDR + 0x2C00U)
#define IWDG_BASEADDR           (APB1PERIPH_BASEADDR + 0x3000U)
#define TAMP_BASEADDR           (APB1PERIPH_BASEADDR + 0x3400U)
#define SPI2_BASEADDR           (APB1PERIPH_BASEADDR + 0x3800U)
#define SPI3_BASEADDR           (APB1PERIPH_BASEADDR + 0x3C00U)
// 0x4000 - 0x43FF: Reserved
#define USART2_BASEADDR         (APB1PERIPH_BASEADDR + 0x4400U)
#define USART3_BASEADDR         (APB1PERIPH_BASEADDR + 0x4800U)
#define UART4_BASEADDR          (APB1PERIPH_BASEADDR + 0x4C00U)
// 0x5000 - 0x53FF: Reserved
#define I2C1_BASEADDR           (APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR           (APB1PERIPH_BASEADDR + 0x5800U)
#define I2C3_BASEADDR           (APB1PERIPH_BASEADDR + 0x5C00U)
#define CRS_BASEADDR            (APB1PERIPH_BASEADDR + 0x6000U)
#define CAN1_BASEADDR           (APB1PERIPH_BASEADDR + 0x6400U)
#define USB_FS_BASEADDR         (APB1PERIPH_BASEADDR + 0x6800U)
#define USB_SRAM_BASEADDR       (APB1PERIPH_BASEADDR + 0x6C00U)
#define PWR_BASEADDR            (APB1PERIPH_BASEADDR + 0x7000U)
#define DAC1_BASEADDR           (APB1PERIPH_BASEADDR + 0x7400U)
#define OPAMP_BASEADDR          (APB1PERIPH_BASEADDR + 0x7800U)
#define LPTIM1_BASEADDR         (APB1PERIPH_BASEADDR + 0x7C00U)
#define LPUART1_BASEADDR        (APB1PERIPH_BASEADDR + 0x8000U)
#define I2C4_BASEADDR           (APB1PERIPH_BASEADDR + 0x8400U)
#define SWPMI1_BASEADDR         (APB1PERIPH_BASEADDR + 0x8800U)
// 0x8C00 - 0x93FF: Reserved
#define LPTIM2_BASEADDR         (APB1PERIPH_BASEADDR + 0x9400U)
// 0x9800 - 0xFFFF: Reserved

// BASE ADDRESSES OF THE PERIPHERALS THATAR HANGING ON THE APB2
#define SYSCFG_BASEADDR         (APB2PERIPH_BASEADDR + 0x0000U)  // 0x4001 0000
#define VREFBUF_BASEADDR        (APB2PERIPH_BASEADDR + 0x0030U)  // 0x4001 0030
#define COMP_BASEADDR           (APB2PERIPH_BASEADDR + 0x0200U)  // 0x4001 0200
#define EXTI_BASEADDR           (APB2PERIPH_BASEADDR + 0x0400U)  // 0x4001 0400
// 0x0800 - 0x1BFF Reserved
#define FIREWALL_BASEADDR       (APB2PERIPH_BASEADDR + 0x1C00U)  // 0x4001 1C00
// 0x2000 - 0x27FF Reserved
#define SDMMC_BASEADDR          (APB2PERIPH_BASEADDR + 0x2800U)  // 0x4001 2800
#define TIM1_BASEADDR           (APB2PERIPH_BASEADDR + 0x2C00U)  // 0x4001 2C00
#define SPI1_BASEADDR           (APB2PERIPH_BASEADDR + 0x3000U)  // 0x4001 3000
// 0x3400 - 0x37FF Reserved
#define USART1_BASEADDR         (APB2PERIPH_BASEADDR + 0x3800U)  // 0x4001 3800
// 0x3C00 - 0x3FFF Reserved
#define TIM15_BASEADDR          (APB2PERIPH_BASEADDR + 0x4000U)  // 0x4001 4000
#define TIM16_BASEADDR          (APB2PERIPH_BASEADDR + 0x4400U)  // 0x4001 4400
// 0x4800 - 0x53FF Reserved
#define SAI1_BASEADDR           (APB2PERIPH_BASEADDR + 0x5400U)  // 0x4001 5400
// 0x5800 - 0x5FFF Reserved
#define DFSDM1_BASEADDR         (APB2PERIPH_BASEADDR + 0x6000U)  // 0x4001 6000
// 0x6400 - 0xFFFF Reserved


// BASE ADDRESSES OF THE PERIPHERALS THATAR HANGING ON THE AHB1
#define DMA1_BASEADDR           (AHB1PERIPH_BASEADDR + 0x0000U)  // 0x4002 0000
#define DMA2_BASEADDR           (AHB1PERIPH_BASEADDR + 0x0400U)  // 0x4002 0400
// 0x0800 - 0x0FFF: Reserved
#define RCC_BASEADDR            (AHB1PERIPH_BASEADDR + 0x1000U)  // 0x4002 1000
// 0x1400 - 0x1FFF: Reserved
// 0x2400 - 0x2FFF: Reserved
#define CRC_BASEADDR            (AHB1PERIPH_BASEADDR + 0x3000U)  // 0x4002 3000
// 0x3400 - 0x3FFF: Reserved
#define TSC_BASEADDR            (AHB1PERIPH_BASEADDR + 0x4000U)  // 0x4002 4000


//base address of the peripherals hanging on the  ahb2
#define GPIOA_BASEADDR          (AHB2PERIPH_BASEADDR + 0x0000U)  // 0x4800 0000
#define GPIOB_BASEADDR          (AHB2PERIPH_BASEADDR + 0x0400U)  // 0x4800 0400
#define GPIOC_BASEADDR          (AHB2PERIPH_BASEADDR + 0x0800U)  // 0x4800 0800
#define GPIOD_BASEADDR          (AHB2PERIPH_BASEADDR + 0x0C00U)  // 0x4800 0C00
#define GPIOE_BASEADDR          (AHB2PERIPH_BASEADDR + 0x1000U)  // 0x4800 1000
// 0x1400 - 0x1BFF Reserved
#define GPIOH_BASEADDR          (AHB2PERIPH_BASEADDR + 0x1C00U)  // 0x4800 1C00


// PERIPHERAL register definition structure for the gpio

typedef struct
{
	_vo uint32_t MODER;     // 0x00: GPIO port mode register
	_vo uint32_t OTYPER;    // 0x04: GPIO port output type register
	_vo uint32_t OSPEEDR;   // 0x08: GPIO port output speed register
    _vo uint32_t PUPDR;     // 0x0C: GPIO port pull-up/pull-down register
    _vo uint32_t IDR;       // 0x10: GPIO port input data register (read-only)
    _vo uint32_t ODR;       // 0x14: GPIO port output data register
    _vo uint32_t BSRR;      // 0x18: GPIO port bit set/reset register (write-only)
    _vo uint32_t LCKR;      // 0x1C: GPIO port configuration lock register
    _vo uint32_t AFR[2];    // 0x20: AFR[0] = AFRL (pins 0-7), 0x24: AFR[1] = AFRH (pins 8-15)
    _vo uint32_t BRR;      //0X24 BIT RESET REGISTER
} GPIO_RegDef_t;



// PERIPHERALS GPIOA BASE ADDRESSES TYPECASTED TO THE REGDEF
#define GPIOA        ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB        ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC        ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD        ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE        ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH        ((GPIO_RegDef_t*)GPIOH_BASEADDR)



// peripheral definition type structure for the rcc

typedef struct {
    volatile uint32_t CR;           // 0x00: Clock control register
    volatile uint32_t ICSCR;        // 0x04: Internal clock sources calibration register
    volatile uint32_t CFGR;         // 0x08: Clock configuration register
    volatile uint32_t PLLCFGR;      // 0x0C: PLL configuration register
    volatile uint32_t PLLSAI1CFGR;  // 0x10: PLL SAI1 configuration register
    uint32_t RESERVED1;             // 0x14: Reserved
    volatile uint32_t CIER;         // 0x18: Clock interrupt enable register
    volatile uint32_t CIFR;         // 0x1C: Clock interrupt flag register
    volatile uint32_t CICR;         // 0x20: Clock interrupt clear register
    uint32_t RESERVED2;             // 0x24: Reserved
    volatile uint32_t AHB1RSTR;     // 0x28: AHB1 peripheral reset register
    volatile uint32_t AHB2RSTR;     // 0x2C: AHB2 peripheral reset register
    volatile uint32_t AHB3RSTR;     // 0x30: AHB3 peripheral reset register
    uint32_t RESERVED3;             // 0x34: Reserved
    volatile uint32_t APB1RSTR1;    // 0x38: APB1 peripheral reset register 1
    volatile uint32_t APB1RSTR2;    // 0x3C: APB1 peripheral reset register 2
    volatile uint32_t APB2RSTR;     // 0x40: APB2 peripheral reset register
    uint32_t RESERVED4;             // 0x44: Reserved
    volatile uint32_t AHB1ENR;      // 0x48: AHB1 peripheral clock enable register
    volatile uint32_t AHB2ENR;      // 0x4C: AHB2 peripheral clock enable register
    volatile uint32_t AHB3ENR;      // 0x50: AHB3 peripheral clock enable register
    uint32_t RESERVED5;             // 0x54: Reserved
    volatile uint32_t APB1ENR1;     // 0x58: APB1 peripheral clock enable register 1
    volatile uint32_t APB1ENR2;     // 0x5C: APB1 peripheral clock enable register 2
    volatile uint32_t APB2ENR;      // 0x60: APB2 peripheral clock enable register
    uint32_t RESERVED6;             // 0x64: Reserved
    volatile uint32_t AHB1SMENR;    // 0x68: AHB1 sleep mode clock enable register
    volatile uint32_t AHB2SMENR;    // 0x6C: AHB2 sleep mode clock enable register
    volatile uint32_t AHB3SMENR;    // 0x70: AHB3 sleep mode clock enable register
    uint32_t RESERVED7;             // 0x74: Reserved
    volatile uint32_t APB1SMENR1;   // 0x78: APB1 sleep mode clock enable register 1
    volatile uint32_t APB1SMENR2;   // 0x7C: APB1 sleep mode clock enable register 2
    volatile uint32_t APB2SMENR;    // 0x80: APB2 sleep mode clock enable register
    uint32_t RESERVED8;             // 0x84: Reserved
    volatile uint32_t CCIPR;        // 0x88: Peripherals independent clock configuration register
    uint32_t RESERVED9;             // 0x8C: Reserved
    volatile uint32_t BDCR;         // 0x90: Backup domain control register
    volatile uint32_t CSR;          // 0x94: Control/status register
    volatile uint32_t CRRCR;        // 0x98: Clock recovery RC register
    volatile uint32_t CCIPR2;       // 0x9C: Peripherals independent clock configuration register 2
} RCC_RegDef_t;


#define RCC     ((RCC_RegDef_t*)RCC_BASEADDR)


// CLOCK ENABLE MACROS FOR THE GPIOX
#define GPIOA_PCLK_EN         (RCC->AHB2ENR |= (1 << 0))
#define GPIOB_PCLK_EN         (RCC->AHB2ENR |= (1 << 1))
#define GPIOC_PCLK_EN         (RCC->AHB2ENR |= (1 << 2))
#define GPIOD_PCLK_EN         (RCC->AHB2ENR |= (1 << 3))
#define GPIOE_PCLK_EN         (RCC->AHB2ENR |= (1 << 4))
#define GPIOH_PCLK_EN         (RCC->AHB2ENR |= (1 << 7))


//CLOCK ENABLE FOR THE IZCX
#define I2C1_PCLK_EN      (RCC->APB1ENR1 |= (1 << 21))
#define I2C2_PCLK_EN      (RCC->APB1ENR1 |= (1 << 22))
#define I2C3_PCLK_EN      (RCC->APB1ENR1 |= (1 << 23))

// CLOCK ENABLE FOR THE SPIX
#define SPI1_PCLK_EN      (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN      (RCC->APB1ENR1 |= (1 << 14))
#define SPI3_PCLK_EN      (RCC->APB1ENR1 |= (1 << 15))


//CLOCK ENABLE FOR THE USARTX
#define USART1_PCLK_EN    (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN    (RCC->APB1ENR1 |= (1 << 17))
#define USART3_PCLK_EN    (RCC->APB1ENR1 |= (1 << 18))
#define UART4_PCLK_EN     (RCC->APB1ENR1 |= (1 << 19))



//CLOCK ENABLE FOR THE SYSCFG PERIPHERALS
#define SYSCFG_PCLK_EN    (RCC->APB2ENR |= (1 << 0))




// DISABLE CLOCK FOR THE GPIOX
#define GPIOA_PCLK_DI         (RCC->AHB2ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI         (RCC->AHB2ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI         (RCC->AHB2ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI         (RCC->AHB2ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI         (RCC->AHB2ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI         (RCC->AHB2ENR &= ~(1 << 7))


//CLOCK DISABLE FOR THE IZCX

#define I2C1_PCLK_DI      (RCC->APB1ENR1 &= ~(1 << 21))
#define I2C2_PCLK_DI      (RCC->APB1ENR1 &= ~(1 << 22))
#define I2C3_PCLK_DI      (RCC->APB1ENR1 &= ~(1 << 23))

// CLOCK DISABLE FOR THE SPIX

#define SPI1_PCLK_DI      (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI      (RCC->APB1ENR1 &= ~(1 << 14))
#define SPI3_PCLK_DI      (RCC->APB1ENR1 &= ~(1 << 15))

//CLOCK DISABLE FOR THE USARTX
#define USART1_PCLK_DI    (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI    (RCC->APB1ENR1 &= ~(1 << 17))
#define USART3_PCLK_DI    (RCC->APB1ENR1 &= ~(1 << 18))
#define UART4_PCLK_DI     (RCC->APB1ENR1 &= ~(1 << 19))

//CLOCK DISABLE FOR THE SYSCFG PERIPHERALS
#define SYSCFG_PCLK_DI    (RCC->APB2ENR &=~ (1 << 0))

// SOME GENERIC MACROS
#define ENABLE 1
#define DISABLE 0
#define SET   ENABLE
#define RESET  DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET  RESET


// MACRO FOR THE GPIO  RESET'
#define GPIOA_REG_RESET()                     do{(RCC->AHB2RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));} while(0)
#define GPIOB_REG_RESET()                     do{(RCC->AHB2RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));} while(0)
#define GPIOC_REG_RESET()                     do{(RCC->AHB2RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));} while(0)
#define GPIOD_REG_RESET()                     do{(RCC->AHB2RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));} while(0)
#define GPIOE_REG_RESET()                     do{(RCC->AHB2RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));} while(0)
#define GPIOH_REG_RESET()                     do{(RCC->AHB2RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));} while(0)


#include <stm32l452rexx_gpio_driver.h>
#endif /* INC_STM32L452REXX_H_ */
